/*
* Change Logs:
* Date            Author          Notes
* 2023-10-09      ChenSihan     1.0.0version 发射线程模块
* 2023-10-14      ChenSihan     发射逻辑优化 pid优化
*/
#include "shoot_task.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "rm_task.h"

#define DBG_TAG   "rm.task"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

//TODO: 弹频和弹速的控制应在cmd线程中决策

/* -------------------------------- 线程间通讯话题相关 ------------------------------- */
static struct shoot_cmd_msg shoot_cmd;
static struct shoot_fdb_msg shoot_fdb;

static publisher_t *pub_shoot;
static subscriber_t *sub_cmd;

static void shoot_pub_init(void);
static void shoot_sub_init(void);
static void shoot_pub_push(void);
static void shoot_sub_pull(void);
/* -------------------------------- 线程间通讯话题相关 ------------------------------- */

/*舵机pwm设备*/
static struct rt_device_pwm *servo_cover_dev;  // 弹仓盖舵机
static struct rt_device_pwm *servo_mirror_dev;  //倍镜舵机

//转子角度标志位，防止切换设计模式时拨弹电机反转
static int total_angle_flag=SHOOT_ANGLE_CONTINUE;
/*发射模块电机使用数量*/
//#define SHT_MOTOR_NUM 3
#define SHT_MOTOR_NUM 4

/*发射模块电机编号：分别为左摩擦轮电机 右摩擦轮电机 拨弹电机*/
#define RIGHT_FRICTION 0
#define LEFT_FRICTION 1
#define MIDDLE_FRICTION 2
#define TRIGGER_MOTOR 3

//枪口热量相关参数
#define HEATADD 100  //英雄一发子弹热量为100
#define MAXHEAT 400
#define COOL 40

/*pid环数结构体*/
static struct shoot_controller_t{
    pid_obj_t *pid_speed;
    pid_obj_t *pid_angle;
}sht_controller[SHT_MOTOR_NUM];

/*电机注册初始化数据*/
motor_config_t shoot_motor_config[SHT_MOTOR_NUM] ={
    {
        .motor_type = M3508,
        .can_name = CAN_GIMBAL,
        .rx_id = RIGHT_FRICTION_MOTOR_ID,
        .controller = &sht_controller[RIGHT_FRICTION],
    },
    {
        .motor_type = M3508,
        .can_name = CAN_GIMBAL,
        .rx_id = LEFT_FRICTION_MOTOR_ID,
        .controller = &sht_controller[LEFT_FRICTION],
    },
    {
        .motor_type = M3508,
        .can_name = CAN_GIMBAL,
        .rx_id = MIDDLE_FRICTION_MOTOR_ID,
        .controller = &sht_controller[MIDDLE_FRICTION],
    },
    {
        .motor_type = M3508,
        .can_name = CAN_GIMBAL,
        .rx_id = TRIGGER_MOTOR_ID,
        .controller = &sht_controller[TRIGGER_MOTOR],
    }
};

static dji_motor_object_t *sht_motor[SHT_MOTOR_NUM];  // 发射器电机实例
static float shoot_motor_ref[SHT_MOTOR_NUM]; // 电机控制期望值

/*函数声明*/
static void shoot_motor_init();
//static void servo_init();
static rt_int16_t motor_control_right(dji_motor_measure_t measure);
static rt_int16_t motor_control_middle(dji_motor_measure_t measure);
static rt_int16_t motor_control_left(dji_motor_measure_t measure);
static rt_int16_t motor_control_trigger(dji_motor_measure_t measure);
static void shoot_self_cmd(void);
static void shoot_self_cmd_Init(void);
static void self_shoot_msg_restrict(void);
static void servo_init();

static char referee_status=1;
typedef struct {
    rt_bool_t referee_flag;
    uint8_t bullet_cnt;//转45度发射一颗子弹
    uint8_t bullet_heat;
    uint8_t last_angle;
    uint8_t now_angle;
    float last_dt;//通过get delta ms计算频率？
    float now_dt;
    risk_level_e risk_level;//0:正常,1:危险，2：停止射击
    int8_t remain_bullet;//还能打多少颗子弹，应该存在负数情况，所以用int定义
}self_shoot_info_e;
static self_shoot_info_e self_shoot_info;
/* --------------------------------- 射击线程入口 --------------------------------- */
static float sht_dt;
static int shoot_cnt;
/**
 * @brief shoot线程入口函数
 */
void shoot_task_entry(void* argument)
{
    static float sht_start;
    static int servo_cvt_num;
    static int reverse_cnt;
    static float sht_gap_time;
    static float sht_gap_start_time;

    shoot_motor_init();
    shoot_pub_init();
    shoot_sub_init();
    servo_init();
/*----------------------射击状态初始化----------------------------------*/
    shoot_cmd.ctrl_mode=SHOOT_STOP;
    shoot_cmd.trigger_status=TRIGGER_OFF;
    shoot_motor_ref[TRIGGER_MOTOR]=0;
    LOG_I("Shoot Task Start");
    for (;;)
    {
        sht_start = dwt_get_time_ms();
        /* 更新该线程所有的订阅者 */
        shoot_sub_pull();

        // /* 弹仓盖舵机控制 */
        // if (shoot_cmd.cover_open == 1)
        //     rt_pwm_set(servo_cover_dev, PWM_COVER_CH, 20000000, 2000000);
        // else
        //     rt_pwm_set(servo_cover_dev, PWM_COVER_CH, 20000000, 750000);
        /* 倍镜舵机控制 */
        if (shoot_cmd.mirror_enable == 1)
            rt_pwm_set(servo_mirror_dev, PWM_MIRROR_CH, 20000000, 500000);
        else
            rt_pwm_set(servo_mirror_dev, PWM_MIRROR_CH, 20000000, 2500000);

        /* 电机控制启动 */
        for (uint8_t i = 0; i < SHT_MOTOR_NUM; i++)
        {
            dji_motor_enable(sht_motor[i]);
        }

        //检测是否堵弹，堵弹反转一次
       /* if (sht_motor[TRIGGER_MOTOR]->measure.real_current>=8000||reverse_cnt!=0)
        {
           shoot_cmd.ctrl_mode=SHOOT_REVERSE;
            if (reverse_cnt<100)
                reverse_cnt++;
            else
                reverse_cnt=0;
        }*/
         shoot_fdb.trigger_motor_current=sht_motor[TRIGGER_MOTOR]->measure.real_current;
        /*控制模式判断*/
        /*subs遥控器*/
#ifdef BSP_USING_RC_SBUS
        switch (shoot_cmd.ctrl_mode)
        {
            case SHOOT_STOP:
                shoot_motor_ref[TRIGGER_MOTOR] = 0;
                shoot_motor_ref[RIGHT_FRICTION] =0;
                shoot_motor_ref[LEFT_FRICTION] = 0;
                total_angle_flag=0;
                break;

            case SHOOT_ONE:
                shoot_motor_ref[RIGHT_FRICTION] = 5000;//摩擦轮常转
                shoot_motor_ref[LEFT_FRICTION] = -5000;
                /*从自动连发模式切换三连发及单发模式时，要继承总转子角度*/
                if(total_angle_flag == 0)
                {
                    shoot_motor_ref[TRIGGER_MOTOR]= sht_motor[TRIGGER_MOTOR]->measure.total_angle;
                    total_angle_flag=1;
                }
                if (shoot_cmd.trigger_status == TRIGGER_ON)
                {
                    shoot_motor_ref[TRIGGER_MOTOR]= shoot_motor_ref[TRIGGER_MOTOR] + TRIGGER_MOTOR_45_TO_ANGLE * 36;//M2006的减速比为36:1，因此转轴旋转45度，要在转子的基础上乘36倍
                    shoot_cmd.trigger_status=TRIGGER_OFF;//扳机归零
                }
                shoot_fdb.trigger_status=SHOOT_OK;
                break;

            case SHOOT_THREE:
                shoot_motor_ref[RIGHT_FRICTION] = 5000;//摩擦轮常转
                shoot_motor_ref[LEFT_FRICTION] = -5000;
                /*从自动连发模式切换三连发及单发模式时，要继承总转子角度*/
                if(total_angle_flag == 0)
                {
                    shoot_motor_ref[TRIGGER_MOTOR]= sht_motor[TRIGGER_MOTOR]->measure.total_angle;
                    total_angle_flag = 1;
                }
                if (shoot_cmd.trigger_status == TRIGGER_ON)
                {
                    shoot_motor_ref[TRIGGER_MOTOR]= shoot_motor_ref[TRIGGER_MOTOR] + 3 * TRIGGER_MOTOR_45_TO_ANGLE * 36;//M2006的减速比为36:1，因此转轴旋转45度，要在转子的基础上乘36倍
                    shoot_cmd.trigger_status=TRIGGER_OFF;//扳机归零
                }
                shoot_fdb.trigger_status=SHOOT_OK;
                break;

            case SHOOT_COUNTINUE:
                shoot_motor_ref[RIGHT_FRICTION] = 5000;//摩擦轮常转
                shoot_motor_ref[LEFT_FRICTION] = -5000;
                shoot_motor_ref[TRIGGER_MOTOR] = shoot_cmd.shoot_freq;//自动模式的时候，只用速度环控制拨弹电机
                if (shoot_cmd.shoot_freq>=3&&shoot_cmd.shoot_freq<=5)
                {
                    shoot_motor_ref[TRIGGER_MOTOR] = 3500;//自动模式的时候，只用速度环控制拨弹电机
                }
                else if(shoot_cmd.shoot_freq>=5&&shoot_cmd.shoot_freq<8)
                {
                    shoot_motor_ref[TRIGGER_MOTOR] = 5000;
                }
                else if(shoot_cmd.shoot_freq>=8&&shoot_cmd.shoot_freq<10)
                {
                    shoot_motor_ref[TRIGGER_MOTOR] = 7000;
                }
                else
                {
                    shoot_motor_ref[TRIGGER_MOTOR] = 0;
                }
                total_angle_flag = 0;
                shoot_fdb.trigger_status = SHOOT_OK;
                break;

            case SHOOT_REVERSE:
                shoot_motor_ref[RIGHT_FRICTION] = 5000;//摩擦轮常转
                shoot_motor_ref[LEFT_FRICTION] = -5000;
                shoot_motor_ref[TRIGGER_MOTOR]=  -2500;
                total_angle_flag = 0;
                break;

            case SHOOT_AUTO:
                shoot_motor_ref[RIGHT_FRICTION] = 5000;//摩擦轮常转
                shoot_motor_ref[LEFT_FRICTION] = -5000;
                shoot_motor_ref[TRIGGER_MOTOR]=  -2500;
                total_angle_flag = 0;
                break;

            default:
                for (uint8_t i = 0; i < SHT_MOTOR_NUM; i++)
                {
                    dji_motor_relax(sht_motor[i]); // 错误情况电机全部松电
                }
                shoot_fdb.trigger_status=SHOOT_ERR;
                break;
        }
#endif
        /*dubs遥控器*/

        /*开关摩擦轮*/
        if (shoot_cmd.friction_status==1)
        {
            shoot_motor_ref[RIGHT_FRICTION] = 8500;//摩擦轮常转 实测最大转速为8800
            shoot_motor_ref[MIDDLE_FRICTION] = 8500;
            shoot_motor_ref[LEFT_FRICTION] = -8500;
            /*从自动连发模式切换三连发及单发模式时，要继承总转子角度*/
        }
        else
        {
            shoot_motor_ref[TRIGGER_MOTOR] = 0;
            shoot_motor_ref[RIGHT_FRICTION] =0;
            shoot_motor_ref[MIDDLE_FRICTION] = 0;
            shoot_motor_ref[LEFT_FRICTION] = 0;
            total_angle_flag=SHOOT_ANGLE_CONTINUE;
        }
        switch (shoot_cmd.ctrl_mode)
        {
            case SHOOT_STOP:
                shoot_motor_ref[TRIGGER_MOTOR] = 0;
                total_angle_flag=0;
                shoot_fdb.trigger_status=SHOOT_WAITING;
                break;

            case SHOOT_ONE:

                if(shoot_cmd.trigger_status == TRIGGER_OFF)
                {
                    shoot_fdb.trigger_status=SHOOT_WAITING;
                }
                if(total_angle_flag == SHOOT_ANGLE_CONTINUE)
                {
                    shoot_motor_ref[TRIGGER_MOTOR]= sht_motor[TRIGGER_MOTOR]->measure.total_angle;
                    total_angle_flag=SHOOT_ANGLE_SINGLE;
                }
                if (shoot_cmd.trigger_status == TRIGGER_ON)
                {
                    sht_gap_time = dwt_get_time_ms() - sht_gap_start_time;
                    sht_gap_start_time = dwt_get_time_ms();
                    if(sht_gap_time>=200)
                    {
                        shoot_fdb.shoot_cnt++;
                        //M3508的减速比为 19:1，因此转轴旋转51.43度，要在转子的基础上乘19倍
                        shoot_motor_ref[TRIGGER_MOTOR]= shoot_motor_ref[TRIGGER_MOTOR] + TRIGGER_MOTOR_51_TO_ANGLE * 19;
                    }
                    shoot_cmd.trigger_status=TRIGGER_OFF;//扳机归零
                    shoot_fdb.trigger_status=SHOOT_OK;
                }

                break;

            case SHOOT_THREE:
                /*从自动连发模式切换三连发及单发模式时，要继承总转子角度*/
                if(total_angle_flag == SHOOT_ANGLE_CONTINUE)
                {
                    shoot_motor_ref[TRIGGER_MOTOR]= sht_motor[TRIGGER_MOTOR]->measure.total_angle;
                    total_angle_flag = SHOOT_ANGLE_SINGLE;
                }
                if (shoot_cmd.trigger_status == TRIGGER_ON)
                {

                    shoot_motor_ref[TRIGGER_MOTOR]= shoot_motor_ref[TRIGGER_MOTOR] + TRIGGER_MOTOR_51_TO_ANGLE * 19;//M3508的减速比为 19:1，因此转轴旋转51.43度，要在转子的基础上乘19倍
                    shoot_cmd.trigger_status=TRIGGER_OFF;//扳机归零
                    shoot_fdb.trigger_status=SHOOT_OK;
                }

                break;

            case SHOOT_COUNTINUE:
                shoot_motor_ref[RIGHT_FRICTION] = 6000;//摩擦轮常转 实测最大转速为8800
                shoot_motor_ref[MIDDLE_FRICTION] = 6000;
                shoot_motor_ref[LEFT_FRICTION] = -6000;
                shoot_motor_ref[TRIGGER_MOTOR] = shoot_cmd.shoot_freq;//自动模式的时候，只用速度环控制拨弹电机
                total_angle_flag = SHOOT_ANGLE_CONTINUE;
                shoot_fdb.trigger_status= SHOOT_OK;
                break;

            case SHOOT_REVERSE:
                shoot_motor_ref[TRIGGER_MOTOR]= -2500;
                total_angle_flag = 0;
                break;

            default:
                for (uint8_t i = 0; i < SHT_MOTOR_NUM; i++)
                {
                    dji_motor_relax(sht_motor[i]); // 错误情况电机全部松电
                }
                shoot_fdb.trigger_status=SHOOT_ERR;
                break;
        }
        /* 更新发布该线程的msg */
        shoot_pub_push();

        //TODO:单独调试shoot模块时打开，用于更新上个模式
        //shoot_cmd.last_mode=shoot_cmd.ctrl_mode;

        /* 用于调试监测线程调度使用 */
        sht_dt = dwt_get_time_ms() - sht_start;
        if (sht_dt > 1)
            LOG_E("Shoot Task is being DELAY! dt = [%f]", &sht_dt);
        rt_thread_mdelay(1);
    }
}

/**
 * @brief 舵机初始化
 */
//static void servo_init(){
//    servo_cover_dev=(struct rt_device_pwm *) rt_device_find(PWM_COVER);
//    if(servo_cover_dev == RT_NULL)
//    {
//        LOG_E("Can't find cover servo pwm device!");
//        return;
//    }
//    rt_pwm_set(servo_cover_dev, PWM_COVER_CH, 20000000, 780000);
//    rt_pwm_enable(servo_cover_dev, PWM_COVER_CH);
//}

/**
 * @brief 倍镜舵机初始化
 */
static void servo_init(){
    servo_mirror_dev=(struct rt_device_pwm *) rt_device_find(PWM_MIRROR);
    if(servo_mirror_dev == RT_NULL)
    {
        LOG_E("Can't find mirror servo pwm device!");
        return;
    }
    rt_pwm_set(servo_mirror_dev, PWM_MIRROR_CH, 20000000, 250000);
    rt_pwm_enable(servo_mirror_dev, PWM_MIRROR_CH);
}

/**
 * @brief shoot 线程电机初始化
 */
static void shoot_motor_init(){
    /* -------------------------------------- right_friction 右摩擦轮电机 ----------------------------------------- */
    pid_config_t right_speed_config = INIT_PID_CONFIG(RIGHT_KP_V, RIGHT_KI_V, RIGHT_KD_V,RIGHT_INTEGRAL_V,RIGHT_MAX_V,
                                                        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    sht_controller[RIGHT_FRICTION].pid_speed = pid_register(& right_speed_config);

/* ------------------------------------------- left_friction 左摩擦轮电机------------------------------------------------- */
    pid_config_t left_speed_config = INIT_PID_CONFIG(LEFT_KP_V,  LEFT_KI_V, LEFT_KD_V , LEFT_INTEGRAL_V, LEFT_MAX_V,
                                                          (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    sht_controller[LEFT_FRICTION].pid_speed = pid_register(&left_speed_config);

    /* ------------------------------------------- middle_friction 中摩擦轮电机------------------------------------------------- */
    pid_config_t middle_speed_config = INIT_PID_CONFIG(MIDDLE_KP_V,  MIDDLE_KI_V, MIDDLE_KD_V , MIDDLE_INTEGRAL_V, MIDDLE_MAX_V,
                                                     (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    sht_controller[MIDDLE_FRICTION].pid_speed = pid_register(&middle_speed_config);
/* ------------------------------------------------  拨弹电机------------------------------------------------------------------------- */
    pid_config_t toggle_speed_config = INIT_PID_CONFIG(TRIGGER_KP_V  , TRIGGER_KI_V , TRIGGER_KD_V  , TRIGGER_INTEGRAL_V, TRIGGER_MAX_V ,
                                                       (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t toggle_angle_config = INIT_PID_CONFIG(TRIGGER_KP_A, TRIGGER_KI_A, TRIGGER_KD_A, TRIGGER_INTEGRAL_A , TRIGGER_MAX_A ,
                                                       (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    sht_controller[TRIGGER_MOTOR].pid_speed = pid_register(&toggle_speed_config);
    sht_controller[TRIGGER_MOTOR].pid_angle = pid_register(&toggle_angle_config);

/* ---------------------------------- shoot电机初注册---------------------------------------------------------------------------------------- */
    sht_motor[TRIGGER_MOTOR] = dji_motor_register(&shoot_motor_config[TRIGGER_MOTOR], motor_control_trigger);
    sht_motor[LEFT_FRICTION] = dji_motor_register(&shoot_motor_config[LEFT_FRICTION], motor_control_left);
    sht_motor[MIDDLE_FRICTION] = dji_motor_register(&shoot_motor_config[MIDDLE_FRICTION], motor_control_middle);
    sht_motor[RIGHT_FRICTION] = dji_motor_register(&shoot_motor_config[RIGHT_FRICTION], motor_control_right);
}

/*右摩擦轮电机控制算法*/
static rt_int16_t motor_control_right(dji_motor_measure_t measure)
{
    static rt_int16_t set = 0;
    set =(int16_t) pid_calculate(sht_controller[RIGHT_FRICTION].pid_speed, measure.speed_rpm, shoot_motor_ref[RIGHT_FRICTION]);
    return set;
}

/*左摩擦轮电机控制算法*/
static rt_int16_t motor_control_left(dji_motor_measure_t measure)
{
    static rt_int16_t set = 0;
    set = (int16_t) pid_calculate(sht_controller[LEFT_FRICTION].pid_speed, measure.speed_rpm, shoot_motor_ref[LEFT_FRICTION]);
    return set;
}
/*中摩擦轮电机控制算法*/
static rt_int16_t motor_control_middle(dji_motor_measure_t measure)
{
    static rt_int16_t set = 0;
    set = (int16_t) pid_calculate(sht_controller[MIDDLE_FRICTION].pid_speed, measure.speed_rpm, shoot_motor_ref[MIDDLE_FRICTION]);
    return set;
}

/*拨弹电机控制算法*/
static rt_int16_t motor_control_trigger(dji_motor_measure_t measure)
{
    /* PID局部指针，切换不同模式下PID控制器 */
    static pid_obj_t *pid_angle;
    static pid_obj_t *pid_speed;
    static float get_speed, get_angle;  // 闭环反馈量
    static float pid_out_angle;         // 角度环输出
    static rt_int16_t send_data;        // 最终发送给电调的数据

    /*拨弹电机采用串级pid，一个角度环和一个速度环*/
    pid_speed = sht_controller[TRIGGER_MOTOR].pid_speed;
    pid_angle = sht_controller[TRIGGER_MOTOR].pid_angle;
    get_angle=measure.total_angle;
    get_speed=measure.speed_rpm;

    /* 切换模式需要清空控制器历史状态 */
    if(shoot_cmd.ctrl_mode != shoot_cmd.last_mode)
    {
        pid_clear(pid_angle);
        pid_clear(pid_speed);
    }

    /*pid计算输出*/
    if (shoot_cmd.ctrl_mode==SHOOT_ONE||shoot_cmd.ctrl_mode==SHOOT_THREE) //非连发模式的时候，用双环pid控制拨弹电机
    {
        pid_out_angle = (int16_t) pid_calculate(pid_angle, get_angle, shoot_motor_ref[TRIGGER_MOTOR]);  // 编码器增长方向与imu相反
        send_data = (int16_t) pid_calculate(pid_speed, get_speed, pid_out_angle);     // 电机转动正方向与imu相反
    }
    /*pid计算输出*/
    else if(shoot_cmd.ctrl_mode==SHOOT_COUNTINUE||shoot_cmd.ctrl_mode==SHOOT_STOP||shoot_cmd.ctrl_mode==SHOOT_REVERSE)//自动模式的时候，只用速度环控制拨弹电机
    {
        send_data = (int16_t) pid_calculate(pid_speed, get_speed, shoot_motor_ref[TRIGGER_MOTOR] );
    }
    return send_data;
}

//枪口热量限制
static void shoot_self_cmd_Init(void) {

    self_shoot_info.last_angle= sht_motor[TRIGGER_MOTOR]->measure.total_angle;
    self_shoot_info.remain_bullet = MAXHEAT / HEATADD;
    self_shoot_info.last_dt=dwt_get_time_ms();
}
static void shoot_self_cmd(void) {
    uint8_t shoot_cnt = 0;//发射子弹数
    static float delta_dt=0;

    self_shoot_info.now_angle = sht_motor[TRIGGER_MOTOR]->measure.total_angle;
    shoot_cnt = (self_shoot_info.now_angle-self_shoot_info.last_angle)/TRIGGER_MOTOR_51_TO_ANGLE;
    self_shoot_info.bullet_cnt += shoot_cnt; //每转45°(英雄为51.43°)就是发射了一颗子弹
    self_shoot_info.bullet_heat += shoot_cnt * HEATADD;
    //枪口热量按 10Hz 的频率结算冷却，每个检测周期热量冷却值 = 每秒冷却值 / 10
    self_shoot_info.now_dt=dwt_get_time_ms();
    delta_dt=(self_shoot_info.now_dt-self_shoot_info.last_dt)/1000.0;//裁判系统是10Hz->每隔100ms结算一次，这里把COOL从ms转化成s
    self_shoot_info.bullet_heat -= COOL * delta_dt;//delta_dt精准到1ms，控制更实时；COOL是1s的冷却值

    if(self_shoot_info.bullet_heat <0)self_shoot_info.bullet_heat = 0;
    //计算剩余热量还能打多少子弹
    self_shoot_info.remain_bullet = (MAXHEAT - self_shoot_info.bullet_heat) / HEATADD;//一颗子弹的热量是HEATADD
/*裁判系统出现断连之后进入状态判断*/
    self_shoot_info.risk_level = REGULAR;//默认为REGULAR，这样能与裁判系统的热量管理兼容

    if(!self_shoot_info.referee_flag) {//裁判系统意外断连才接入cmd msg的修改
        if(self_shoot_info.remain_bullet <= 3) {
            /*裁判系统到机器人有100ms延迟，如果freq>10,一秒发射10发以上会出现裁判系统来不及反馈，但其实已经过热的情况，
             *临界状态应该降低freq，但在前文中freq>10的情况下trig ref已经设置为0了,所以暂时不管临界状态？*/
            if(shoot_cmd.ctrl_mode == SHOOT_COUNTINUE) { //处于三连发射击模式时击发一次之后停火，也可以改成单发模式/或不击发/
                 self_shoot_info.risk_level = RISK;
                // shoot_cmd.ctrl_mode= SHOOT_ONE;
                // shoot_cmd.trigger_status=TRIGGER_OFF;
            }
            if(self_shoot_info.remain_bullet <= 0) {
                self_shoot_info.risk_level = STOP;
                // shoot_cmd.shoot_freq=0;
            }
        }

    }
    self_shoot_info.last_angle=self_shoot_info.now_angle;
    self_shoot_info.last_dt=self_shoot_info.now_dt;
}
static void self_shoot_msg_restrict(void) {
    if(self_shoot_info.referee_flag== referee_status) {
        switch (self_shoot_info.risk_level)
        {//如果需要控制临界状态，可以新增一个case，所以写成这个形式
            case REGULAR:
                break;
            case RISK://RISK状态用作三连发的边界状态判断，所以直接在射击状态机里控制了
                break;
            case STOP://在此把STOP状态转化为cmd的ctrl mode，减少在射击状态机中的if判断
                shoot_cmd.ctrl_mode = SHOOT_STOP;
            break;
        }
    }
}

/**
 * @brief shoot 线程中所有发布者初始化
 */
static void shoot_pub_init()
{
    pub_shoot = pub_register("shoot_fdb", sizeof(struct shoot_fdb_msg));
}
/**
 * @brief shoot 线程中所有订阅者初始化
 */
static void shoot_sub_init()
{
    sub_cmd = sub_register("shoot_cmd", sizeof(struct shoot_cmd_msg));
}
/**
 * @brief shoot 线程中所有发布者推送更新话题
 */
static void shoot_pub_push()
{
    pub_push_msg(pub_shoot, &shoot_fdb);
}
/**
 * @brief shoot 线程中所有订阅者推送更新话题
 */
static void shoot_sub_pull()
{
    sub_get_msg(sub_cmd, &shoot_cmd);
}
