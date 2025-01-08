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

/*发射模块电机使用数量*/
#define SHT_MOTOR_NUM 4//摩擦轮目前是四个，之后要改成六个,外加拨弹电机，总共是7

/*发射模块电机编号*/
#define SHOOT_MOTOR1 0
#define SHOOT_MOTOR2 1
#define SHOOT_MOTOR3 2
#define SHOOT_MOTOR4 3
// #define SHOOT5_MOTOR 4
// #define SHOOT6_MOTOR 5
#define LOAD_MOTOR 6//GM2006供弹
//准备用两个宏定义作为初始值和目标值，总角度达到目标值就停止转动（完成发射）

/*pid环数结构体*/
static struct shoot_controller_t{
    pid_obj_t *pid_speed;
}sht_controller[SHT_MOTOR_NUM];
static struct load_controller_t{
    pid_obj_t *pid_speed;
    pid_obj_t *pid_angle;
}load_controller;

/*电机注册初始化数据*/
motor_config_t shoot_motor_config[SHT_MOTOR_NUM] ={
    {
        .motor_type = M3508,
        .can_name = CAN_CHASSIS,//待定
        .rx_id = SHOOT1_MOTOR_ID,
        .controller = &sht_controller[SHOOT_MOTOR1],
    },
    {
        .motor_type = M3508,
        .can_name = CAN_CHASSIS,
        .rx_id = SHOOT2_MOTOR_ID,
        .controller = &sht_controller[SHOOT_MOTOR2],
    },
    {
        .motor_type = M3508,
        .can_name = CAN_CHASSIS,
        .rx_id = SHOOT3_MOTOR_ID,
        .controller = &sht_controller[SHOOT_MOTOR3],
    },
    {
    .motor_type = M3508,
    .can_name = CAN_CHASSIS,
    .rx_id = SHOOT4_MOTOR_ID,
    .controller = &sht_controller[SHOOT_MOTOR4],
    }
};
motor_config_t load_motor_config ={
    .motor_type = M3508,
    .can_name = CAN_CHASSIS,
    .rx_id = LOAD_MOTOR_ID,
    .controller = &load_controller
};

static dji_motor_object_t *sht_motor[SHT_MOTOR_NUM];  // 发射器电机实例

static float shoot_motor_ref[SHT_MOTOR_NUM]; // shoot电机控制期望值
static dji_motor_object_t *load_motor;//供弹电机实例
static float load_ref_rpm, load_ref_distance;//供弹电机控制期望值
/*函数声明*/
static void shoot_motor_init();
static rt_int16_t shoot_control_1(dji_motor_measure_t measure);
static rt_int16_t shoot_control_2(dji_motor_measure_t measure);
static rt_int16_t shoot_control_3(dji_motor_measure_t measure);
static rt_int16_t shoot_control_4(dji_motor_measure_t measure);
static rt_int16_t load_control(dji_motor_measure_t measure);
/* --------------------------------- 射击线程入口 --------------------------------- */
static float sht_dt;
static int ref_rpm_1;//motor 0 1 一级
static int ref_rpm_2;//motor 2 3 二级
// static int ref_load = 1000;
/**
 * @brief shoot线程入口函数
 */
void shoot_task_entry(void* argument)
{
    static float sht_start;

    shoot_motor_init();
    shoot_pub_init();
    shoot_sub_init();

/*----------------------射击状态初始化----------------------------------*/
    shoot_cmd.ctrl_mode = SHOOT_STOP;
    shoot_cmd.friction_status = 0;
    for(int i=0;i < SHT_MOTOR_NUM; i++) {
        shoot_motor_ref[i] = 0;
    }
    /*load motor not singed up*/
    // load_motor_ref = 0;
    LOG_I("Shoot Task Start");
    for (;;)
    {
        sht_start = dwt_get_time_ms();
        /* 更新该线程所有的订阅者 */
        shoot_sub_pull();

        /* 电机控制启动 */
        for (uint8_t i = 0; i < SHT_MOTOR_NUM; i++)
        {
            dji_motor_enable(sht_motor[i]);
        }

         // shoot_fdb.trigger_motor_current=sht_motor[TRIGGER_MOTOR]->measure.real_current;
        /*控制模式判断*/
        /*subs遥控器*/
#ifdef BSP_USING_RC_SBUS
        switch (shoot_cmd.ctrl_mode)
        {
            case SHOOT_STOP:
                for(int i=0;i<SHT_MOTOR_NUM;i++) {
                    shoot_motor_ref[i] = 0;
                }
                // load_ref_rpm = -1000;//拨弹电机下行
                load_ref_distance = LOAD_INIT_DISTANCE;
                if(load_motor->measure.total_round <= 10) {
                    shoot_fdb.load_status == LOAD_BACK_OK;
                }else {
                    shoot_fdb.load_status == LOAD_BACK_ON;
                }

                break;

            case SHOOT_ONE:
                if(shoot_cmd.friction_speed == HIGH_FREQUENCY) {
                    ref_rpm_1 = 9000;
                    ref_rpm_2 = 9000;
                }else if(shoot_cmd.friction_speed == LOW_FREQUENCY) {
                    ref_rpm_1 = 6000;
                    ref_rpm_2 = 6000;
                }

                shoot_motor_ref[SHOOT_MOTOR1] = -ref_rpm_1 ;//摩擦轮常转
                shoot_motor_ref[SHOOT_MOTOR2] = ref_rpm_1;
                shoot_motor_ref[SHOOT_MOTOR3] = -ref_rpm_2;//摩擦轮常转
                shoot_motor_ref[SHOOT_MOTOR4] = ref_rpm_2;
                load_ref_rpm = 5000;//拨弹电机上行
                load_ref_distance = LOAD_MAX_DISTANCE;

                shoot_fdb.load_status = LOADING;
                break;
            case SHOOT_REVERSE:
                for(int i=0;i<SHT_MOTOR_NUM;i++) {
                    shoot_motor_ref[i] = 0;
                }
                load_ref_rpm = -1000;//拨弹电机下行
                load_ref_distance = 0;
                if(load_motor->measure.total_round <= 10) {
                    shoot_fdb.load_status == LOAD_BACK_OK;
                }else {
                    shoot_fdb.load_status == LOAD_BACK_ON;
                }
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
 * @brief shoot 线程电机初始化
 */
static void shoot_motor_init(){
    /* -------------------------------------- 摩擦轮电机1----------------------------------------- */
    pid_config_t shoot1_speed_config = INIT_PID_CONFIG(RIGHT_KP_V, RIGHT_KI_V, RIGHT_KD_V,RIGHT_INTEGRAL_V,RIGHT_MAX_V,
                                                        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    sht_controller[SHOOT_MOTOR1].pid_speed = pid_register(& shoot1_speed_config);

/* ------------------------------------------- 摩擦轮电机2------------------------------------------------- */
    pid_config_t shoot2_speed_config = INIT_PID_CONFIG(LEFT_KP_V,  LEFT_KI_V, LEFT_KD_V , LEFT_INTEGRAL_V, LEFT_MAX_V,
                                                          (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    sht_controller[SHOOT_MOTOR2].pid_speed = pid_register(&shoot2_speed_config);
/* ------------------------------------------- 摩擦轮电机3------------------------------------------------- */
pid_config_t shoot3_speed_config = INIT_PID_CONFIG(LEFT_KP_V,  LEFT_KI_V, LEFT_KD_V , LEFT_INTEGRAL_V, LEFT_MAX_V,
                                                      (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
sht_controller[SHOOT_MOTOR3].pid_speed = pid_register(&shoot3_speed_config);
/* ------------------------------------------- 摩擦轮电机4------------------------------------------------- */
pid_config_t shoot4_speed_config = INIT_PID_CONFIG(LEFT_KP_V,  LEFT_KI_V, LEFT_KD_V , LEFT_INTEGRAL_V, LEFT_MAX_V,
                                                      (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
sht_controller[SHOOT_MOTOR4].pid_speed = pid_register(&shoot4_speed_config);

/* ------------------------------------------------  供弹电机------------------------------------------------------------------------- */
    pid_config_t load_speed_config = INIT_PID_CONFIG(TRIGGER_KP_V  , TRIGGER_KI_V , TRIGGER_KD_V  , TRIGGER_INTEGRAL_V, TRIGGER_MAX_V ,
                                                       (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t load_angle_config = INIT_PID_CONFIG(TRIGGER_KP_A, TRIGGER_KI_A, TRIGGER_KD_A, TRIGGER_INTEGRAL_A , TRIGGER_MAX_A ,
                                                       (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    load_controller.pid_speed = pid_register(&load_speed_config);
    load_controller.pid_angle = pid_register(&load_angle_config);

/* ---------------------------------- shoot电机初注册---------------------------------------------------------------------------------------- */
    sht_motor[SHOOT_MOTOR1] = dji_motor_register(&shoot_motor_config[SHOOT_MOTOR1], shoot_control_1);
    sht_motor[SHOOT_MOTOR2] = dji_motor_register(&shoot_motor_config[SHOOT_MOTOR2], shoot_control_2);
    sht_motor[SHOOT_MOTOR3] = dji_motor_register(&shoot_motor_config[SHOOT_MOTOR3], shoot_control_3);
    sht_motor[SHOOT_MOTOR4] = dji_motor_register(&shoot_motor_config[SHOOT_MOTOR3], shoot_control_4);

    load_motor = dji_motor_register(&load_motor_config,load_motor);
}

/* ---------------------------------- shoot电机控制算法---------------------------------------------------------------------------------------- */
static rt_int16_t shoot_control_1(dji_motor_measure_t measure)
{
    static rt_int16_t set = 0;
    set =(int16_t) pid_calculate(sht_controller[SHOOT_MOTOR1].pid_speed, measure.speed_rpm, shoot_motor_ref[SHOOT_MOTOR1]);
    return set;
}
static rt_int16_t shoot_control_2(dji_motor_measure_t measure)
{
    static rt_int16_t set = 0;
    set = (int16_t) pid_calculate(sht_controller[SHOOT_MOTOR2].pid_speed, measure.speed_rpm, shoot_motor_ref[SHOOT_MOTOR2]);
    return set;
}
static rt_int16_t shoot_control_3(dji_motor_measure_t measure)
{
    static rt_int16_t set = 0;
    set = (int16_t) pid_calculate(sht_controller[SHOOT_MOTOR3].pid_speed, measure.speed_rpm, shoot_motor_ref[SHOOT_MOTOR3]);
    return set;
}
static rt_int16_t shoot_control_4(dji_motor_measure_t measure)
{
    static rt_int16_t set = 0;
    set = (int16_t) pid_calculate(sht_controller[SHOOT_MOTOR4].pid_speed, measure.speed_rpm, shoot_motor_ref[SHOOT_MOTOR4]);
    return set;
}

/*拨弹电机控制算法*/
static rt_int16_t load_motor_control(dji_motor_measure_t measure)
{
    /* PID局部指针，切换不同模式下PID控制器 */
    static pid_obj_t *pid_location;
    static pid_obj_t *pid_speed;
    static float get_speed, get_location;  // 闭环反馈量
    static float pid_out_location;         // 角度环输出
    static rt_int16_t send_data;        // 最终发送给电调的数据

    /*拨弹电机采用串级pid，一个角度环和一个速度环*/
    pid_speed = load_controller.pid_speed;
    pid_location = load_controller.pid_angle;
    get_location = measure.total_angle * PITCH_GEAR_RATIO * ANGLE_TO_DISTANCE;//load也是2006，减速比同PITCH
    get_speed = measure.speed_rpm;

    /* 切换模式需要清空控制器历史状态 */
    if(shoot_cmd.ctrl_mode != shoot_cmd.last_mode)
    {
        pid_clear(pid_location);
        pid_clear(pid_speed);
    }

    /*pid计算输出*/
    if (shoot_cmd.ctrl_mode==SHOOT_ONE) //非连发模式的时候，用双环pid控制拨弹电机
    {
        pid_out_location = (int16_t) pid_calculate(pid_location, get_location, load_ref_distance);
        send_data = (int16_t) pid_calculate(pid_speed, get_speed, pid_out_location);
    }
    /*pid计算输出*/
    else if(shoot_cmd.ctrl_mode==SHOOT_STOP||shoot_cmd.ctrl_mode==SHOOT_REVERSE)//自动模式的时候，只用速度环控制拨弹电机
    {
        send_data = (int16_t) pid_calculate(pid_speed, get_speed, load_ref_rpm);
    }
    return send_data;
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
