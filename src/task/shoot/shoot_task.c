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

static int debug_ref1,debug_ref2;
static int debug_mode;
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
static int shoot_cnt;
static int shoot_flag;
/*发射模块电机使用数量*/
// #define SHT_MOTOR_NUM 3
#define SHT_MOTOR_NUM 4

#define SHOOT_MOTOR1 0
#define SHOOT_MOTOR2 1
#define SHOOT_MOTOR3 2
#define SHOOT_MOTOR4 3

#define LOAD_MOTOR 4
// #define LOAD_MOTOR 6//GM2006供弹


/*pid环数结构体*/
static struct shoot_controller_t{
    pid_obj_t *pid_speed;
    pid_obj_t *pid_angle;
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
        },
};
motor_config_t load_motor_config ={
    .motor_type = M3508,
    .can_name = CAN_CHASSIS,
    .rx_id = LOAD_MOTOR_ID,
    .controller = &load_controller
};
static dji_motor_object_t *sht_motor[SHT_MOTOR_NUM];  // 发射器电机实例
static float shoot_motor_ref[SHT_MOTOR_NUM]; // 电机控制期望值
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
    shoot_cmd.ctrl_mode=SHOOT_STOP;
    shoot_cmd.friction_status = 0;
    LOG_I("Shoot Task Start");
    for (;;)
    {
        sht_start = dwt_get_time_ms();
        /* 更新该线程所有的订阅者 */
        shoot_sub_pull();

        shoot_cnt++;
        shoot_cnt%=1000;

        /* 电机控制启动 */
        for (uint8_t i = 0; i < SHT_MOTOR_NUM; i++)
        {
            dji_motor_enable(sht_motor[i]);
            dji_motor_enable(load_motor);
        }
         // shoot_fdb.trigger_motor_current=sht_motor[TRIGGER_MOTOR]->measure.real_current;

        /*subs遥控器*/

        switch (shoot_cmd.ctrl_mode)
        {
            case SHOOT_STOP:
                for(int i=0;i<SHT_MOTOR_NUM;i++) {
                    shoot_motor_ref[i] = 0;
                }
                load_ref_rpm = 0;
                break;

            case SHOOT_ONE:
                if(shoot_cmd.friction_speed == HIGH_FREQUENCY) {
                    ref_rpm_1 = 9000;
                    ref_rpm_2 = 7000;
                }else if(shoot_cmd.friction_speed == LOW_FREQUENCY) {
                    /*是否改成宏定义在menuconfig里？*/
                    if(!debug_mode) {
                        ref_rpm_1 = 5500;
                        ref_rpm_2 = 7000;
                    }else if(debug_mode == 0){
                        ref_rpm_1 = debug_ref1;
                        ref_rpm_2 = debug_ref2;
                    }
                }
                shoot_motor_ref[SHOOT_MOTOR1] = -ref_rpm_1 ;//摩擦轮常转
                shoot_motor_ref[SHOOT_MOTOR2] = ref_rpm_1;
                shoot_motor_ref[SHOOT_MOTOR3] = -ref_rpm_2;//摩擦轮常转
                shoot_motor_ref[SHOOT_MOTOR4] = ref_rpm_2;
                load_ref_rpm = 0;//拨弹电机上行
                // shoot_fdb.trigger_status=SHOOT_OK;
                break;
            case SHOOT_REVERSE:
                for(int i=0;i<SHT_MOTOR_NUM;i++) {
                    shoot_motor_ref[i] = 0;
                }
                load_ref_rpm = -4000;//拨弹电机下行,要改成角度环
                // if(load_motor->measure.total_round <= 10) {
                //     shoot_fdb.load_status == LOAD_BACK_OK;
                // }else {
                //     shoot_fdb.load_status == LOAD_BACK_ON;
                // }
                break;
            default:
                for (uint8_t i = 0; i < SHT_MOTOR_NUM; i++)
                {
                    dji_motor_relax(sht_motor[i]); // 错误情况电机全部松电
                    dji_motor_enable(load_motor);
                }
            dji_motor_relax(load_motor);
                shoot_fdb.trigger_status=SHOOT_ERR;
                break;
        }



        /* 更新发布该线程的msg */
        shoot_pub_push();

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
    /* -------------------------------------- 电机1 ----------------------------------------- */
    pid_config_t right_speed_config = INIT_PID_CONFIG(RIGHT_KP_V, RIGHT_KI_V, RIGHT_KD_V,RIGHT_INTEGRAL_V,RIGHT_MAX_V,
                                                        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    sht_controller[SHOOT_MOTOR1].pid_speed = pid_register(& right_speed_config);

/* ------------------------------------------- 电机2------------------------------------------------- */
    pid_config_t left_speed_config = INIT_PID_CONFIG(LEFT_KP_V,  LEFT_KI_V, LEFT_KD_V , LEFT_INTEGRAL_V, LEFT_MAX_V,
                                                          (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    sht_controller[SHOOT_MOTOR2].pid_speed = pid_register(&left_speed_config);
/* ------------------------------------------- 摩擦轮电机3------------------------------------------------- */
pid_config_t shoot3_speed_config = INIT_PID_CONFIG(LEFT_KP_V,  LEFT_KI_V, LEFT_KD_V , LEFT_INTEGRAL_V, LEFT_MAX_V,
                                                      (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
sht_controller[SHOOT_MOTOR3].pid_speed = pid_register(&shoot3_speed_config);
/* ------------------------------------------- 摩擦轮电机4------------------------------------------------- */
pid_config_t shoot4_speed_config = INIT_PID_CONFIG(LEFT_KP_V,  LEFT_KI_V, LEFT_KD_V , LEFT_INTEGRAL_V, LEFT_MAX_V,
                                                      (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
sht_controller[SHOOT_MOTOR4].pid_speed = pid_register(&shoot4_speed_config);
/* ------------------------------------------------  拨弹电机------------------------------------------------------------------------- */
    pid_config_t load_speed_config = INIT_PID_CONFIG(TRIGGER_KP_V  , TRIGGER_KI_V , TRIGGER_KD_V  , TRIGGER_INTEGRAL_V, TRIGGER_MAX_V ,
                                                       (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    // pid_config_t load_angle_config = INIT_PID_CONFIG(TRIGGER_KP_A, TRIGGER_KI_A, TRIGGER_KD_A, TRIGGER_INTEGRAL_A , TRIGGER_MAX_A ,
    //                                                    (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    load_controller.pid_speed = pid_register(&load_speed_config);
    // load_controller.pid_angle = pid_register(&load_angle_config);

/* ---------------------------------- shoot电机初注册---------------------------------------------------------------------------------------- */
    sht_motor[SHOOT_MOTOR1] = dji_motor_register(&shoot_motor_config[SHOOT_MOTOR1], shoot_control_1);
    sht_motor[SHOOT_MOTOR2] = dji_motor_register(&shoot_motor_config[SHOOT_MOTOR2], shoot_control_2);
    sht_motor[SHOOT_MOTOR3] = dji_motor_register(&shoot_motor_config[SHOOT_MOTOR3], shoot_control_3);
    sht_motor[SHOOT_MOTOR4] = dji_motor_register(&shoot_motor_config[SHOOT_MOTOR4], shoot_control_4);
    load_motor = dji_motor_register(&load_motor_config,load_control);

}

/*右摩擦轮电机控制算法*/
static rt_int16_t shoot_control_1(dji_motor_measure_t measure)
{
    static rt_int16_t set = 0;
    set =(int16_t) pid_calculate(sht_controller[SHOOT_MOTOR1].pid_speed, measure.speed_rpm, shoot_motor_ref[SHOOT_MOTOR1]);
    return set;
}
/*右摩擦轮电机控制算法*/
static rt_int16_t shoot_control_2(dji_motor_measure_t measure)
{
    static rt_int16_t set = 0;
    set = (int16_t) pid_calculate(sht_controller[SHOOT_MOTOR2].pid_speed, measure.speed_rpm, shoot_motor_ref[SHOOT_MOTOR2]/*left_speed*/);
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
static rt_int16_t load_control(dji_motor_measure_t measure)
{
    static rt_int16_t set = 0;
    set = (int16_t) pid_calculate(load_controller.pid_speed, measure.speed_rpm, load_ref_rpm);
    return set;
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
