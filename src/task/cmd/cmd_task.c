/*
* Change Logs:
* Date            Author          Notes
* 2023-09-24      ChuShicheng     first version
* 2023-10-10      ChenSihan       发射模块状态机
*/

#include "cmd_task.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "rm_task.h"
#include "src/modules/rc/sbus/rc_sbus.c"

#define DBG_TAG   "rm.task"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static publisher_t  *pub_chassis, *pub_shoot;
static subscriber_t *sub_shoot, *sub_chassis;

static struct shoot_cmd_msg  shoot_cmd;
static struct shoot_fdb_msg  shoot_fdb;
static struct chassis_cmd_msg chassis_cmd;
static struct chassis_fdb_msg chassis_fdb;


static void cmd_pub_init(void);
static void cmd_pub_push(void);
static void cmd_sub_init(void);
static void cmd_sub_pull(void);
static void remote_to_cmd_sbus(void);

// /*发射停止标志位*/
// static int trigger_flag=0;

static rc_obj_t *rc_now, *rc_last;
/*用于清除环形缓冲区buffer的指针*/
extern rt_uint8_t *r_buffer_point;
/*----------------------------------裁判系统数据接收/比赛状态-------------------------------------*/
// extern robot_status_t robot_status;//应该也不需要

/* --------------------------------- cmd线程入口 -------------------------------- */
static float cmd_dt;

void cmd_thread_entry(void *argument)
{
    static float cmd_start;

    cmd_pub_init();
    cmd_sub_init();

    rc_now = sbus_rc_init();
    rc_last = (rc_now + 1);   // rc_obj[0]:当前数据NOW,[1]:上一次的数据LAST

    /* 初始化拨杆为上位 */
    rc_now->sw1 = RC_UP;
    rc_now->sw2 = RC_UP;
    rc_now->sw4 = RC_UP;
    LOG_I("Cmd Task Start");
    for (;;)
    {
        cmd_start = dwt_get_time_ms();
        /* 更新该线程所有的订阅者 */
        cmd_sub_pull();
        /* 将遥控器原始数据转换为控制指令 */
        #ifdef BSP_USING_RC_SBUS
        remote_to_cmd_sbus();
        #endif

        /* 更新发布该线程的msg */
        cmd_pub_push();

        /* 用于调试监测线程调度使用 */
        cmd_dt = dwt_get_time_ms() - cmd_start;
        if (cmd_dt > 1)
                LOG_E("Cmd Task is being DELAY! dt = [%f]", &cmd_dt);

        rt_thread_delay(1);
    }
}

/* --------------------------------- 线程间通讯相关 -------------------------------- */
/**
 * @brief cmd 线程中所有发布者初始化
 */
static void cmd_pub_init(void)
{
    pub_shoot= pub_register("shoot_cmd", sizeof(struct shoot_cmd_msg));
    pub_chassis = pub_register("chassis_cmd",sizeof(struct chassis_cmd_msg));
}

/**
 * @brief cmd 线程中所有发布者推送更新话题
 */
static void cmd_pub_push(void)
{
    pub_push_msg(pub_shoot, &shoot_cmd);
    pub_push_msg(pub_chassis,&chassis_cmd);
}

/**
 * @brief cmd 线程中所有订阅者初始化
 */
static void cmd_sub_init(void)
{
    sub_shoot= sub_register("shoot_fdb", sizeof(struct shoot_fdb_msg));
    sub_chassis = sub_register("chassis_fdb",sizeof(struct chassis_fdb_msg));
}

/**
 * @brief cmd 线程中所有订阅者获取更新话题
 */
static void cmd_sub_pull(void)
{
    sub_get_msg(sub_shoot, &shoot_fdb);
    sub_get_msg(sub_chassis,&chassis_fdb);
}

/* ------------------------------ 将遥控器数据转换为控制指令 ----------------------------- */
/**
 * @brief 将遥控器数据转换为控制指令
 */
#ifdef BSP_USING_RC_SBUS
static void remote_to_cmd_sbus(void)
{
    chassis_cmd.last_mode = chassis_cmd.ctrl_mode;
    shoot_cmd.last_mode=shoot_cmd.ctrl_mode;
    *rc_last = *rc_now;
//遥控器的控制信息转化为标准单位，平移为(mm/s)旋转为(degree/s)
    /* 限制发射架角度 */
    VAL_LIMIT(chassis_cmd.pitch, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
    /*-------------------------------------------------底盘状态机--------------------------------------------------------------*/
    //ch1 left right(yaw)
    //ch2 up down(pitch),
    //sw1 up(chassis enable),down(chassis disable)
    //sw4 up(全局失能）

    if (chassis_cmd.ctrl_mode==CHASSIS_INIT||chassis_cmd.ctrl_mode==CHASSIS_RELAX)
    {//归中
        chassis_cmd.pitch = PITCH_INIT_ANGLE;
        chassis_cmd.yaw = YAW_INIT_ANGLE;
    }
    switch (rc_now->sw1)
    {//底盘 UP：使能 DOWN：失能
        case RC_UP:
            chassis_cmd.ctrl_mode = CHASSIS_OPEN_LOOP;
            /*控制pitch、yaw,系数待定*/
            chassis_cmd.vw_yaw = 2000*(float)(rc_now->ch1) / 784.0;
            chassis_cmd.vw_pitch = 2000*(float)(rc_now->ch2) / 784.0;;
            break;
        case RC_MI://先归中再失能
            if(chassis_cmd.last_mode != CHASSIS_INIT) {
                chassis_cmd.ctrl_mode = CHASSIS_INIT;
            }
            else if(chassis_fdb.back_mode == BACK_IS_OK)
            {
                chassis_cmd.ctrl_mode = CHASSIS_RELAX;
            }
            break;
        default:
            chassis_cmd.ctrl_mode = CHASSIS_RELAX;


    }
    /* 因为左拨杆值会影响到底盘RELAX状态，所以后判断 */
    switch(rc_now->sw4)
    {//全机 UP：使能 DOWN：失能
        case RC_UP:
            break;
        case RC_MI:
            chassis_cmd.ctrl_mode = CHASSIS_RELAX;
            shoot_cmd.ctrl_mode = SHOOT_STOP;
            shoot_cmd.friction_status = 0;
            break;
        default:
            chassis_cmd.ctrl_mode = CHASSIS_RELAX;

    }
    /*--------------------------------------------------发射状态机--------------------------------------------------------------*/
    //ch6 load motor
    //sw2 up(friction on)
    //sw3 up(friction highspeed)
    if(rc_now->sw4 == RC_UP) {//不是全局失能状态，允许发射
        switch(rc_now->sw2)
        {
            case RC_UP:
                // chassis_cmd.ctrl_mode = CHASSIS_RELAX;
                shoot_cmd.ctrl_mode=SHOOT_STOP;
            break;
            case RC_MI:
                if(chassis_fdb.back_mode == BACK_IS_OK)
                {
                    shoot_cmd.friction_status = 1;
                }
                if(rc_now->ch3> LOAD_VALUE ) {
                    if(shoot_fdb.load_status == LOAD_BACK_OK) {//判断是否可以发射
                        shoot_cmd.ctrl_mode = SHOOT_ONE;//单发模式，在这里完成供弹，并且定义挡板回归到最初位置的的状态量
                    }
                }
                switch (rc_now->sw3) {
                    case RC_UP:
                        shoot_cmd.friction_speed = HIGH_FREQUENCY;
                    break;
                    case RC_MI:
                        shoot_cmd.friction_speed = LOW_FREQUENCY;
                    break;
                    default:
                        shoot_cmd.friction_speed = HIGH_FREQUENCY;
                    break;

                }
                break;
            default:
                shoot_cmd.ctrl_mode=SHOOT_STOP;
            break;
        }
    }





}
#endif

