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
// #include "src/modules/rc/sbus/rc_sbus.c"
#include "src/modules/rc/sbus/rc_sbus.h"
#define DBG_TAG   "rm.task"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static publisher_t  *pub_chassis, *pub_shoot;
static subscriber_t  *sub_shoot,*sub_ins,*sub_chassis;
static struct shoot_cmd_msg  shoot_cmd;
static struct shoot_fdb_msg  shoot_fdb;
static struct chassis_cmd_msg chassis_cmd;
static struct chassis_fdb_msg chassis_fdb;
static struct ins_msg ins_data;


static rc_obj_t *rc_now, *rc_last;
static void remote_to_cmd_sbus(void);
static void cmd_pub_init(void);
static void cmd_pub_push(void);
static void cmd_sub_init(void);
static void cmd_sub_pull(void);

static int cmd_cnt;

static rc_obj_t *rc_now,*rc_last;
/*用于清除环形缓冲区buffer的指针*/
extern rt_uint8_t *r_buffer_point;
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
    rc_now->sw3 = RC_UP;
    rc_now->sw4 = RC_UP;
    LOG_I("Cmd Task Start");
    for (;;)
    {
        cmd_start = dwt_get_time_ms();
        /* 更新该线程所有的订阅者 */
        cmd_sub_pull();

        cmd_cnt++;
        cmd_cnt%=1000;

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
    pub_chassis = pub_register("chassis_cmd", sizeof(struct chassis_cmd_msg));
    pub_shoot= pub_register("shoot_cmd", sizeof(struct shoot_cmd_msg));
}

/**
 * @brief cmd 线程中所有发布者推送更新话题
 */
static void cmd_pub_push(void)
{
    pub_push_msg(pub_chassis, &chassis_cmd);
    pub_push_msg(pub_shoot, &shoot_cmd);
}

/**
 * @brief cmd 线程中所有订阅者初始化
 */
static void cmd_sub_init(void)
{
    sub_shoot= sub_register("shoot_fdb", sizeof(struct shoot_fdb_msg));
    sub_chassis = sub_register("chassis_fdb",sizeof(struct chassis_fdb_msg));
    sub_ins = sub_register("ins_msg", sizeof(struct ins_msg));
}

/**
 * @brief cmd 线程中所有订阅者获取更新话题
 */
static void cmd_sub_pull(void)
{
    sub_get_msg(sub_shoot, &shoot_fdb);
    sub_get_msg(sub_ins, &ins_data);
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
    /* 限制发射架角度 */
    // VAL_LIMIT(chassis_cmd.pitch, PIT_ANGLE_MIN, PIT_ANGLE_MAX);

    /*-------------------------------------------------底盘状态机--------------------------------------------------------------*/
    //ch3 left right(yaw)
    //ch4 up down(pitch),
    //sw1 up(chassis enable),down(chassis disable)
    //sw4 up(全局失能）
    switch (rc_now->sw1)
    {//底盘 UP：失能 DOWN：使能
        case RC_UP:
            chassis_cmd.ctrl_mode = CHASSIS_STOP;
        break;
        case RC_DN:
            chassis_cmd.ctrl_mode = CHASSIS_OPEN_LOOP;
            /*控制pitch、yaw,系数待定*/
/*pitch,ch3*/
            if(rc_now->ch3 > 80) {//死区，pitch的比50多
                chassis_cmd.vw_pitch = 7000 *(float)( rc_now->ch3) / -784.0 ;
            }else if(rc_now->ch3 <-80) {
                chassis_cmd.vw_pitch = -7000 *(float)(rc_now->ch3) / 784.0 ;
            }else{
                chassis_cmd.vw_pitch = 0;
            }
/*yaw,ch4*/
            if(rc_now->ch4 > 80) {//yaw的死区差不多50
                chassis_cmd.vw_yaw = -1500 * (float) (rc_now->ch4) / 784.0 ;
            }else if(rc_now->ch4 < -80) {
                chassis_cmd.vw_yaw = 1500 * (float)(rc_now->ch4) / -784.0 ;
            }else{
                chassis_cmd.vw_yaw = 0;
            }
        break;
    }

    /*--------------------------------------------------发射状态机--------------------------------------------------------------*/
    //ch6 load motor(shoot one在shoot task内实现）
    //sw2 up(friction on)
    //sw3 up(friction highspeed)
    if(rc_now->sw4 == RC_DN) {//不是全局失能状态，允许发射
        switch(rc_now->sw3) {
            case RC_UP://stop
                shoot_cmd.ctrl_mode = SHOOT_STOP;
            shoot_cmd.friction_status = 0;
            break;
            case RC_MI://shoot one
                shoot_cmd.ctrl_mode = SHOOT_ONE;
                switch(rc_now->sw2) {
                    case RC_UP:
                        shoot_cmd.friction_speed = HIGH_FREQUENCY;
                        break;
                    case RC_DN:
                        shoot_cmd.friction_speed = LOW_FREQUENCY;
                        break;
                }
                shoot_cmd.friction_status = 1;
            break;
            case RC_DN://shoot reverse
                shoot_cmd.ctrl_mode = SHOOT_REVERSE;
                shoot_cmd.friction_status = 0;
        }
    }

    /*全局失能最后判断*/
    switch(rc_now->sw4)
    {//全机 UP：失能 DOWN：使能
        case RC_UP:
            chassis_cmd.ctrl_mode = CHASSIS_RELAX;
            shoot_cmd.ctrl_mode = SHOOT_STOP;
            shoot_cmd.friction_status = 0;
            break;
        case RC_DN:
            break;


    }

}
#endif




