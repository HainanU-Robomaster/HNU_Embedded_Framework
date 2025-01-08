 /**
 * @file rm_algorithm.h
 * @author ChuShicheng
 * @author modified by neozng
 * @brief  RM电控算法库,仅被应用层调用
 * @date 2023-09-04
 */

 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-09-04      ChuShicheng     first version
 */
#ifndef _RM_TASK_H
#define _RM_TASK_H

#include <rtthread.h>

#ifdef BSP_USING_EXAMPLE_TASK
#include "example_task.h"
#endif /* BSP_USING_EXAMPLE_TASK */
#ifdef BSP_USING_INS_TASK
#include "ins_task.h"
#endif /* BSP_USING_INS_TASK */
#ifdef BSP_USING_MOTOR_TASK
#include "motor_task.h"
#endif /* BSP_USING_MOTOR_TASK */
#ifdef BSP_USING_CHASSIS_TASK
#include "chassis_task.h"
#endif /* BSP_USING_CHASSIS_TASK */
#ifdef BSP_USING_CMD_TASK
#include "cmd_task.h"
#endif /* BSP_USING_CMD_TASK */
#ifdef BSP_USING_TRANSMISSION_TASK
#include "transmission_task.h"
#endif /* BSP_USING_TRANSMISSION_TASK */
#ifdef BSP_USING_SHOOT_TASK
#include "shoot_task.h"
#endif /* BSP_USING_SHOOT_TASK */
#ifdef BSP_USING_REFEREE_TASK
#include "referee_task.h"
#include "Referee_system.h"
#endif /* BSP_USING_REFEREE_TASK */


/* --------------------------------- 话题的数据格式 -------------------------------- */
struct ins_msg
{
    // IMU量测值
    float gyro[3];  // 角速度
    float accel[3]; // 加速度
    // 位姿
    float roll;
    float pitch;
    float yaw;
    float yaw_total_angle;
};

/**
 * @brief cmd发布的云台控制数据,由shoot订阅
 */
struct shoot_cmd_msg
{ // 发射器
    shoot_mode_e ctrl_mode;  // 当前发射器控制模式
    shoot_mode_e last_mode;  // 上一次发射器控制模式
    // trigger_mode_e trigger_status;
    rt_bool_t friction_status;//摩擦轮开关
    shoot_frequency_e friction_speed;//摩擦轮高低转速控制
};



/* ------------------------------ chassis反馈状态数据 ------------------------------ */
/**
 * @brief 底盘真实反馈状态数据,由chassis发布
 */
 struct chassis_cmd_msg
 {
     /* pos, controled by degree*/
     float yaw;
     float pitch;
     /*v, controled by value of ch1 2 */
     float vw_yaw;
     float vw_pitch;

     float offset_angle;        // 底盘和归中位置的夹角
     chassis_mode_e ctrl_mode;  // 当前云台控制模式
     chassis_mode_e last_mode;  // 上一次云台控制模式
 };

struct chassis_fdb_msg
{
    // float yaw_pos;
    // float pitch_pos;
    float yaw_degree;
    float pitch_degree;
    chassis_back_e back_mode;
};

/* ------------------------------ shoot反馈状态数据 ------------------------------ */
/**
 * @brief 发射机真实反馈状态数据,由shoot发布
 */
struct shoot_fdb_msg
{
    shoot_back_e trigger_status;  // shoot状态反馈
    load_back_e load_status;//load moter fdb
    // int16_t trigger_motor_current; //拨弹电机电流，传给cmd控制反转
};

/* ------------------------------ transmission反馈状态数据 ------------------------------ */
/**
 * @brief 上位机反馈状态数据,由transmission发布
 */
 // struct trans_fdb_msg
 // { // 云台自瞄角度控制
 //     float yaw;
 //     float pitch;
 //     float roll;
 //     rt_uint8_t heartbeat;
 // };


#endif /* _RM_TASK_H */
