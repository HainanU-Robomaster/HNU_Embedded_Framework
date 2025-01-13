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
#ifdef BSP_USING_CMD_TASK
#include "cmd_task.h"
#endif /* BSP_USING_CMD_TASK */
#ifdef BSP_USING_CHASSIS_TASK
#include "chassis_task.h"
#endif /* BSP_USING_CHASSIS_TASK */
#ifdef BSP_USING_GIMBAL_TASK
#include "gimbal_task.h"
#endif /* BSP_USING_GIMBAL_TASK */
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
#ifdef BSP_USING_INA226_TASK
#include "example_ina226.h"
#endif /* BSP_USING_INA226_TASK */

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

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief cmd发布的底盘控制数据,由chassis订阅
 */
struct chassis_cmd_msg
{
    float vx;                  // 前进方向速度
    float vy;                  // 横移方向速度
    float vw;                  // 旋转速度
    float offset_angle;        // 底盘和归中位置的夹角
    chassis_mode_e ctrl_mode;  // 当前底盘控制模式
    chassis_mode_e last_mode;  // 上一次底盘控制模式
};

/**
 * @brief cmd发布的云台控制数据,由gimbal订阅
 */
struct gimbal_cmd_msg
{ // 云台期望角度控制
    float yaw;
    float pitch;
    gimbal_mode_e ctrl_mode;  // 当前云台控制模式
    gimbal_mode_e last_mode;  // 上一次云台控制模式
};

/**
 * @brief cmd发布的云台控制数据,由shoot订阅
 */
struct shoot_cmd_msg
{ // 发射器
    shoot_mode_e ctrl_mode;  // 当前发射器控制模式
    shoot_mode_e last_mode;  // 上一次发射器控制模式
    trigger_mode_e trigger_status;
    int16_t shoot_freq;      // 发射弹频
    // TODO: 添加发射弹速控制
    int16_t shoot_speed;     // 发射弹速
    uint8_t cover_open;      // 弹仓盖开关
    uint8_t mirror_enable;     // 倍镜使能开关
    rt_bool_t friction_status;
};

/* ------------------------------ gimbal反馈状态数据 ------------------------------ */
/**
 * @brief 云台真实反馈状态数据,由gimbal发布
 */
struct gimbal_fdb_msg
{
    gimbal_back_e back_mode;  // 云台归中情况

    float yaw_offset_angle_total;    //云台初始 yaw 轴角度 （由imu得）
    float yaw_offset_angle;    //云台初始 yaw 轴角度 （由imu得）
    float pit_offset_angle;    //云台初始 pit 轴角度 （由imu得）
    float yaw_relative_angle;  //云台相对于初始位置的yaw轴角度
};

/* ------------------------------ chassis反馈状态数据 ------------------------------ */
/**
 * @brief 底盘真实反馈状态数据,由chassis发布
 */
 struct chassis_fdb_msg
 {
     float x_pos_gim;
     float y_pos_gim;
 };

/* ------------------------------ shoot反馈状态数据 ------------------------------ */
/**
 * @brief 发射机真实反馈状态数据,由shoot发布
 */
struct shoot_fdb_msg
{
    shoot_back_e trigger_status;  // shoot状态反馈
    int16_t trigger_motor_current; //拨弹电机电流，传给cmd控制反转
    int shoot_cnt;
};

/* ------------------------------ transmission反馈状态数据 ------------------------------ */
/**
 * @brief 上位机反馈状态数据,由transmission发布
 */
 struct trans_fdb_msg
 { // 云台自瞄角度控制
     float yaw;
     float pitch;
     rt_uint8_t heartbeat;
 };

 /* ------------------------------ referee反馈状态数据 ------------------------------ */
/**
 * @brief 上位机反馈状态数据,由referee发布
 */
 struct referee_fdb_msg
 {
     robot_status_t robot_status;
     ext_power_heat_data_t power_heat_data;
 };
#endif /* _RM_TASK_H */
