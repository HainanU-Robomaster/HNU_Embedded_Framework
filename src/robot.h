#ifndef ROBOT_H
#define ROBOT_H

#include "rm_module.h"
#include "chassis_task.h"

/**
 * @brief 机器人初始化,请在开启rtos之前调用
 * 
 */
void robot_init();

/**
 * @brief 机器人任务,放入实时系统以一定频率运行,内部会调用各个应用的任务
 * 
 */
void robot_task();

/* ------------------------------- ipc uMCN 相关 ------------------------------ */
struct ins_msg
{
    // IMU量测值
    float gyro[3];  // 角速度
    float accel[3]; // 加速度
    float motion_accel_b[3]; // 机体坐标加速度
    // 位姿
    float roll;
    float pitch;
    float yaw;
    float yaw_total_angle;
};

// TODO：后续优化启用，目前时间紧急，使用extern
struct referee_msg
{
    robot_status_t robot_status;
    ext_power_heat_data_t power_heat_data_t;
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
    // TODO: 轮腿前期调试使用
    float leg_length;          // 腿长
    float leg_angle;           // 腿角度
    float offset_angle;        // 底盘和归中位置的夹角
    chassis_mode_e ctrl_mode;  // 当前底盘控制模式
    chassis_mode_e last_mode;  // 上一次底盘控制模式
    leg_level_e leg_level;     // 腿长等级
};

/* ------------------------------ chassis反馈状态数据 ------------------------------ */
/**
 * @brief 底盘真实反馈状态数据,由chassis发布
 */
struct chassis_fdb_msg
{
    leg_back_state_e leg_state;  // 腿部归中初始化情况
    chassis_stand_state_e stand_state;  // 机器人站立状态
    /*  底盘任务使用到的电机句柄,仅能对其 measure 成员当作传感器数据读取，禁止改写 */
    lk_motor_measure_t lk_l;   // 左轮毂电机
    lk_motor_measure_t lk_r;   // 右轮毂电机
    bool touch_ground;         // 是否触地
};

/* ------------------------------ trans解析自瞄数据 ------------------------------ */
/**
 * @brief 上位机自瞄数据,由trans发布
 */
struct trans_fdb_msg
{
   float yaw;
   float pitch;
};

#endif