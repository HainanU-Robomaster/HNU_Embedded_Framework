 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-09-15      ChuShicheng     first version
 */

#ifndef _INS_H
#define _INS_H

#include "stdint.h"
typedef struct
{
    float gyro[3];  // 角速度
    float accel[3]; // 加速度
    // 还需要增加角速度数据
    float roll;
    float pitch;
    float yaw;
    float yaw_total_angle;
} attitude_t; // 最终解算得到的角度,以及yaw转动的总角度(方便多圈控制)

typedef struct
{
    float q[4]; // 四元数估计值

    float motion_accel_b[3]; // 机体坐标加速度
    float motion_accel_n[3]; // 绝对系加速度

    float accel_lpf; // 加速度低通滤波系数

    // bodyframe在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    // 加速度在机体系和XY两轴的夹角
    // float atanxz;
    // float atanyz;

    // IMU量测值
    float gyro[3];  // 角速度
    float accel[3]; // 加速度
    // 位姿
    float roll;
    float pitch;
    float yaw;
    float yaw_total_angle;

    uint8_t init;
} ins_t;

/* 用于修正安装误差的参数 */
typedef struct
{
    uint8_t flag;

    float scale[3];

    float yaw;
    float pitch;
    float roll;
} imu_param_t;

/**
 * @brief 姿态解算任务主体，在主循环中被调用
 */
__attribute__((noreturn)) void ins_task_entry(void const *argument);

#endif // _INS_H
