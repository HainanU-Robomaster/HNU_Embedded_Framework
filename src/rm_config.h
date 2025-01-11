/*
* Change Logs:
* Date            Author          Notes
* 2023-08-23      ChuShicheng     first version
* 2024-10-23      YouYekai        Add macro definition and add arguements'units
*/
#ifndef _RM_CONFIG_H
#define _RM_CONFIG_H

/* CPU主频(mHZ) */
#define CPU_FREQUENCY 168

/* 底盘和云台分别对应的 can 设备名称 */
#define CAN_CHASSIS    "can1"
#define CAN_GIMBAL     "can2"

/* 磁力计所挂载的 i2c 设备名称(软件i2c) Notice: PA8 --> 8; PC9 --> 41 */
#define I2C_MAG        "i2c1"

/* 陀螺仪所挂载的 SPI 设备名称及 CS 引脚 */
#define SPI_GYRO       "spi1"
#define SPI_GYRO_CS    16

/* 加速度计所挂载的 SPI 设备名称及 CS 引脚 */
#define SPI_ACC        "spi1"
#define SPI_ACC_CS     4

/* 灯板所挂载的 SPI 设备名称及 CS 引脚 */
#define SPI_LED        "spi2"
#define SPI_LED_DEVICE "spi20"

/* 弹仓盖舵机所挂载的 PWM 设备及通道号 */
#define PWM_COVER        "pwm1"
#define PWM_COVER_CH     2

/* 遥控器所挂载的 usart 设备名称 */
#define USART_RC       "uart3"

/* ---------------------------------- 遥控器相关 --------------------------------- */
/* 遥控器通道最大值 */
#define RC_MAX_VALUE      784.0f
/* DBUS遥控器通道最大值 */
#define RC_DBUS_MAX_VALUE      660.0f
#define RC_RATIO               0.0009f
#define KB_RATIO               0.010f
/* 遥控器模式下的底盘最大速度限制 */
/* 底盘平移速度 */
#define CHASSIS_RC_MOVE_RATIO_X 1.0f
/* 底盘前进速度 */
#define CHASSIS_RC_MOVE_RATIO_Y 0.8f
/* 底盘旋转速度，只在底盘开环模式下使用 */
#define CHASSIS_RC_MOVE_RATIO_R 1.0f

/* 鼠标键盘模式下的底盘最大速度限制 */
/* 底盘平移速度 */
#define CHASSIS_PC_MOVE_RATIO_X 0.5f
/* 底盘前进速度 */
#define CHASSIS_PC_MOVE_RATIO_Y 1.0f
/* 底盘旋转速度，只在底盘开环模式下使用 */
#define CHASSIS_PC_MOVE_RATIO_R 5.0f

/* 遥控器模式下的云台速度限制 */
/* 云台pitch轴速度 */
#define GIMBAL_RC_MOVE_RATIO_PIT 0.5f
/* 云台yaw轴速度 */
#define GIMBAL_RC_MOVE_RATIO_YAW 0.5f

/* 鼠标键盘模式下的云台速度限制 */
/* 云台pitch轴速度 */
#define GIMBAL_PC_MOVE_RATIO_PIT 0.3f
/* 云台yaw轴速度 */
#define GIMBAL_PC_MOVE_RATIO_YAW 0.5f

/* 遥控器拨杆对应档位值 */
#define RC_UP_VALUE 240
#define RC_MID_VALUE 0
#define RC_DN_VALUE 15

/* ---------------------------------- 底盘相关 ---------------------------------- */
/* 底盘轮距(mm) */
#define WHEELTRACK 340
/* 底盘轴距(mm) */
#define WHEELBASE 388
/* 底盘轮子周长(mm) */
#define WHEEL_PERIMETER 471
/* 底盘的半径(mm) */
#define LENGTH_RADIUS 245

/******** 底盘电机使用3508 *******/
/* 3508底盘电机减速比 */
#define CHASSIS_DECELE_RATIO (1.0f/19.0f)
/* 单个电机速度极限，单位是分钟每转，8347rpm = 3500mm/s */
#define MAX_WHEEL_RPM        9000

/******** 底盘最大速度设置 *******/
/* 底盘移动最大速度，单位是毫米每秒 */
#define MAX_CHASSIS_VX_SPEED 5000
#define MAX_CHASSIS_VY_SPEED 5000
/* 高速档位（mm/s） */
#define MAX_CHASSIS_VX_SPEED_HIGH 11000
#define MAX_CHASSIS_VY_SPEED_HIGH 11000
/* 低速档位（mm/s） */
#define MAX_CHASSIS_VX_SPEED_LOW 5000
#define MAX_CHASSIS_VY_SPEED_LOW 5000

/******** SBUS底盘小陀螺相关设置 *******/
/** 旋转速度=遥控器ch5值/功率限制比例 **/
/* 功率限制比例 */
#define SBUS_ROTATE_LIMIT_RATIO 784.0 * 5.0

/* 底盘旋转最大速度，单位是度每秒 */
#define MAX_CHASSIS_VR_SPEED 8
/* 旋转速度,rad/s */
#define ROTATE_RATIO_VW 4
/******** DBUS底盘小陀螺相关设置 *******/
/*旋转速度,rad/s*/
#define ROTATE_RATIO 4

/* --------------------------------- 底盘PID参数 -------------------------------- */
/* 电机速度环 */
#define CHASSIS_KP_V_MOTOR              12
#define CHASSIS_KI_V_MOTOR              22.6908896696549f
#define CHASSIS_KD_V_MOTOR              0
#define CHASSIS_INTEGRAL_V_MOTOR        8000
#define CHASSIS_MAX_V_MOTOR             16000
// TODO: 参数待整定
/* 跟随云台PID */
#define CHASSIS_KP_V_FOLLOW             0.3205f
#define CHASSIS_KI_V_FOLLOW             0
#define CHASSIS_KD_V_FOLLOW             0.0055639f
#define CHASSIS_INTEGRAL_V_FOLLOW       0
#define CHASSIS_MAX_V_FOLLOW            1500

/* ---------------------------------- 云台相关 ---------------------------------- */
#define YAW_MOTOR_ID     0x20B
#define PITCH_MOTOR_ID   0x207
#define YAW_DOWN_MOTOR_ID   0x209

#define CENTER_ECD_YAW   6128         //云台yaw轴编码器归中值
#define CENTER_ECD_PITCH 4142         //云台pitch轴编码器归中值
#define CENTER_ECD_YAW_DOWN   1328     //云台yaw_down轴编码器归中值

/* pitch轴最大仰角 */
#define PIT_ANGLE_MAX        14.0f
/* pitch轴最大俯角 */
#define PIT_ANGLE_MIN        -20.0f

/* 云台控制周期 (ms) */
#define GIMBAL_PERIOD 1
/* 云台回中初始化时间 (ms) */
#define BACK_CENTER_TIME 10

/* -------------------------------- 云台电机PID参数 ------------------------------- */
/*auto模式下的PID和手动模式PID参数实际上不能共用一套,要单独操作*/
/* 云台yaw轴电机PID参数 */
/* imu速度环 */
#define YAW_KP_V_IMU             10000
#define YAW_KI_V_IMU             20000
#define YAW_KD_V_IMU             0
#define YAW_INTEGRAL_V_IMU       0
#define YAW_MAX_V_IMU            30000
/* imu角度环 */
#define YAW_KP_A_IMU             0.494710033270487f
#define YAW_KI_A_IMU             0.218443192644351f
#define YAW_KD_A_IMU             0
#define YAW_INTEGRAL_A_IMU       0
#define YAW_MAX_A_IMU            25
/* auto速度环 */
#define YAW_KP_V_AUTO            10000
#define YAW_KI_V_AUTO            20000
#define YAW_KD_V_AUTO            0
#define YAW_INTEGRAL_V_AUTO      0
#define YAW_MAX_V_AUTO           30000
/* auto角度环 */
#define YAW_KP_A_AUTO            0.494710033270487f
#define YAW_KI_A_AUTO            0.218443192644351f
#define YAW_KD_A_AUTO            0
#define YAW_INTEGRAL_A_AUTO      0
#define YAW_MAX_A_AUTO           25

/* 云台PITCH轴电机PID参数 */
/* imu速度环 */
#define PITCH_KP_V_IMU           1000
#define PITCH_KI_V_IMU           23000
#define PITCH_KD_V_IMU           0
#define PITCH_INTEGRAL_V_IMU     1500
#define PITCH_MAX_V_IMU          20000
/*
#define PITCH_KP_V_IMU           4250
#define PITCH_KI_V_IMU           1000
#define PITCH_KD_V_IMU           3
#define PITCH_INTEGRAL_V_IMU     1500
#define PITCH_MAX_V_IMU          20000
*/

/* imu角度环 */
#define PITCH_KP_A_IMU           0.70f
#define PITCH_KI_A_IMU           0.08009f
#define PITCH_KD_A_IMU           0.00000032f
#define PITCH_INTEGRAL_A_IMU     0.0f
#define PITCH_MAX_A_IMU          20
/* auto速度环 */
#define PITCH_KP_V_AUTO          1000
#define PITCH_KI_V_AUTO          23000
#define PITCH_KD_V_AUTO          0
#define PITCH_INTEGRAL_V_AUTO    1500
#define PITCH_MAX_V_AUTO         20000
/* auto角度环 */
#define PITCH_KP_A_AUTO          0.70f
#define PITCH_KI_A_AUTO          0.08009f
#define PITCH_KD_A_AUTO          0.00000032f
#define PITCH_INTEGRAL_A_AUTO    0.0f
#define PITCH_MAX_A_AUTO         20

/* 下云台yaw_down轴电机PID参数 */
/* imu速度环 */
#define YAW_DOWN_KP_V_IMU             10000
#define YAW_DOWN_KI_V_IMU             20000
#define YAW_DOWN_KD_V_IMU             0
#define YAW_DOWN_INTEGRAL_V_IMU       0
#define YAW_DOWN_MAX_V_IMU            30000
/* imu角度环 */
#define YAW_DOWN_KP_A_IMU             0.494710033270487f
#define YAW_DOWN_KI_A_IMU             0.218443192644351f
#define YAW_DOWN_KD_A_IMU             0
#define YAW_DOWN_INTEGRAL_A_IMU       0
#define YAW_DOWN_MAX_A_IMU            25
/* auto速度环 */
#define YAW_DOWN_KP_V_AUTO            10000
#define YAW_DOWN_KI_V_AUTO            20000
#define YAW_DOWN_KD_V_AUTO            0
#define YAW_DOWN_INTEGRAL_V_AUTO      0
#define YAW_DOWN_MAX_V_AUTO           30000
/* auto角度环 */
#define YAW_DOWN_KP_A_AUTO            0.494710033270487f
#define YAW_DOWN_KI_A_AUTO            0.218443192644351f
#define YAW_DOWN_KD_A_AUTO            0
#define YAW_DOWN_INTEGRAL_A_AUTO      0
#define YAW_DOWN_MAX_A_AUTO           25

/* ---------------------------------- 发射相关 ---------------------------------- */
// TODO: 实际值待整定
#define RIGHT_FRICTION_MOTOR_ID     0x201
#define LEFT_FRICTION_MOTOR_ID   0x202
#define TRIGGER_MOTOR_ID  0x203

/*M2006的减速比为36:1，因此转轴旋转45度，要在转子的基础上乘36倍*/
#define TRIGGER_MOTOR_45_TO_ANGLE 45 * 36

/** SBUS遥控器发射速度 **/
#define SBUS_FRICTION_LAUNCH_SPEED 5000
/* 拨弹电机参数*/
#define SBUS_SHOOT_REVERSE_SPEED 2500
/** COUNTINUE模式参数 **/
#define SBUS_FRICTION_AUTO_SPEED_L 3500
#define SBUS_FRICTION_AUTO_SPEED_H 7000

/** DBUS遥控器发射速度 **/
#define DBUS_FRICTION_LAUNCH_SPEED 7000
#define DBUS_SHOOT_REVERSE_SPEED 3000
/** COUNTINUE模式参数 **/
#define DBUS_FRICTION_AUTO_SPEED_L 2500
#define DBUS_FRICTION_AUTO_SPEED_H 4500
/* -------------------------------- 发射电机PID参数 ------------------------------- */
// TODO: 速度期望应改为变量应对速度切换。初次参数调整已完成
/* 右摩擦轮M3508电机PID参数 */
/* 速度环 */
#define RIGHT_KP_V             30
#define RIGHT_KI_V             0.059f
#define RIGHT_KD_V             0.0012f
#define RIGHT_INTEGRAL_V       50
#define RIGHT_MAX_V            30000

/* 左摩擦轮M3508电机PID参数 */
/* 速度环 */
#define LEFT_KP_V           30
#define LEFT_KI_V           0.059f
#define LEFT_KD_V           0.0012f
#define LEFT_INTEGRAL_V     50
#define LEFT_MAX_V          30000

// TODO：PID参数初次微调已完成，期待后续微调
/* 拨弹电机M2006电机PID参数 */
/* 速度环 */
#define TRIGGER_KP_V           10
#define TRIGGER_KI_V           5
#define TRIGGER_KD_V           0.01f
#define TRIGGER_INTEGRAL_V     1000
#define TRIGGER_MAX_V          20000
/* 角度环 */
#define TRIGGER_KP_A           5
#define TRIGGER_KI_A           0
#define TRIGGER_KD_A           0
#define TRIGGER_INTEGRAL_A     0
#define TRIGGER_MAX_A          10000

// 以下编码准寻· 光学色相环(RGB模型)-12色
#define red_0 0xFF0000             // 红
#define brown_30 0xFF7F00          // 棕色
#define yellow_60 0xFFFF00         // 黄色
#define dark_green_90 0x7FFF00     // 深绿色
#define medium_green_120 0x00FF00  // 中绿色
#define light_green_150 0x00FF7F   // 浅绿色
#define baby_blue_180 0x00FFFF     // 浅蓝色
#define medium_blue_210 0x007FFF   // 中蓝色
#define dark_blue_240 0x0000FF     // 深蓝色
#define modena_270 0x7F00FF        // 深紫色
#define medium_purple_300 0xFF00FF // 中紫色
#define lilac_330 0xFF007F         // 浅紫色
#define white_360 0xFFFFFF         // 白色

#endif /* _RM_CONFIG_H */
