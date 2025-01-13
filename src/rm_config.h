 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-08-23      ChuShicheng     first version
 * 2024-09-24      YouYekai        Add macro definition and add arguements'units
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
#define CHASSIS_PC_MOVE_RATIO_X 1.0f
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
#define GIMBAL_PC_MOVE_RATIO_PIT 0.1f
/* 云台yaw轴速度 */
#define GIMBAL_PC_MOVE_RATIO_YAW 0.5f

/* 遥控器拨杆对应档位值 */
#define RC_UP_VALUE 240
#define RC_MID_VALUE 0
#define RC_DN_VALUE 15

/* ---------------------------------- 底盘相关 ---------------------------------- */
#define SCREW_LEN  261.2f
#define ABOVE_POLE_LEN 482.5f
#define HEIGHT_LEN 304.3f
#define BOTTOM_POLE_LEN 357.2f

#define SQUARE_ABOVE_POLE_LEN 232806.25f
#define SQUARE_HEIGHT_LEN  92598.49f
#define DOUBLE_ABOVE_POLE_LEN 965.0f

/******** 底盘电机使用3508 *******/
/* 3508底盘电机减速比 */
#define YAW_GEAR_RATIO (1.0f/19.0f)
#define CHASSIS_DECELE_RATIO (1.0f/19.0f)
/* 单个电机速度极限，单位是分钟每转，8347rpm = 3500mm/s */
#define MAX_WHEEL_RPM        9000
/*M2006减速比*/
#define PITCH_GEAR_RATIO (1.0f/36.0f)
/*M2006转动一圈对应丝杆走过长度为2mm*/
#define ANGLE_TO_DISTANCE 0.002f

/* 底盘旋转最大速度，单位是度每秒 */
#define MAX_CHASSIS_VR_SPEED 8

/******** SBUS底盘小陀螺相关设置 *******/
/** 旋转速度=遥控器ch5值/功率限制比例 **/
/* 功率限制比例 */
#define SBUS_ROTATE_LIMIT_RATIO 784.0 * 5.0

/******** DBUS底盘小陀螺相关设置 *******/
/** 旋转速度=截距+倍率*底盘功率限制/功率限制比例 **/
/* 旋转速度截距 */
#define ROTATE_INTERCEPT 4.5
/* 旋转速度倍率 */
#define ROTATE_MULTIPLIER 2
/* 功率限制比例 */
#define ROTATE_LIMIT_RATIO 60

/* --------------------------------- 底盘PID参数 -------------------------------- */
/* 电机速度环 */
#define CHASSIS_KP_V_MOTOR              6
#define CHASSIS_KI_V_MOTOR              0
#define CHASSIS_KD_V_MOTOR              0.0001
#define CHASSIS_INTEGRAL_V_MOTOR        2000
#define CHASSIS_MAX_V_MOTOR             16000


/* ---------------------------------- 云台相关 ---------------------------------- */
#define YAW_MOTOR_ID     0x208
#define PITCH_MOTOR_ID   0x207

/*云台编码器归中*/
//#define GIMBAL_SIDEWAYS
#ifdef GIMBAL_SIDEWAYS
#define SIDEWAYS_ANGLE   36
/* 云台yaw轴编码器归中值(侧身) */
#define CENTER_ECD_YAW   3818
#else
/* 云台yaw轴编码器归中值 */
#define CENTER_ECD_YAW   7913
#define SIDEWAYS_ANGLE   0
#endif

/* 云台pitch轴编码器归中值 */
#define CENTER_ECD_PITCH 2717
/* pitch轴最大仰角 */
#define PIT_ANGLE_MAX    30.0f
/* pitch轴最大俯角 */
#define PIT_ANGLE_MIN    -20.0f

/* 云台控制周期 (ms) */
#define GIMBAL_PERIOD 1
/* 云台回中初始化时间 (ms) */
#define BACK_CENTER_TIME 100

/* -------------------------------- 云台电机PID参数 ------------------------------- */
/* 云台yaw轴电机PID参数 */
#define YAW_KP_V             6//10000
#define YAW_KI_V             0//0.5
#define YAW_KD_V             0.0001//10
#define YAW_INTEGRAL_V       2000//0
#define YAW_MAX_V            16000//30000

/* 云台PITCH轴电机PID参数 */
#define PITCH_KP_V           6//10000//5000
#define PITCH_KI_V           0//0.5//0.001
#define PITCH_KD_V           0.0001//10//0
#define PITCH_INTEGRAL_V     2000//0//1500
#define PITCH_MAX_V          16000//30000//30000

/* ---------------------------------- 发射相关 ---------------------------------- */
#define SHOOT1_MOTOR_ID 0x201
#define SHOOT2_MOTOR_ID 0x202
#define SHOOT3_MOTOR_ID 0x203
#define SHOOT4_MOTOR_ID 0x204
#define LOAD_MOTOR_ID 0x206
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
#define DBUS_FRICTION_AUTO_SPEED_L 3500
#define DBUS_FRICTION_AUTO_SPEED_H 7000
/* -------------------------------- 发射电机PID参数 ------------------------------- */
// TODO: 速度期望应改为变量应对速度切换。初次参数调整已完成
/* 右摩擦轮M3508电机PID参数 */
/* 速度环 */
#define RIGHT_KP_V             23
#define RIGHT_KI_V             0.1
#define RIGHT_KD_V             0.001f
#define RIGHT_INTEGRAL_V       50
#define RIGHT_MAX_V            30000

/* 左摩擦轮M3508电机PID参数 */
/* 速度环 */
#define LEFT_KP_V           23
#define LEFT_KI_V           0.1
#define LEFT_KD_V           0.001f
#define LEFT_INTEGRAL_V     50
#define LEFT_MAX_V          30000

// TODO：PID参数初次微调已完成，期待后续微调
/* 拨弹电机M2006电机PID参数 */
/* 速度环 */
#define TRIGGER_KP_V           10
#define TRIGGER_KI_V           5
#define TRIGGER_KD_V           0.01f
#define TRIGGER_INTEGRAL_V     1500
#define TRIGGER_MAX_V          20000
/* 角度环 */
#define TRIGGER_KP_A           5
#define TRIGGER_KI_A           0
#define TRIGGER_KD_A           0
#define TRIGGER_INTEGRAL_A     0
#define TRIGGER_MAX_A          10000

#endif /* _RM_CONFIG_H */
