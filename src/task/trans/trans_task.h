/**
 * @file trans_task.h
 * @brief  上下位机通讯线程，实现与上位机的双向通讯任务
 * @date 2023-12-29
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef TRANS_TASK_H
#define TRANS_TASK_H


/* BCP通讯协议相关 */
//TODO: 考虑不同帧长的情况
#include "rm_config.h"

#define FRAME_NUM     10        /* 所有通讯帧的类型总数 */
#define FRAME_MAX_LEN 36        /* 通讯帧的最大长度 */
#define FRAME_RPY_LEN 25        /* 欧拉角rpy方式控制长度 */
#define FRAME_ODOM_LEN 36       /* 里程计控制方式长度 */
#define FRAME_IMU_LEN 24        /* imu控制方式长度 */
#define FRAME_CTRL_LEN 24       /* 角/线速度控制方式长度 */
/* 目标地址表 */
#define BROADCAST   0x00        /* 广播 */
#define MAINFLOD    0x01        /* 上位机 */
#define SENTRY_UP   0x02        /* 哨兵机器人上云台 */
#define SENTRY_DOWN 0x03        /* 哨兵机器人下云台 */
#define INFANTRY    0x04        /* 步兵机器人 */
#define ENGINEER    0x05        /* 工程机器人 */
#define HERO        0x06        /* 英雄机器人 */
#define AIR         0x07        /* 空中机器人 */
#define RADAR       0x08        /* 雷达站 */
#define GATHER      0x09        /* 视觉采集台 */
#define STANDARD    0x10        /* AI机器人/全自动步兵机器人 */
/* 功能码表 */
#define CHASSIS                 0x10        /* 速度方式控制 */
#define CHASSIS_ODOM            0x11        /* 里程计方式控制 */
#define CHASSIS_CTRL            0x12        /* 角/线速度方式控制 */
#define CHASSIS_IMU             0x13        /* 底盘imu数据 */
#define GIMBAL                  0x20        /* 欧拉角rpy方式控制 */
#define GAME_STATUS             0x30        /* 比赛类型数据*/
#define ROBOT_HP                0x31        /* 机器人血量数据 */
#define ICRA_BUFF_DEBUFF_ZONE   0x32        /* 增益区数据 */
#define GAME_MODE               0x33        /* 机器人颜色数据 */
#define ROBOT_COMMAND           0x34        /* 机器人位置信息 */
#define CLIENT_MAP_COMMAND      0x35        /* 雷达发送目标位置信息 */
#define BARREL                  0x40        /* 发射机构数据 */
#define MANIFOLD_CTRL           0x50        /* 控制模式 */
#define MODE                    0x60        /* 模式控制 */
#define DEV_ERROR               0xE0        /* 故障信息 */
#define HEARTBEAT               0xF0        /* 心跳数据 */


/**
  * @brief  通讯帧结构体 （BCP通讯协议） 此为最大DATA长度的帧，用于接收中转
  */
typedef  struct
{
    uint8_t HEAD;  				    /*! 帧头 */
    uint8_t D_ADDR;                 /*! 目标地址 */
    uint8_t ID;                     /*! 功能码 */
    uint8_t LEN;                    /*! 数据长度 */
    int8_t DATA[FRAME_MAX_LEN];     /*! 数据内容 */
    uint8_t SC;                     /*! 和校验 */
    uint8_t AC;                     /*! 附加校验 */
}__attribute__((packed)) FrameTypeDef;

/**
  * @brief  欧拉角方式控制通讯帧结构体
  */
typedef  struct
{
    uint8_t HEAD;  				    /*! 帧头 */
    uint8_t D_ADDR;                 /*! 目标地址 */
    uint8_t ID;                     /*! 功能码 */
    uint8_t LEN;                    /*! 数据长度 */
    int8_t DATA[FRAME_RPY_LEN];     /*! 数据内容 */
    uint8_t SC;                     /*! 和校验 */
    uint8_t AC;                     /*! 附加校验 */
}__attribute__((packed)) RpyTypeDef;
extern int recv_flag;    //虚拟串口接收标志位
extern FrameTypeDef upper_rx_data;       //接收上位机数据中转帧
extern RpyTypeDef rpy_rx_data;  //接收欧拉角方式控制数据帧
/**
 * @brief 上下位机通讯任务初始化
 */
void trans_task_init(void);

/**
 * @brief 上下位机通讯任务
 */
void trans_control_task(void);

#endif //HNU_RM_DOWN_TRANS_TASK_H