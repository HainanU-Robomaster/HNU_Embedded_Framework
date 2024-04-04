/*
* Change Logs:
* Date            Author          Notes
* 2023-09-24      ChuShicheng     first version
*/
#ifndef _CMD_TASK_H
#define _CMD_TASK_H

#include <rtthread.h>
typedef struct
{
    float num[3];
    float error[2];
}gim_auto_judge;
void cmd_thread_entry(void *argument);


/**
 * @brief 板间通讯 CAN 反馈报文接收回调函数,该函数被can_rx_call调用
 *
 * @param dev 接收到报文的CAN设备
 * @param id 接收到的报文的id
 * @param data 接收到的报文的数据
 */
void gimbal_board_rx_callback(rt_device_t dev, uint32_t id, uint8_t *data);

#endif /* _CMD_TASK_H */
