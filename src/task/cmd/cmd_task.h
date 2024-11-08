/**
 * @file cmd_task.h
 * @brief  控制命令线程，robot只能执行cmd及进行反馈，不得改写控制指令
 * @date 2023-12-29
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef CMD_TASK_H
#define CMD_TASK_H

#include "stdio.h"

/**
 * @brief 控制指令任务初始化
 */
void cmd_task_init(void);

/**
 * @brief 控制指令任务,在RTOS中应该设定为200hz运行
 */
void cmd_control_task(void);

int chassis_board_rx_callback(uint32_t id, uint8_t *data);

#endif // CMD_TASK_H

