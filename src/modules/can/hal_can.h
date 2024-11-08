 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-12-23      ChuShicheng     first version
 */
#ifndef _HAL_CAN_H
#define _HAL_CAN_H

#include <stm32f4xx.h>

void CAN_send(CAN_HandleTypeDef *can, uint32_t send_id, uint8_t data[]);

/**
 * @brief 初始化并启动 CAN 服务
 *
 * @note 此函数会启动 CAN1 和 CAN2 ,开启 FIFO0 & FIFO1 接收通知
 *
 */
void CAN_service_init(void);

#endif /* _HAL_CAN_H */
