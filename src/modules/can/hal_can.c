 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-12-23      ChuShicheng     first version
 */
#include <string.h>
#include "hal_can.h"
#include "rm_module.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
static CAN_TxHeaderTypeDef  tx_message;


extern int chassis_board_rx_callback(uint32_t id, uint8_t *data);

void CAN_send(CAN_HandleTypeDef *can, uint32_t send_id, uint8_t data[])
{
    uint32_t send_mail_box;
    tx_message.StdId = send_id;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    while (HAL_CAN_GetTxMailboxesFreeLevel(can) == 0) // 等待邮箱空闲
    {
    }
    HAL_CAN_AddTxMessage(can, &tx_message, data, &send_mail_box);
}

/**
 * @brief 初始化并启动 CAN 服务
 *
 * @note 此函数会启动 CAN1 和 CAN2 ,开启 FIFO0 & FIFO1 接收通知
 *
 */
void CAN_service_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0)) // FIFO不为空,有可能在其他中断时有多帧数据进入
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
        if (hcan == &hcan1)
        {
            // if(dji_motot_rx_callback(rx_header.StdId, rx_data) == 0)
            //     return;

             if(lk_motot_rx_callback(rx_header.StdId, rx_data) == 0)
                 return;

//            if(ht_motot_rx_callback(rx_header.StdId, rx_data) == 0)
//                return;
        }
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1)) // FIFO不为空,有可能在其他中断时有多帧数据进入
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);
        // TODO: 2024 RMUL 期间部署上下板间通讯，之后需要优化调整
        if (hcan == &hcan2)
        {
            // if(dji_motot_rx_callback(rx_header.StdId, rx_data) == 0)
            //     return;

            if(ht_motot_rx_callback(rx_header.StdId, rx_data) == 0)
                return;

//            if(lk_motot_rx_callback(rx_header.StdId, rx_data) == 0)
//                return;

//            if(chassis_board_rx_callback(rx_header.StdId, rx_data) == 0)
//                return;
        }
    }
}
