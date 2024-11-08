# bsp_can

该模块基于 stm32 HAL 库实现 CAN 驱动，CAN1 使用 FIFO0，CAN2 使用 FIFO1 ，CAN 的相关初始会在 BOARD INiT 阶段进行，无需用户调用

## 常用功能

### 一帧 CAN 报文的发送

```c
void CAN_send(CAN_HandleTypeDef *can, uint32_t send_id, uint8_t data[]);
```

### CAN 报文的接收

CAN 报文的接收回调位于 user_callback 中：

```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
```
