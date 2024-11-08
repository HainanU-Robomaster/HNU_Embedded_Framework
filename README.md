# HNU_RMUL_DOWN_2022

#### 介绍
海南大学2022赛季联盟赛，重构的步兵底盘代码

#### 软件架构

1. Tasks文件中为主要的任务函数及bsp文件
2. Transmission.c中主要包含底盘和云台以及超级电容之间的通信
3. 云台和底盘的通信的接受回调部分在bsp_can.c中

#### 使用说明

1. 开机前，将遥控器左SW1拨杆拨至最上

   ![模式转换](images/%E6%A8%A1%E5%BC%8F%E8%BD%AC%E6%8D%A2.jpg)

#### 通讯协议：

由于23赛季哨兵的改动，各模块之间通讯更加重要，于是采用了 [BCP通讯协议](https://birdiebot.github.io/bubble_documentation/guide/%E6%9C%A8%E9%B8%A2%E9%80%9A%E8%AE%AF%E5%8D%8F%E8%AE%AE.html) ，并在此基础上根据实际需求进行改动。

下位机与上位机之间通过USB虚拟串口进行连接，下位机通过 CDC_Receive_FS 进行接收，通过 CDC_Transmit_FS 进行发送，在 transmission_task 中装填需要发送的数据。

为便于上位机分辨下位机，将stm32的USB PID（16进制）更改为： 云台：573f  底盘：5740

##### 相关 API：

```
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
}__attribute__((packed)) BCPFrameTypeDef;

/**
 * @brief BCP和校验算法
 * @param frame
 * @retval 合并为一个数据的校验码 SC:和校验（高8位） AC:附加和校验（低8位）
 */
uint16_t Sumcheck_Cal(BCPFrameTypeDef frame);

/**
 * @brief 将要发送的数据类型填入到对应的数据帧中
 * @param send_mode 功能码表
 * @param data_buf 填装好的DATA
 */
void Add_Frame_To_Upper(uint16_t send_mode, int8_t* data_buf);
```

##### 具体改动：

- ```
  #define CHASSIS_CTRL            0x12        /* 角/线速度方式控制 */
  #define FRAME_CTRL_LEN 24       /* 角/线速度控制方式长度 */
  /* 传输底盘的速度Vx Vy Vw */
  
  #define CHASSIS_IMU             0x13        /* 底盘imu数据 */
  #define FRAME_IMU_LEN 40        /* imu控制方式长度 */
  /* 传输底盘的imu信息 顺序为：四元数 gyro accel */
  ```

