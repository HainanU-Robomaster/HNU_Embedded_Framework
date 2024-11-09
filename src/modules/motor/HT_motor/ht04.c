 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-10-16      ChuShicheng     first version
 */
#include "ht04.h"
#include <string.h>
#include <stdio.h>
#include "rm_config.h"
#include "rm_algorithm.h"

#define HT_MOTOR_CNT 4
/* 滤波系数设置为1的时候即关闭滤波 */
#define CURRENT_SMOOTH_COEF 0.9f
#define SPEED_BUFFER_SIZE 5
#define HT_SPEED_BIAS -0.0109901428f // 电机速度偏差,单位rad/s

#define P_MIN -95.5f   // Radians
#define P_MAX 95.5f
#define V_MIN -45.0f   // Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f    // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f    // N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN -18.0f   // N·m
#define T_MAX 18.0f

static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用
static ht_motor_object_t ht_motor_obj[HT_MOTOR_CNT];
static osThreadId ht_task_handle[HT_MOTOR_CNT];

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static void pack_contol_para(ht_motor_para_t para, uint8_t *buf);

/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
/**
  * @brief  Converts a float to an unsigned int, given range and number of bits
  */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
/**
  * @brief  converts unsigned int to float, given range and number of bits
  */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief 设置电机模式,报文内容[0xff,0xff,0xff,0xff,0xff,0xff,0xff,cmd]
 *
 * @param cmd
 * @param motor
 */
static void motor_set_mode(ht_motor_object_t *motor, ht_motor_mode_e cmd)
{
    if(motor->ctrl_mode == cmd) return; // 电机已经处于该模式,直接返回

    motor->to_mode = cmd;
    osSemaphoreWait(motor->turn_complete, 20);
}

 static float fdb_dt[4];
 static float fdb_start[4];

/**
 * @brief 根据返回的can_object对反馈报文进行解析
 *
 * @param object 收到数据的object,通过遍历与所有电机进行对比以选择正确的实例
 */
static void motor_decode(ht_motor_object_t *motor, uint8_t *data)
{
    /*fdb_dt[motor->tx_id-1] = dwt_get_time_us() - fdb_start[motor->tx_id-1];
    fdb_start[motor->tx_id-1] = dwt_get_time_us();*/

    uint16_t var;   // 解析数据的中间变量
    uint8_t *rxbuff = data;
    ht_motor_measure_t *measure = &motor->measure; // measure要多次使用,保存指针减小访存开销

//    rt_timer_start(motor->timer);  // 判定电机未离线，重置电机定时器

    // 解析数据并对电流和速度进行滤波,电机的反馈报文具体格式见电机说明手册
    measure->last_angle = measure->total_angle;
    var = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->total_angle = uint_to_float(var, P_MIN, P_MAX, 16);

    var = (uint16_t)(rxbuff[3] << 4) | (rxbuff[4] >> 4);
    measure->speed_rads = AverageFilter((uint_to_float(var, V_MIN, V_MAX, 12) - HT_SPEED_BIAS), measure->speed_buf, SPEED_BUFFER_SIZE);

    var = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->real_current = CURRENT_SMOOTH_COEF * uint_to_float(var, T_MIN, T_MAX, 12) +
                            (1 - CURRENT_SMOOTH_COEF) * measure->real_current;
}

//TODO: 添加具体的电机超时处理，如报警或停止等，以及影响机器人状态机等
/**
 * @brief 电机定时器超时回调函数
 * @param motor_ptr
 */
static void motor_lost_callback(void *motor_ptr)
{
    ht_motor_object_t *motor = (ht_motor_object_t *)motor_ptr;
//    ht_motor_relax(motor);
//    LOG_W("Motor lost, can bus [%x] , id 0x[%x]", motor->can, motor->tx_id);
}

/**
 * @brief 电机反馈报文接收回调函数,该函数被can_rx_call调用
 *
 * @param dev 接收到报文的CAN设备
 * @param id 接收到的报文的id
 * @param data 接收到的报文的数据
 */
int ht_motot_rx_callback(uint32_t id, uint8_t *data){
   // 找到对应的实例后再调用motor_decode进行解析
   for (size_t i = 0; i < idx; ++i)
   {   /* 详见HT04电机手册反馈报文 */
       if (ht_motor_obj[i].tx_id == data[0])
       {
           motor_decode(&ht_motor_obj[i], data);
           return 0;
       }
   }

   return -1; // 未找到对应的电机实例
}

void ht_motor_disable_all()
{
    for (size_t i = 0; i < idx; i++)
    {
        motor_set_mode(&ht_motor_obj[i], CMD_RESET_MODE);
    }
}

void ht_motor_enable_all()
{
    for (size_t i = 0; i < idx; i++)
    {
        motor_set_mode(&ht_motor_obj[i], CMD_MOTOR_MODE);
    }
}

/**
 * @brief 海泰电机设置启停模式
 */
void ht_motor_set_type(ht_motor_object_t *motor, motor_working_type_e type)
{
    motor->stop_flag = type;
}

// 运算电机实例的控制器,发送控制报文
static void ht_motor_control(void const *parameter)
{
    ht_motor_object_t *motor = (ht_motor_object_t *)parameter;
    ht_motor_measure_t measure = motor->measure;
    ht_motor_para_t set; // 电机控制器计算得到的控制参数
    ht_motor_para_t control_get; // 电机控制器计算得到的控制参数
    uint8_t data_buf[8];

    PrintLog("[freeRTOS] HT Task is start [%x]\n", &motor->tx_id);
//    while (1)
    {
        fdb_dt[motor->tx_id-1] = dwt_get_time_us() - fdb_start[motor->tx_id-1];
        fdb_start[motor->tx_id-1] = dwt_get_time_us();
        control_get = motor->control(measure); // 调用对接的电机控制,保证控制频率

        /* 首先检查是否需要切换模式 */
        if (motor->to_mode != motor->ctrl_mode)
        {
            memset(data_buf, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
            data_buf[7] = (uint8_t)motor->to_mode; // 最后一位是命令id
            CAN_send(motor->can, motor->tx_id, data_buf);  // 发送报文
            motor->ctrl_mode = motor->to_mode;  // 切换模式成功
            osSemaphoreRelease(motor->turn_complete);
        }
        else /* 不需要切换模式情况，发送控制值 */
        {
            if (motor->ctrl_mode == CMD_MOTOR_MODE) {
                set = control_get;
                if (motor->stop_flag == MOTOR_STOP) {
                    memset(&set, 0, sizeof(ht_motor_para_t));
                }
                pack_contol_para(set, data_buf); // 将控制参数打包成报文数据帧
                CAN_send(motor->can, motor->tx_id, data_buf);  // 发送报文
            }
        }
    }
}


void ht_controll_all_poll(void)
{
    static uint8_t i;
    ht_motor_control(&ht_motor_obj[i++]);
    if(i==4)
        i=0;
}

/**
 * @brief 为了避免总线堵塞,为每个电机创建一个发送任务
 * @param argument 传入的电机指针
 */
void ht_motor_task_init()
{
    char ht_task_name[3] = "ht";
    char name_buf[5];
    // 遍历所有电机实例,创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        sprintf(name_buf, "%s%d", ht_task_name, ht_motor_obj[i].tx_id);
        osThreadDef(name_buf, ht_motor_control, osPriorityNormal, 0, 128);
        ht_task_handle[i] = osThreadCreate(osThread(name_buf), &ht_motor_obj[i]);
    }
}

/**
 * @brief 电机初始化,返回一个电机实例
 * @param config 电机配置
 * @return ht_motor_object_t* 电机实例指针
 */
ht_motor_object_t *ht_motor_register(motor_config_t *config, void *control)
{
    // 对接用户配置的 motor_config
    ht_motor_obj[idx].motor_type = config->motor_type;             // HT04
    ht_motor_obj[idx].rx_id = config->rx_id;                       // 接收报文的ID(主收)
    ht_motor_obj[idx].tx_id = config->tx_id;                       // 发送报文的ID(主发)
    ht_motor_obj[idx].control = control;                           // 电机控制器执行
    ht_motor_obj[idx].set_mode = motor_set_mode;                   // 对接电机设置参数方法
    // 电机挂载CAN总线
    switch (config->can_id)
    {
    case 1:
        ht_motor_obj[idx].can = &hcan1;
        break;
    case 2:
        ht_motor_obj[idx].can = &hcan2;
        break;
    default:
        break;
    }

    osSemaphoreDef(turn_Sem);
    ht_motor_obj[idx].turn_complete = osSemaphoreCreate(osSemaphore(turn_Sem), 1);  // 初始化信号量
    // 电机离线检测定时器相关

    ht_motor_obj[idx].ctrl_mode = CMD_RESET_MODE;
    ht_motor_obj[idx].to_mode = CMD_RESET_MODE;
    motor_set_mode(&ht_motor_obj[idx], CMD_RESET_MODE);   // 初始化为 RESET 模式

    return &ht_motor_obj[idx++];
}


/**
  * @brief  封装一帧参数控制报文的数据帧
  * @param  para: 电机控制参数
  * @param  buf:  CAN数据帧
  * @retval
  */
static void pack_contol_para(ht_motor_para_t para, uint8_t *buf)
{
    uint16_t p, v, kp, kd, t;

    /* 输入参数限幅 */
    LIMIT_MIN_MAX(para.p,  P_MIN,  P_MAX);
    LIMIT_MIN_MAX(para.v,  V_MIN,  V_MAX);
    LIMIT_MIN_MAX(para.kp, KP_MIN, KP_MAX);
    LIMIT_MIN_MAX(para.kd, KD_MIN, KD_MAX);
    LIMIT_MIN_MAX(para.t,  T_MIN,  T_MAX);

    /* 转换float参数 */
    p = float_to_uint(para.p,     P_MIN,  P_MAX,  16);
    v = float_to_uint(para.v,     V_MIN,  V_MAX,  12);
    kp = float_to_uint(para.kp,   KP_MIN, KP_MAX, 12);
    kd = float_to_uint(para.kd,   KD_MIN, KD_MAX, 12);
    t = float_to_uint(para.t,     T_MIN,  T_MAX,  12);

    /* 将参数存入CAN数据帧 */
    buf[0] = p>>8;
    buf[1] = p&0xFF;
    buf[2] = v>>4;
    buf[3] = ((v&0xF)<<4)|(kp>>8);
    buf[4] = kp&0xFF;
    buf[5] = kd>>4;
    buf[6] = ((kd&0xF)<<4)|(t>>8);
    buf[7] = t&0xff;
}

/* 预留命令接口，可用于调试 */
static void enable(void){
    static ht_motor_para_t set; // 电机控制器计算得到的控制参数m
    for (size_t i = 0; i < idx; i++)
    {
        motor_set_mode(&ht_motor_obj[i], CMD_MOTOR_MODE);
    }
}
//MSH_CMD_EXPORT(enable, enter motor_mode);

static void relax(void){
    for (size_t i = 0; i < idx; i++)
    {
        motor_set_mode(&ht_motor_obj[i], CMD_RESET_MODE);
    }
}
//MSH_CMD_EXPORT(relax, out motor_mode);

static void zero(void){
    for (size_t i = 0; i < idx; i++)
    {
        motor_set_mode(&ht_motor_obj[i], CMD_ZERO_POSITION);
    }
}
//MSH_CMD_EXPORT(zero, set motor zero);
