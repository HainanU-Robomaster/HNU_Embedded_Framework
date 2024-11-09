 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-10-25      ChuShicheng     first version
 */
#include "lk_motor.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"

#define LK_MOTOR_CNT 4
/* 滤波系数设置为1的时候即关闭滤波 */
#define CURRENT_SMOOTH_COEF 0.9f
#define SPEED_SMOOTH_COEF 1.0f
#define ECD_ANGLE_COEF_LK (360.0f / 65536.0f)  // 使用电机编码器为 16 bit
#define CURRENT_TORQUE_COEF_LK 0.003645f  // 电流设定值转换成扭矩的系数,算出来的设定值除以这个系数就是扭矩值

#define I_MIN -2000
#define I_MAX 2000

static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用
/* 瓴控电机的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行 */
static lk_motor_object_t *lk_motor_obj[LK_MOTOR_CNT] = {NULL};

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

// TODO: 目前仅对接多电机控制模式，后续根据需求可以适配更多模式
// 根据电机CAN协议手册，多电机模式下，电机的ID为 1~4 ，反馈报文标识符为 0x140+ID(1~4)

#ifdef LK_MOTOR_TEST
static float decode_dt;
static float speed_acc;
static float speed_last;
extern KalmanFilter_t Acc_KF;;
static float lk_dv;
#endif
/**
 * @brief 根据返回的can_object对反馈报文进行解析
 *
 * @param object 收到数据的object,通过遍历与所有电机进行对比以选择正确的实例
 */
static void motor_decode(lk_motor_object_t *motor, uint8_t *data)
{
    uint8_t *rx_buff = data;
    lk_motor_measure_t *measure = &motor->measure; // measure要多次使用,保存指针减小访存开销

#ifdef LK_MOTOR_TEST
    static float decode_start;
    speed_last = measure->speed_rads;
#endif

//    rt_timer_start(motor->timer);  // 判定电机未离线，重置电机定时器

    // 解析数据并对电流和速度进行滤波,电机的反馈报文具体格式见电机说明手册
    measure->temperature = rx_buff[1];

    measure->real_current = (1 - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rx_buff[3] << 8 | rx_buff[2]));

    measure->speed_rads = (1 - SPEED_SMOOTH_COEF) * measure->speed_rads +
                          DEGREE_2_RAD * SPEED_SMOOTH_COEF * (float)((int16_t)(rx_buff[5] << 8 | rx_buff[4]));
#ifdef LK_MOTOR_TEST
    Acc_KF.MeasuredVector[0] = measure->speed_rads;
//    measure->speed_rads = *Kalman_Filter_Update(&Acc_KF);
#endif
    measure->last_ecd = measure->ecd;
    measure->ecd = (uint16_t)((rx_buff[7] << 8) | rx_buff[6]);

    measure->angle_single_round = ECD_ANGLE_COEF_LK * measure->ecd * DEGREE_2_RAD;

    if (measure->ecd - measure->last_ecd > 32768)
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -32768)
        measure->total_round++;
    measure->total_angle = (measure->total_round * 2 * PI + measure->angle_single_round)/**WHEEL_RADIUS*/;

#ifdef LK_MOTOR_TEST
    decode_dt = dwt_get_time_ms() - decode_start;
    lk_dv =  measure->speed_rads - speed_last / decode_dt;
//     Acc_KF.MeasuredVector[0] = lk_dv;
    speed_acc = (measure->speed_rads - speed_last) / (decode_dt / 1000.0f);
    decode_start = dwt_get_time_ms();
#endif
}

// lk_chassis_motor[0].measure->speed_rads - speed_last / decode_dt

//TODO: 添加具体的电机超时处理，如报警或停止等，以及影响机器人状态机等
/**
 * @brief 电机定时器超时回调函数
 * @param motor_ptr
 */
static void motor_lost_callback(void *motor_ptr)
{
    lk_motor_object_t *motor = (lk_motor_object_t *)motor_ptr;
//    lk_motor_relax(motor);
}

/**
 * @brief 电机反馈报文接收回调函数,该函数被can_rx_call调用
 *
 * @param dev 接收到报文的CAN设备
 * @param id 接收到的报文的id
 * @param data 接收到的报文的数据
 */
int lk_motot_rx_callback(uint32_t id, uint8_t *data){
    // 找到对应的实例后再调用motor_decode进行解析
    for (size_t i = 0; i < idx; ++i)
    {   /* 详见瓴控M系列电机手册反馈报文 */
        if (lk_motor_obj[i]->rx_id == id)
        {
            motor_decode(lk_motor_obj[i], data);
            return 0;
        }
    }

    return -1; // 未找到对应的电机实例
}

void lk_motor_relax(lk_motor_object_t *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void lk_motor_enable(lk_motor_object_t *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

// 运算所有电机实例的控制器,发送控制报文
void lk_motor_control()
{
    lk_motor_object_t *motor;
    lk_motor_measure_t measure;
    int16_t set; // 电机控制器计算得到的控制参数
    uint8_t id;  // 1~4 用于装填多电机CAN控制报文
    uint8_t data_buf[8];  // 用于多电机模式下合并一帧CAN报文

    // 遍历所有电机实例,运行控制算法并填入报文
    for (size_t i = 0; i < idx; ++i)
    {
        motor = lk_motor_obj[i];
        id = motor->rx_id - 0x141;     // 对应多电机模式下的ID转换规则
        measure = motor->measure;
        set = motor->control(measure); // 调用对接的电机控制器计算
        LIMIT_MIN_MAX(set,  I_MIN,  I_MAX);

        // 合并报文
        if (motor->stop_flag == MOTOR_STOP)
        {
            data_buf[id * 2] = 0;
            data_buf[id * 2 + 1] = 0;
        }
        else
        {
            data_buf[id * 2] = (uint8_t) (set & 0x00ff);
            data_buf[id * 2 + 1] = (uint8_t) (set >> 8);
        }


        /*// 合并报文
        data_buf[0] = 0xA1;
        // 若该电机处于停止状态,直接将buff置零
        if (motor->stop_flag == MOTOR_STOP)
        {
            data_buf[4] = 0;
            data_buf[5] = 0;
        }
        else
        {
            data_buf[4] = (uint8_t)(set & 0x00ff);
            data_buf[5] = (uint8_t)(set >> 8);
        }*/
        // 发送报文
        if(i == idx - 1)
            CAN_send(motor->can, 0x280, data_buf);
    }
}

/**
 * @brief 电机初始化,返回一个电机实例
 * @param config 电机配置
 * @return lk_motor_object_t* 电机实例指针
 */
lk_motor_object_t *lk_motor_register(motor_config_t *config, void *control)
{
    lk_motor_object_t *object = (lk_motor_object_t *)user_malloc(sizeof(lk_motor_object_t));
    memset(object, 0, sizeof(lk_motor_object_t));

    // 对接用户配置的 motor_config
    object->motor_type = config->motor_type;             // 瓴控M系列电机
    object->rx_id = config->rx_id;                       // 接收报文的ID(主收)
    object->tx_id = config->tx_id;                       // 发送报文的ID(主发) 多电机控制模式下应为 0x280
    object->control = control;                           // 电机控制器执行
    // 电机挂载CAN总线
    switch (config->can_id)
    {
    case 1:
        object->can = &hcan1;
        break;
    case 2:
        object->can = &hcan2;
        break;
    default:
        break;
    }
    // 电机离线检测定时器相关

    lk_motor_enable(object);
    lk_motor_obj[idx++] = object;
    return object;
}
