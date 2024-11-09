#include "motor_task.h"
#include "rm_module.h"
#include "robot.h"
static osMutexId semMotorHandle; // 触发CAN消息发送的信号量

void motor_task_init(void)
{
    osMutexDef(motor_Sem);
    semMotorHandle = osMutexCreate(osMutex(motor_Sem));  // 初始化信号量
}

//float lk_dt, lk_start;

void motor_control_task(void)
{
//    static osEvent evt;
    // static uint8_t cnt = 0; 设定不同电机的任务频率
    // if(cnt%5==0) //200hz
    // if(cnt%10==0) //100hz
//    dji_motor_control();
//    osSemaphoreWait(semMotorHandle, osWaitForever); // 等待信号量可用
/*    for (uint8_t i=0; i < 5; ++i)
    {
        evt = osSignalWait (0x01, osWaitForever);
        if (evt.status == osEventSignal)  {
            if(i==4){
                lk_motor_control();
            }
            else {
                ht_controller(i);
            }
        }*/
    lk_motor_control();
}

static float can_tim_dt, can_tim_start;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    static uint8_t i = 0;
    if (htim->Instance == htim3.Instance)
    {
        can_tim_dt = dwt_get_time_us() - can_tim_start;
        can_tim_start = dwt_get_time_us();
        ht_controll_all_poll();
    }
}
