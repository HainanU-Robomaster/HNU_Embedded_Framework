 /**
 * @file rm_task.h
 * @brief  注意该文件应只用于任务初始化,只能被robot.c包含
 * @date 2023-12-28
 */

 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-12-28      ChuShicheng     first version
 */
#ifndef _RM_TASK_H
#define _RM_TASK_H

#include "rm_config.h"
#include "rm_module.h"
#include "cmsis_os.h"

#include "ins_task.h"
#include "motor_task.h"
#include "cmd_task.h"
#include "chassis_task.h"
#include "trans_task.h"
#include "referee_task.h"

/* ---------------------------------- 线程相关 ---------------------------------- */
osThreadId insTaskHandle;
osThreadId robotTaskHandle;
osThreadId motorTaskHandle;
osThreadId transTaskHandle;
osThreadId refereeTaskHandle;

void ins_task_entry(void const *argument);
void motor_task_entry(void const *argument);
void robot_task_entry(void const *argument);
void trans_task_entry(void const *argument);
void referee_task_entry(void const *argument);

static float motor_dt;
static float robot_dt;
static float trans_dt;
static float referee_dt;

/**
 * @brief 初始化机器人任务,所有持续运行的任务都在这里初始化
 *
 */
void OS_task_init()
{
    osThreadDef(instask, ins_task_entry, osPriorityAboveNormal, 0, 1024);
    insTaskHandle = osThreadCreate(osThread(instask), NULL); // 为姿态解算设置较高优先级,确保以1khz的频率执行

    osThreadDef(motortask, motor_task_entry, osPriorityNormal, 0, 512);
    motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

    osThreadDef(robottask, robot_task_entry, osPriorityNormal, 0, 2048);
    robotTaskHandle = osThreadCreate(osThread(robottask), NULL);

    osThreadDef(transtask, trans_task_entry, osPriorityNormal, 0, 1024);
    transTaskHandle = osThreadCreate(osThread(transtask), NULL);

    osThreadDef(refereetask, referee_task_entry, osPriorityNormal, 0, 1024);
    refereeTaskHandle = osThreadCreate(osThread(refereetask), NULL);
}

__attribute__((noreturn)) void motor_task_entry(void const *argument)
{
    float motor_start = dwt_get_time_ms();
    PrintLog("[freeRTOS] Motor Task Start\n");
    uint32_t motor_wake_time = osKernelSysTick();
    for (;;)
    {
/* ------------------------------ 调试监测线程调度 ------------------------------ */
        motor_dt = dwt_get_time_ms() - motor_start;
        motor_start = dwt_get_time_ms();
        if (motor_dt > 1.5)
            PrintLog("[freeRTOS] Motor Task is being DELAY! dt = [%f]\n", &motor_dt);
/* ------------------------------ 调试监测线程调度 ------------------------------ */

        motor_control_task();

        vTaskDelayUntil(&motor_wake_time, 1);
    }
}

__attribute__((noreturn)) void robot_task_entry(void const *argument)
{
    float robot_start = dwt_get_time_ms();
    PrintLog("[freeRTOS] Robot Task Start\n");
    uint32_t robot_wake_time = osKernelSysTick();
    for (;;)
    {
/* ------------------------------ 调试监测线程调度 ------------------------------ */
        robot_dt = dwt_get_time_ms() - robot_start;
        robot_start = dwt_get_time_ms();
        if (robot_dt > 5.5)
            PrintLog("[freeRTOS] Robot Task is being DELAY! dt = [%f]\n", &robot_dt);
/* ------------------------------ 调试监测线程调度 ------------------------------ */

        cmd_control_task();
        chassis_control_task();

        vTaskDelayUntil(&robot_wake_time, 3);  // 平衡步兵需要1khz
    }
}

 __attribute__((noreturn)) void trans_task_entry(void const *argument)
{
    float trans_start = dwt_get_time_ms();
    PrintLog("[freeRTOS] Trans Task Start\n");
    uint32_t trans_wake_time = osKernelSysTick();
    for (;;)
    {
/* ------------------------------ 调试监测线程调度 ------------------------------ */
        trans_dt = dwt_get_time_ms() - trans_start;
        trans_start = dwt_get_time_ms();
        if (trans_dt > 1.5)
             PrintLog("[freeRTOS] Trans Task is being DELAY! dt = [%f]\n", &trans_dt);
/* ------------------------------ 调试监测线程调度 ------------------------------ */

        trans_control_task();

        vTaskDelayUntil(&trans_wake_time, 1);
    }
}

__attribute__((noreturn)) void referee_task_entry(void const *argument)
{
    /* USER CODE BEGIN RefereeTask */
    static float referee_start;
    static uint32_t referee_dwt = 0;
    static float dt = 0;
    static uint32_t count = 0;

    // referee_UI_task_init();

    dt = dwt_get_delta(&referee_dwt);
    referee_start = dwt_get_time_ms();
    
    uint32_t referee_wake_time = osKernelSysTick();
    PrintLog("[freeRTOS] Ins Task Start\n");
    /* Infinite loop */
    for(;;)
    {
/* ------------------------------ 调试监测线程调度 ------------------------------ */
        referee_dt = dwt_get_time_ms() - referee_start;
        referee_start = dwt_get_time_ms();
/* ------------------------------ 调试监测线程调度 ------------------------------ */

        dt = dwt_get_delta(&referee_dwt);

        referee_control_task();

        vTaskDelayUntil(&referee_wake_time, 10); // 100hz
    }
}

#endif /* _RM_TASK_H */
