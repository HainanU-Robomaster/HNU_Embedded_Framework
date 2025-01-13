/*
* Change Logs:
* Date            Author          Notes
* 2023-09-25      ChuShicheng     first version
*/
#ifndef _CHASSIS_TASK_H
#define _CHASSIS_TASK_H

#include <rtthread.h>

/**
 * @brief 底盘模式
 */
typedef enum
{
    CHASSIS_INIT,           //底盘初始化
    CHASSIS_RELAX,         //底盘失能
    CHASSIS_STOP,          //底盘停止
    CHASSIS_OPEN_LOOP,     //底盘开环
} chassis_mode_e;

typedef enum
{
    BACK_STEP = 0,             //正在回中
    BACK_IS_OK = 1,            //回中完毕
} chassis_back_e;
void chassis_thread_entry(void *argument);

#endif /* _CHASSIS_TASK_H */
