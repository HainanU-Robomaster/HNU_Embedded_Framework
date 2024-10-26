/*
* Change Logs:
* Date            Author          Notes
* 2023-10-09      ChenSihan     first version
*/

#ifndef RTTHREAD_SHOOT_TASK_H
#define RTTHREAD_SHOOT_TASK_H

/**
 * @brief shoot线程入口
 */
void shoot_task_entry(void* argument);

/**
 * @brief 发射器模式
 */
typedef enum
{
    /*发射模式*/
  SHOOT_STOP=0        ,     //射击关闭
  SHOOT_ONE=1         ,     //单发模式
  SHOOT_THREE=2       ,     //三连发模式
  SHOOT_COUNTINUE=3   ,     //自动射击
  SHOOT_REVERSE=4     ,     //堵弹反转
  SHOOT_AUTO=5        ,     //自动发射模式
} shoot_mode_e;
/**
 * @brief 扳机模式
 */
typedef enum
{

    /*扳机状态*/
    TRIGGER_ON=1      ,     //扳机开火状态
    TRIGGER_OFF=0     ,     //扳机闭火状态
    TRIGGER_ING=2     ,     //扳机持续状态

} trigger_mode_e;
/**
 * @brief 发射弹弹频
 */
typedef enum
{
    LOW_FREQUENCY=1,
    MIDDLE_FREQUENCY=2,
    HIGH_FREQUENCY=3,
} shoot_frequency_e;
/**
  * @brief   发射器状态回馈
  */
//TODO:具体回馈设置待讨论
typedef enum
{
  SHOOT_OK=1,   //发射正常
  SHOOT_ERR=0,  //发射异常
  SHOOT_WAITING=2, //发射异常
} shoot_back_e;
/**
  * @brief 单发和连发角度继承
  */
typedef enum
{
    SHOOT_ANGLE_CONTINUE=0,   //角度为连发状态
    SHOOT_ANGLE_SINGLE=1,  //角度为单发状态
} shoot_angle_inherit_e;
#endif //RTTHREAD_SHOOT_TASK_H


