 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-08-28      ChuShicheng     first version
 */
#include "robot.h"
#include "rm_config.h"
#include "rm_module.h"
#include "rm_task.h"

/* 同一个主题名称不可被重复定义 */
MCN_DEFINE(chassis_cmd, sizeof(struct chassis_cmd_msg));
MCN_DEFINE(chassis_fdb, sizeof(struct chassis_fdb_msg));
MCN_DEFINE(ins_topic, sizeof(struct ins_msg));
MCN_DEFINE(trans_fdb,sizeof(struct trans_fdb_msg));

static void mcn_topic_init(void);

void robot_init()
{  
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用 dwt 进行延时
    __disable_irq();

    // Ins_task_init;

    OS_task_init(); // 创建基础任务

    mcn_topic_init(); // 话题注册初始化

    motor_task_init();
    cmd_task_init();
    chassis_task_init();
    trans_task_init();
    referee_UI_task_init();

    // 初始化完成,开启中断
    __enable_irq();
}

/**
 * @brief ipc uMCN 各话题注册
 * 
 */
static void mcn_topic_init(void)
{
    mcn_advertise(MCN_HUB(ins_topic), NULL);
    mcn_advertise(MCN_HUB(chassis_cmd), NULL);
    mcn_advertise(MCN_HUB(chassis_fdb), NULL);
    mcn_advertise(MCN_HUB(trans_fdb), NULL);
}
