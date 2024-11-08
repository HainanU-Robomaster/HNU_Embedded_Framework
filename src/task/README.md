#  RM_Task 说明文档。

RM_Task 为针对 RM 常用任务的函数封装。

| 任务 | 支持情况 |
| -------- | ----------------- |
| Eample | 两个电机的PID控制示例 |



## 注意：

- 应用层调用该库时，先通过 menuconfig 使能相关外设模块选项，**仅包含 `rm_task.h` 即可**；

- 在 `rm_task.h`  文件中对线程进行统一注册管理;

- 在 robot.c 文件中实现的 `robot_init()` 负责RTOS任务的创建，各线程的初始化，线程间通讯话题的初始化和注册， `robot_init()` 在 `main.c` 中初始化调用

- task 文件基本框架

```c
#include "xxx_task.h"
#include "rm_module.h"
#include "robot.h"

/* ------------------------------- ipc 线程间通讯相关 ------------------------------ */
// 订阅
MCN_DECLARE(chassis_fdb);
static McnNode_t chassis_fdb_node;
static struct chassis_fdb_msg chassis_fdb;   // 默认订阅不加 _data 后缀
// 发布
MCN_DECLARE(chassis_cmd);
static struct chassis_cmd_msg chassis_cmd_data;  // 默认发布加 _data 后缀

static void cmd_pub_push(void);
static void cmd_sub_init(void);
static void cmd_sub_pull(void);


/* -------------------------------- xxx 线程主体 -------------------------------- */
void xxx_task_init(void)
{
    xxx_sub_init();
}

void xxx_control_task(void)
{
    xxx_sub_pull();
    
    /* 线程处理主体 */

    xxx_pub_push();
}

/* --------------------------------- 线程间通讯相关 -------------------------------- */
/**
 * @brief xxx 线程中所有发布者推送更新话题
 */
static void xxx_pub_push(void)
{
    // data_content my_data = ;
    mcn_publish(MCN_HUB(chassis_cmd), &chassis_cmd_data);
}

/**
 * @brief xxx 线程中所有订阅者初始化
 */
static void xxx_sub_init(void)
{
    chassis_fdb_node = mcn_subscribe(MCN_HUB(chassis_fdb), NULL, NULL);
}


/**
 * @brief xxx 线程中所有订阅者获取更新话题
 */
static void xxx_sub_pull(void)
{
    if (mcn_poll(chassis_fdb_node))
    {
        mcn_copy(MCN_HUB(chassis_fdb), chassis_fdb_node, &chassis_cmd_data);
    }
}

```

