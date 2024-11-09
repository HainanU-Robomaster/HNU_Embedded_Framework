#include <string.h>
#include "trans_task.h"
#include "rm_module.h"
#include "robot.h"
#include "usbd_cdc_if.h"
/* ------------------------------- ipc 线程间通讯相关 ------------------------------ */
// 订阅
MCN_DECLARE(ins_topic);
static McnNode_t ins_topic_node;
static struct ins_msg ins;
// 发布
MCN_DECLARE(trans_fdb);
static struct trans_fdb_msg trans_fdb_data;
static void trans_pub_push(void);
static void trans_sub_init(void);
static void trans_sub_pull(void);
/*------------------------------传输数据相关 --------------------------------- */
int recv_flag=0;
FrameTypeDef upper_rx_data;       //接收上位机数据中转帧
RpyTypeDef rpy_rx_data;  //接收欧拉角方式控制数据帧
RpyTypeDef rpy_tx_data={
        .HEAD = 0XFF,
        .D_ADDR = MAINFLOD,
        .ID = GIMBAL,
        .LEN = FRAME_RPY_LEN,
        .DATA={0},
        .SC = 0,
        .AC = 0,
};
/* ------------------------------- 发送数据指令 ------------------------------ */
static void send_data(RpyTypeDef data_r);
static void pack_rpy(RpyTypeDef *frame, float yaw, float pitch,float roll);
static void check_rpy(RpyTypeDef *frame);
/* ------------------------------- 接受数据指令 ------------------------------ */
static void get_data(void);
/* -------------------------------- trans线程主体 -------------------------------- */
void trans_task_init(void)
{
    trans_sub_init();
}

void trans_control_task(void)
{
    trans_sub_pull();
    send_data(rpy_tx_data);
    get_data();
    trans_pub_push();
}

/**
 * @brief 向上位机发送数据
 */
static void send_data(RpyTypeDef data_r)
{
/*填充数据*/
    pack_rpy(&data_r, ins.yaw, ins.pitch, ins.roll);
    //pack_rpy(&data_r, 1, 1, 1);
    check_rpy(&data_r);
    CDC_Transmit_FS((uint8_t*)&data_r, sizeof(data_r));

}

static void pack_rpy(RpyTypeDef *frame, float yaw, float pitch,float roll)
{
    int8_t rpy_tx_buffer[FRAME_RPY_LEN] = {0} ;
    int32_t rpy_data = 0;
    uint32_t *gimbal_rpy = (uint32_t *)&rpy_data;

    rpy_tx_buffer[0] = 0;
    rpy_data = yaw * 1000;
    rpy_tx_buffer[1] = *gimbal_rpy;
    rpy_tx_buffer[2] = *gimbal_rpy >> 8;
    rpy_tx_buffer[3] = *gimbal_rpy >> 16;
    rpy_tx_buffer[4] = *gimbal_rpy >> 24;
    rpy_data = pitch * 1000;
    rpy_tx_buffer[5] = *gimbal_rpy;
    rpy_tx_buffer[6] = *gimbal_rpy >> 8;
    rpy_tx_buffer[7] = *gimbal_rpy >> 16;
    rpy_tx_buffer[8] = *gimbal_rpy >> 24;
    rpy_data = roll *1000;
    rpy_tx_buffer[9] = *gimbal_rpy;
    rpy_tx_buffer[10] = *gimbal_rpy >> 8;
    rpy_tx_buffer[11] = *gimbal_rpy >> 16;
    rpy_tx_buffer[12] = *gimbal_rpy >> 24;

    memcpy(&frame->DATA[0], rpy_tx_buffer,13);

    frame->LEN = FRAME_RPY_LEN;
}

static void check_rpy(RpyTypeDef *frame)
{
    uint8_t sum = 0;
    uint8_t add = 0;

    sum += frame->HEAD;
    sum += frame->D_ADDR;
    sum += frame->ID;
    sum += frame->LEN;
    add += sum;

    for (int i = 0; i < frame->LEN; i++)
    {
        sum += frame->DATA[i];
        add += sum;
    }

    frame->SC = sum & 0xFF;
    frame->AC = add & 0xFF;
}

static void get_data(void)
{
    if(recv_flag) {
        if (!rpy_rx_data.DATA[0]){//绝对角度控制
            trans_fdb_data.yaw = *(int32_t*)&rpy_rx_data.DATA[1] / 1000.0;
            trans_fdb_data.pitch = *(int32_t*)&rpy_rx_data.DATA[5] / 1000.0;
        }
        else {     //相对角度控制
            trans_fdb_data.yaw = (*(int32_t *) &rpy_rx_data.DATA[1] / 1000.0);
            trans_fdb_data.pitch= (*(int32_t *) &rpy_rx_data.DATA[5] / 1000.0);
            //send_data(rpy_tx_data);
        }
        recv_flag = 0;
    }
}

/* --------------------------------- 线程间通讯相关 -------------------------------- */
/**
 * @brief cmd 线程中所有发布者推送更新话题
 */
static void trans_pub_push(void)
{
    mcn_publish(MCN_HUB(trans_fdb), &trans_fdb_data);
}

/**
 * @brief cmd 线程中所有订阅者初始化
 */
static void trans_sub_init(void)
{
    ins_topic_node = mcn_subscribe(MCN_HUB(ins_topic), NULL, NULL);
}

/**
 * @brief cmd 线程中所有订阅者获取更新话题
 */
static void trans_sub_pull(void)
{
    if (mcn_poll(ins_topic_node))
    {
        mcn_copy(MCN_HUB(ins_topic), ins_topic_node, &ins);
    }
}