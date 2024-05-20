/*
* Change Logs:
* Date            Author          Notes
* 2024-01-9      ChenSihan     first version
*/
#ifndef REFEREE_SYSTEM_H
#define REFEREE_SYSTEM_H

#pragma once
#define HEADER_SOF 0xA5
#define Agreement_RX_BUF_NUM 512        //DMA要传输的数据项数目NDTR寄存器填充值 200，即收200字节后自动填充并转换缓冲数组

#define FIFO_BUF_LENGTH     1024
#ifndef SETINGS_REFEREE_SYSTEM_H
#define SETINGS_REFEREE_SYSTEM_H
#define REF_PROTOCOL_FRAME_MAX_SIZE         128
#define REF_PROTOCOL_HEADER_SIZE            sizeof(Frame_header_Typedef)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))
#include "drv_common.h"


typedef enum
{
    GAME_STATUS_CMD_ID                = 0x0001,
    GAME_RESULT_CMD_ID                = 0x0002,
    GAME_ROBOT_HP_CMD_ID              = 0x0003,
    FIELD_EVENTS_CMD_ID               = 0x0101,
    SUPPLY_PROJECTILE_ACTION_CMD_ID   = 0x0102,
    SUPPLY_PROJECTILE_BOOKING_CMD_ID  = 0x0103,
    REFEREE_WARNING_CMD_ID            = 0x0104,
    DART_FIRE_CMD_ID                  = 0x0105,
    ROBOT_STATUS_CMD_ID               = 0x0201,
    POWER_HEAT_DATA_CMD_ID            = 0x0202,
    ROBOT_POS_CMD_ID                  = 0x0203,
    BUFF_MUSK_CMD_ID                  = 0x0204,
    AERIAL_ROBOT_ENERGY_CMD_ID        = 0x0205,
    ROBOT_HURT_CMD_ID                 = 0x0206,
    SHOOT_DATA_CMD_ID                 = 0x0207,
    BULLET_REMAINING_CMD_ID           = 0x0208,
    ROBOT_RFID_CMD_ID                 = 0x0209,
    DART_DIRECTIONS_CMD_ID            = 0x020A,
    ROBOT_LOCATION_CMD_ID             = 0x020B,
    RADAR_PROGRESS_CMD_ID             = 0x020C,
    SENTRY_AUTONOMY__CMD_ID           = 0x020D,
    RADAR_AUTONOMY_CMD_ID             = 0x020E,
    STUDENT_INTERACTIVE_DATA_CMD_ID   = 0x0301,
    CUSTOMER_CONTROLLER_ROBOT_CMD_ID  = 0x0302,
    PLAYER_MINIMAP_CMD_ID             = 0x0303,
    KEYBOARD_MOUSE_CMD_ID             = 0x0304,
    RADAR_MINIMAP_CMD_ID              = 0x0305,
    CUSTOMER_CONTROLLER_PLAYER_CMD_ID = 0x0306,
    PLAYER_MINIMAP_SENTRY_CMD_ID      = 0x0307,
    PLAYER_MINIMAP_ROBOT_CMD_ID       = 0x0308,
    IDCustomData,
}referee_cmd_id_t;


/**
*   接收协议数据的帧头数据结构体
*/
typedef  struct
{
    uint8_t SOF;                           /*! 数据帧起始字节，固定值为 0xA5 */
    uint16_t data_length;                  /*! 数据帧中 data 的长度 */
    uint8_t seq;                           /*! 包序号 */
    uint8_t CRC8;                          /*! 帧头 CRC8校验 */
} __attribute__((__packed__)) Frame_header_Typedef;

/**
*   接收的协议数据
*/
typedef struct
{
    Frame_header_Typedef* frame_header;        /*! 帧头数据结构体 */
    uint16_t cmd_id;                          /*! 命令码 ID */
    uint8_t* data;                            /*! 接收的数据指针 */
    uint16_t frame_tail;                      /*! frame_tail */
} __attribute__((__packed__)) RX_AgreementData;


typedef enum
{
    STEP_HEADER_SOF  = 0,
    STEP_LENGTH_LOW  = 1,
    STEP_LENGTH_HIGH = 2,
    STEP_FRAME_SEQ   = 3,
    STEP_HEADER_CRC8 = 4,
    STEP_DATA_CRC16  = 5,
} unpack_step_e;

/**
*   在单字节解包时用来码放从fifo中取出来的字节的容器，将fifo中取出来的字节拾掇成数据帧的结构
*/
typedef struct
{
    Frame_header_Typedef *p_header;
    uint16_t       data_len;
    uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
    unpack_step_e  unpack_step;
    uint16_t       index;
} unpack_data_t;
/**
*   比赛状态数据         对应的命令码ID为：0x0001
*/
typedef struct
{
    uint8_t game_type : 4;                      /*! 比赛的类型 */
    uint8_t game_progress : 4;                  /*! 当前比赛阶段 */
    uint16_t stage_remain_time;                 /*! 当前阶段剩余时间 */
    uint64_t SyncTimeStamp;                     /*! 机器人接收到该指令的精确 Unix 时间 */
} __attribute__((__packed__)) game_status_t;

/**
*   比赛结果数据         对应的命令码ID为：0x0002
*/
typedef struct
{
    uint8_t winner;                             /*! 0 平局 1 红方胜利 2 蓝方胜利 */
} __attribute__((__packed__)) game_result_t;




/**
*   机器人血量数据         对应的命令码ID为：0x0003
*/
typedef struct
{
    uint16_t red_1_robot_HP;                    /*! 红 1 英雄机器人血量，未上场以及罚下血量为 0 */
    uint16_t red_2_robot_HP;                    /*! 红 2 工程机器人血量 */
    uint16_t red_3_robot_HP;                    /*! 红 3 步兵机器人血量 */
    uint16_t red_4_robot_HP;                    /*! 红 4 步兵机器人血量 */
    uint16_t red_5_robot_HP;                    /*! 红 5 步兵机器人血量 */
    uint16_t red_7_robot_HP;                    /*! 红 7 哨兵机器人血量 */
    uint16_t red_outpost_HP;                    /*! 红方前哨站血量 */
    uint16_t red_base_HP;                       /*! 红方基地血量 */
    uint16_t blue_1_robot_HP;                   /*! 蓝 1 英雄机器人血量 */
    uint16_t blue_2_robot_HP;                   /*! 蓝 2 工程机器人血量 */
    uint16_t blue_3_robot_HP;                   /*! 蓝 3 步兵机器人血量 */
    uint16_t blue_4_robot_HP;                   /*! 蓝 4 步兵机器人血量 */
    uint16_t blue_5_robot_HP;                   /*! 蓝 5 步兵机器人血量 */
    uint16_t blue_7_robot_HP;                   /*! 蓝 7 哨兵机器人血量 */
    uint16_t blue_outpost_HP;                   /*! 蓝方前哨站血量 */
    uint16_t blue_base_HP;                      /*! 蓝方基地血量 */
} __attribute__((__packed__)) game_robot_HP_t;


/**
 *   人工智能挑战赛加成/惩罚区分布与潜伏模式状态        对应的命令码ID为：0x0005
 *   发送频率：1Hz 周期发送，发送范围：所有机器人
 *   激活状态： 0:未激活  1:可激活
 *   状态信息：1 为红方回血区；
 *           2 为红方弹药补给区；
 *           3 为蓝方回血区；
 *           4 为蓝方弹药补给区；
 *           5 为禁止射击区；
 *           6 为禁止移动区
 *   潜伏模式阶段：0：正常阶段；
 *           1：准备进入潜伏阶段；
 *           2：潜伏阶段
*/
typedef struct
{
    uint8_t F1_zone_status:1;                               /*! F1 激活状态 */
    uint8_t F1_zone_buff_debuff_status:3;                   /*! F1 状态信息 */
    uint8_t F2_zone_status:1;                               /*! F2 激活状态 */
    uint8_t F2_zone_buff_debuff_status:3;                   /*! F1 状态信息 */
    uint8_t F3_zone_status:1;                               /*! F3 激活状态 */
    uint8_t F3_zone_buff_debuff_status:3;                   /*! F1 状态信息 */
    uint8_t F4_zone_status:1;                               /*! F4 激活状态 */
    uint8_t F4_zone_buff_debuff_status:3;                   /*! F1 状态信息 */
    uint8_t F5_zone_status:1;                               /*! F5 激活状态 */
    uint8_t F5_zone_buff_debuff_status:3;                   /*! F1 状态信息 */
    uint8_t F6_zone_status:1;                               /*! F6 激活状态 */
    uint8_t F6_zone_buff_debuff_status:3;                   /*! F1 状态信息*/
    uint16_t red1_bullet_left;                              /*! 红方 1 号剩余弹量 */
    uint16_t red2_bullet_left;                              /*! 红方 2 号剩余弹量 */
    uint16_t blue1_bullet_left;                             /*! 蓝方 1 号剩余弹量 */
    uint16_t blue2_bullet_left;                             /*! 蓝方 2 号剩余弹量 */
    uint8_t lurk_mode;                                      /*! 潜伏模式阶段 */
    uint8_t res;                                            /*! 保留字节 */
} __attribute__((__packed__)) ext_ICRA_buff_debuff_zone_and_lurk_status_t;


/**
 *   场地事件数据。        对应的命令码ID为：：0x0101
 *   发送频率：1Hz
 *   bit 0-2：
 *   bit 0：己方补给站前补血点的占领状态，1 为已占领
 *   bit 1：己方补给站内部补血点的占领状态，1 为已占领
 *   bit 2：己方补给区的占领状态，1 为已占领（仅 RMUL 适用）
     bit 3-5：己方能量机关状态：
         bit 3：己方能量机关激活点的占领状态，1 为已占领
         bit 4：己方小能量机关的激活状态，1 为已激活
         bit 5：己方大能量机关的激活状态，1 为已激活
     bit 6-11：己方高地占领状态
         bit 6-7：己方环形高地的占领状态，1 为被己方占领，2 为被对方占领
         bit 8-9：己方梯形高地的占领状态，1 为被己方占领，2 为被对方占领
         bit 10-11：己方梯形高地的占领状态，1 为被己方占领，2 为被对方占领
     bit 12-18：己方基地虚拟护盾的剩余值百分比（四舍五入，保留整数）
     bit 19-27：飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为0）
     bit 28-29：飞镖最后一次击中己方前哨站或基地的具体目标，开局默认为 0，1 为击中前哨站，2 为击中基地固定目标，3 为击中基地随机目标
     bit 30-31：中心增益点的占领情况，0 为未被占领，1 为被己方占领，2 为被对方占领，3 为被双方占领。（仅 RMUL 适用）
*/
typedef struct
{
    uint32_t event_type;
} __attribute__((__packed__)) event_data_t;



/**
 *   补给站动作标识         对应的命令码ID为：0x0102
 *   发送频率：动作改变后发送, 发送范围：己方机器人
 *   保留
 *   补弹机器人 ID：
 *          0 为当前无机器人补弹，
 *          1 为红方英雄机器人补弹
 *          3/4/5 为红方步兵机器人补弹
 *          101 为蓝方英雄机器人补弹
 *          103/104/105 为蓝方步兵机器人补弹
 *    出弹口开闭状态：
 *          0 关闭
 *          1 弹丸准备中
 *          2 弹丸释放
 *
 *    补弹数量：
 *          50：50 颗子弹；
 *          100：100 颗子弹；
 *          150：150 颗子弹；
 *          200：200 颗子弹
*/
typedef struct
{
    uint8_t reserved;                           /*! 保留*/
    uint8_t supply_robot_id;                    /*! 补弹机器人 ID */
    uint8_t supply_projectile_step;             /*! 出弹口开闭状态 */
    uint8_t supply_projectile_num;              /*! 补弹数量 */
} __attribute__((__packed__))  ext_supply_projectile_action_t;



/**
 *   裁判警告信息         对应的命令码ID为：0x0104
 *   发送频率：己方警告发生后发送
 *   己方最后一次受到判罚的等级：
 *          1：双方黄牌
 *          2：黄牌
 *          3：红牌
 *          4：判负
 *    犯规机器人 ID：
 *          己方最后一次受到判罚的违规机器人 ID。（如红 1 机器人 ID 为 1，蓝1 机器人 ID 为 101）
 *          判负和双方黄牌时，该值为 0
 *    违规次数：
 *          己方最后一次受到判罚的违规机器人对应判罚等级的违规次数。（开局默认为 0。）
*/
typedef struct
{
    uint8_t level;                              /*! 警告等级 */
    uint8_t offending_robot_id;                      /*! 犯规机器人 ID */
    uint8_t count;                                /*! 违规次数 */
} __attribute__((__packed__)) referee_warning_t;

/**
 *   飞镖发射口倒计时         对应的命令码ID为：0x0105
 *   发送频率：1Hz 周期发送，发送范围：己方机器人
 *   剩余时间：
 *         己方飞镖发射剩余时间，单位：秒
 *   飞镖目标：
 *   bit 0-1：
 *      最近一次己方飞镖击中的目标，开局默认为 0，1 为击中前哨站，2 为击中
 *      基地固定目标，3 为击中基地随机目标
 *   bit 2-4：
 *      对方最近被击中的目标累计被击中计数，开局默认为 0，至多为 4
 *   bit 5-6：
 *     飞镖此时选定的击打目标，开局默认或未选定/选定前哨站时为 0，选中基  地固定目标为 1，选中基地随机目标为 2
 *   bit 7-15：保留
*/
typedef struct
{
    uint8_t dart_remaining_time;            /*! 15s 倒计时 */
    uint16_t dart_info;
} __attribute__((__packed__)) dart_info_t;


/**
 *   比赛机器人状态         对应的命令码ID为：0x0201
 *   发送频率：10Hz
 *    本机器人 ID：
 *            1：红方英雄机器人；
 *            2：红方工程机器人；
 *            3/4/5：红方步兵机器人；
 *            6：红方空中机器人；
 *            7：红方哨兵机器人；
 *            8：红方飞镖机器人；
 *            9：红方雷达站；
 *            101：蓝方英雄机器人；
 *            102：蓝方工程机器人；
 *            103/104/105：蓝方步兵机器人；
 *            106：蓝方空中机器人；
 *            107：蓝方哨兵机器人；
 *            108：蓝方飞镖机器人；
 *            109：蓝方雷达站。
 *   主控电源输出情况：
 *            gimbal 口输出： 1 为有 24V 输出，0 为无 24v 输出；
 *            chassis 口输出：1 为有 24V 输出，0 为无 24v 输出；
 *            shooter 口输出：1 为有 24V 输出，0 为无 24v 输出；
*/
typedef struct
{
    uint8_t robot_id;                               /*! 本机器人 ID */
    uint8_t robot_level;                            /*! 机器人等级 */
    uint16_t current_HP;                             /*! 机器人剩余血量 */
    uint16_t maximum_HP;                                /*! 机器人上限血量 */
    uint16_t shooter_barrel_cooling_value;             /*!机器人枪口热量每秒冷却值*/
    uint16_t shooter_barrel_heat_limit;                /*!机器人枪口热量上限*/
    uint16_t chassis_power_limit;                   /*! 机器人底盘功率限制上限 */
    uint8_t mains_power_gimbal_output : 1;          /*! 主控电源输出情况：gimbal 口输出 */
    uint8_t mains_power_chassis_output : 1;         /*! 主控电源输出情况：chassis 口输出 */
    uint8_t mains_power_shooter_output : 1;         /*! 主控电源输出情况：shooter 口输出 */
} __attribute__((__packed__)) robot_status_t;
#endif //SETINGS_REFEREE_SYSTEM_H


/**
 *   实时功率热量数据   对应命令码ID为：0x0202
 *   发送频率：50Hz
 *   底盘输出电压 单位 毫伏
 *   底盘输出电流 单位 毫安
 *   底盘输出功率 单位 W 瓦
 *   底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
 *   1 号 17mm 枪口热量
 *   2 号 17mm 枪口热量
 *   42mm 枪口热量
 */
typedef  struct
{
    uint16_t chassis_voltage;                          /*!底盘输出电压*/
    uint16_t chassis_current;                          /*!底盘输出电流*/
    float chassis_power;                               /*!底盘输出功率*/
    uint16_t buffer_energy;                            /*!底盘功率缓存*/
    uint16_t shooter_17mm_1_barrel_heat;               /*!机器人1 号 17mm 枪口热量*/
    uint16_t shooter_17mm_2_barrel_heat;               /*!机器人2 号 17mm 枪口热量*/
    uint16_t shooter_42mm_barrel_heat;                 /*!机器人42mm 枪口热量*/
} __attribute__((__packed__)) power_heat_data_t;


/**
 * 机器人位置    对应命令码ID为：0x0203
 * 发送频率：10Hz
 * 位置 x 坐标，单位 m
 * 位置 y 坐标，单位 m
 * 位置枪口，单位度
 */
typedef  struct
{
    float x;                               /*!位置 x 坐标*/
    float y;                               /*!位置 y 坐标*/
    float angle;                           /*!本机器人测速模块的朝向，单位：度。正北为 0 度*/
} __attribute__((__packed__)) robot_pos_t;


/**
* 机器人增益  对应命令码ID为：0x0204
* 发送频率：1Hz
* 机器人血量补血状态
* 枪口热量冷却加速
* 机器人防御加成
* 机器人攻击加成
* 其他 bit 保留
*/
typedef  struct
{
    uint8_t recovery_buff;               /*!机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）*/
    uint8_t cooling_buff;                /*!机器人枪口冷却倍率（直接值，值为 5 表示 5 倍冷却）*/
    uint8_t defence_buff;                /*!机器人防御增益（百分比，值为 50 表示 50%防御增益）*/
    uint8_t vulnerability_buff;          /*!机器人负防御增益（百分比，值为 30 表示-30%防御增益）*/
    uint16_t attack_buff;                /*!机器人攻击增益（百分比，值为 50 表示 50%攻击增益）*/
}__attribute__((__packed__))ext_buff_t;


/**
* 空中机器人能量状态 对应命令码ID为：0x0205
* 发送频率：10Hz
* 可攻击时间 单位 s 30s 递减至 0
* 剩余时间：
* 此状态的剩余时间（单位为：秒，向下取整，即冷却时间剩余 1.9 秒时，此值为 1）
* 若冷却时间为 0，但未呼叫空中支援，则该值为 0
*/
typedef  struct
{
    uint8_t airforce_status;            /*!空中机器人状态（0 为正在冷却，1 为冷却完毕，2 为正在空中支援）*/
    uint8_t time_remain;              /*!剩余时间*/
} __attribute__((__packed__))air_support_data_t;

/**
*  伤害状态  对应命令码ID为：0x0206
*  发送频率：伤害发生后发送
*  bit 0-3：当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线时，
*  该 4 bit 组成的数值为装甲模块或测速模块的 ID 编号；
*  当其他原因导致扣血时，该数值为 0
*  bit 4-7：血量变化类型
*  0x0 装甲伤害扣血；
*  0x1 模块掉线扣血；
*  0x2 超射速扣血；
*  0x3 超枪口热量扣血；
*  0x4 超底盘功率扣血；
*  0x5 装甲撞击扣血
*/
typedef  struct
{
    uint8_t armor_id : 4;                    /*!装甲ID */
    uint8_t HP_deduction_reason : 4;         /*!血量变化原因*/
}  __attribute__((__packed__)) hurt_data_t;


/**
*  实时射击信息   对应命令码ID为：0x0207
*  发送频率：射击后发送
*  子弹类型: 1：17mm 弹丸 2：42mm 弹丸
*  发射机构 ID：
*  1：1 号 17mm 发射机构
*  2：2 号 17mm 发射机构
*  3：42mm 发射机构
* 子弹射频 单位 Hz
* 子弹射速 单位 m/s
*/
typedef struct
{
    uint8_t bullet_type;                 /*!子弹类型*/
    uint8_t shooter_number;              /*!发射枪口ID*/
    uint8_t launching_frequency;         /*!子弹射频*/
    float initial_speed;                 /*!子弹射速*/
} __attribute__((__packed__)) shoot_data_t;


/**
*  子弹剩余发射数    对应命令码ID为：0x0208
*  发送频率：10Hz 周期发送，所有机器人发送
*   17mm 子弹剩余发射数量含义说明             联盟赛                              对抗赛
*    步兵机器人                   全队步兵与英雄剩余可发射 17mm 弹丸总量          全队 17mm 弹丸剩余可兑换数量
*    英雄机器人                   全队步兵与英雄剩余可发射 17mm 弹丸总量          全队 17mm 弹丸剩余可兑换数量
*    空中机器人、哨兵机器人         该机器人剩余可发射 17mm 弹丸总量               该机器人剩余可发射 17mm 弹丸总量
*
*   17mm 子弹剩余发射数目
*   42mm 子弹剩余发射数目
*   剩余金币数量
*/
typedef  struct
{
    uint16_t projectile_allowance_17mm;                 /*!17mm弹头剩余数量*/
    uint16_t projectile_allowance_42mm;                 /*!42mm弹头剩余数量*/
    uint16_t remaining_gold_coin;                       /*!金币剩余数量*/
}  __attribute__((__packed__)) projectile_allowance_t;

/**
*  机器人 RFID 状态  对应命令码ID为：0x0209
*  发送频率：1Hz    发送范围：己方装有 RFID模块的机器人
*  bit 位值为 1/0 的含义：是否已检测到该增益点 RFID 卡
*  bit 0：己方基地增益点
*  bit 1：己方环形高地增益点
*  bit 2：对方环形高地增益点
*  bit 3：己方 R3/B3 梯形高地增益点
*  bit 4：对方 R3/B3 梯形高地增益点
*  bit 5：己方 R4/B4 梯形高地增益点
*  bit 6：对方 R4/B4 梯形高地增益点
*  bit 7：己方能量机关激活点
*  bit 8：己方飞坡增益点（靠近己方一侧飞坡前）
*  bit 9：己方飞坡增益点（靠近己方一侧飞坡后）
*  bit 10：对方飞坡增益点（靠近对方一侧飞坡前）
*  bit 11：对方飞坡增益点（靠近对方一侧飞坡后）
*  bit 12：己方前哨站增益点
*  bit 13：己方补血点（检测到任一均视为激活）
*  bit 14：己方哨兵巡逻区
*  bit 15：对方哨兵巡逻区
*  bit 16：己方大资源岛增益点
*  bit 17：对方大资源岛增益点
*  bit 18：己方兑换区
*  bit 19：中心增益点（仅 RMUL 适用）
*  bit 20-31：保留
* 注：基地增益点、高地增益点、飞坡增益点、前哨站增益点、资源岛增益点、
* 补血点、兑换区、中心增益点（仅适用于 RMUL）和哨兵巡逻区的 RFID 卡
* 仅在赛内生效。在赛外，即使检测到对应的 RFID 卡，对应值也为 0。
*/
typedef  struct
{
    uint32_t rfid_status;                        /*!机器人RFID状态 */
}  __attribute__((__packed__)) rfid_status_t;

/**
* 飞镖机器人客户端指令数据   对应命令码ID为：0x020A
* 发送频率：10Hz 发送范围：己方飞镖机器人
*  当前飞镖发射站的状态
*  1：关闭；
*  2：正在开启或者关闭中
*  0：已经开启
*  保留
*  切换打击目标时的比赛剩余时间，单位秒，从未切换默认为 0。
*  最近一次操作手确定发射指令时的比赛剩余时间，单位秒, 初始值为 0。
*/
typedef  struct
{
    uint8_t dart_launch_opening_status;             /*!飞镖发射站状态*/
    uint8_t reserved;                               /*!保留*/
    uint16_t target_change_time;                    /*!切换打击目标时的比赛剩余时间*/
    uint16_t operate_launch_cmd_time;               /*!最近一次操作手确定发射指令时的比赛剩余时间*/
}  __attribute__((__packed__)) dart_client_cmd_t;

/**
*  地面机器人位置数据   对应命令码ID为：0x020B
*  发送频率：1Hz 发送范围：己方哨兵机器人
*  单位：m
*
*/

typedef  struct
{
    float hero_x;       /*!己方英雄机器人位置 x 轴坐标*/
    float hero_y;       /*!己方英雄机器人位置 y 轴坐标*/
    float engineer_x;   /*!己方工程机器人位置 x 轴坐标*/
    float engineer_y;   /*!己方工程机器人位置 y 轴坐标*/
    float standard_3_x; /*!己方 3 号步兵机器人位置 x 轴坐标*/
    float standard_3_y; /*!己方 3 号步兵机器人位置 y 轴坐标*/
    float standard_4_x; /*!己方 4 号步兵机器人位置 x 轴坐标*/
    float standard_4_y; /*!己方 4 号步兵机器人位置 y 轴坐标*/
    float standard_5_x; /*!己方 5 号步兵机器人位置 x 轴坐标*/
    float standard_5_y; /*!己方 5 号步兵机器人位置 y 轴坐标*/
}  __attribute__((__packed__)) ground_robot_position_t;

/**
*  雷达标记进度数据  对应命令码ID为：0x020C
*  发送频率：1Hz 发送范围：己方雷达机器人
*/

typedef  struct
{
    uint8_t mark_hero_progress;       /*!对方英雄机器人被标记进度：0-120*/
    uint8_t mark_engineer_progress;   /*!对方工程机器人被标记进度：0-120*/
    uint8_t mark_standard_3_progress; /*!对方 3 号步兵机器人被标记进度：0-120*/
    uint8_t mark_standard_4_progress; /*!对方 4 号步兵机器人被标记进度：0-120*/
    uint8_t mark_standard_5_progress; /*!对方 5 号步兵机器人被标记进度：0-120*/
    uint8_t mark_sentry_progress;     /*!对方哨兵机器人被标记进度：0-120*/
}  __attribute__((__packed__)) radar_mark_data_t;

/**
*  哨兵自主决策信息同步  对应命令码ID为：0x020D
*  发送频率：1Hz 发送范围：己方哨兵机器人
* bit 0-10：
*     除远程兑换外，哨兵成功兑换的发弹量，开局为 0，在哨兵成功兑
*     换一定发弹量后，该值将变为哨兵成功兑换的发弹量值。
* bit 11-14：
*     哨兵成功远程兑换发弹量的次数，开局为 0，在哨兵成功远程兑
*     换发弹量后，该值将变为哨兵成功远程兑换发弹量的次数。
* bit 15-18：
*     哨兵成功远程兑换血量的次数，开局为 0，在哨兵成功远程兑换
*     血量后，该值将变为哨兵成功远程兑换血量的次数。
* bit 19-31：保留
*/

typedef  struct
{
    uint32_t sentry_info;
}  __attribute__((__packed__)) sentry_info_t;

/**
*  雷达自主决策信息同步  对应命令码ID为：0x020E
*  发送频率：1Hz 发送范围：己方雷达机器人
* bit 0-1：
*     雷达是否拥有触发双倍易伤的机会，开局为 0，
*     数值为雷达拥有触发双倍易伤的机会，至多为 2
* bit 2：
*     对方是否正在被触发双倍易伤
*      0：对方未被触发双倍易伤
*      1：对方正在被触发双倍易伤
* bit 3-7：保留
*/

typedef  struct
{
    uint8_t radar_info;
}  __attribute__((__packed__)) radar_info_t;

/**
 *  交互数据接收信息   对应命令码ID为：0x0301
 * 子内容 ID：
 *      需为开放的子内容 ID
 * 发送者的 ID:
 *      需要校验发送者的 ID 正确性，例如红 1 发送给红 5，此项需要校验红 1
 * 接收者的 ID:
 *      仅限己方通信
 *      需为规则允许的多机通讯接收者
 *      若接收者为选手端，则仅可发送至发送者对应的选手端
 *      ID 编号详见附录
 * 内容数据段:
 *  最大为 113
*/
typedef  struct
{
    uint16_t data_cmd_id;                           /*! 子内容 ID*/
    uint16_t sender_id;                             /*! 发送者 ID */
    uint16_t receiver_id;                           /*! 接收者 ID */
    uint8_t user_data;                              /*! 内容数据段 */
} __attribute__((__packed__)) robot_interaction_data_t;
//TODO:机器人交互数据子内容的完善

/**
*  操作手可使用自定义控制器通过图传链路向对应的机器人发送数据  对应命令码ID为：0x0302
*/
typedef  struct
{
    uint8_t data[30];
}  __attribute__((__packed__)) custom_robot_data_t;

/**
*  云台手可通过选手端大地图向机器人发送固定数据。对应命令码为 0x0303
*  触发时发送，两次发送间隔不得低于 0.5 秒。
* Byte 0-3：
*     目标位置 x 轴坐标，单位 m
* Byte 4-7：
*     目标位置 y 轴坐标，单位 m
* Byte 8：
*     云台手按下的键盘按键通用键值
* Byte 9：
*     对方机器人 ID
* Byte 10-11：
*     信息来源 ID
*/
typedef  struct
{
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint8_t cmd_source;
}  __attribute__((__packed__)) map_command_t;

/**
*  通过遥控器发送的键鼠遥控数据将同步通过图传链路发送给对应机器人。  对应命令码ID为：0x0304
* Byte 0-1：
*     鼠标 x 轴移动速度，负值标识向左移动
* Byte 2-3：
*     鼠标 y 轴移动速度，负值标识向下移动
* Byte 4-5：
*     鼠标滚轮移动速度，负值标识向后滚动
* Byte 6：
*     鼠标左键是否按下：0 为未按下；1 为按下
* Byte 7：
*     鼠标右键是否按下：0 为未按下，1 为按下
* Byte 8-9：
*     键盘按键信息，每个 bit 对应一个按键，0 为未按下，1 为按下：
*      bit 0：W 键
*      bit 1：S 键
*      bit 2：A 键
*      bit 3：D 键
*      bit 4：Shift 键
*      bit 5：Ctrl 键
*      bit 6：Q 键
*      bit 7：E 键
*      bit 8：R 键
*      bit 9：F 键
*      bit 10：G 键
*      bit 11：Z 键
*      bit 12：X 键
*      bit 13：C 键
*      bit 14：V 键
*      bit 15：B 键
* Byte 10-11：
*     保留位
*/
typedef  struct
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
}  __attribute__((__packed__)) remote_control_t;

/**
*  选手端小地图接收的雷达发送的对方机器人的坐标数据。 对应命令码ID为：0x0305
*     Byte 0-1：
*     目标机器人 ID
* Byte 2-5：
*     目标 x 位置坐标，单位：m
* Byte 6-9：
*     目标 y 位置坐标，单位：m
*/
typedef  struct
{
    uint16_t target_robot_id;
    float target_position_x;
    float target_position_y;
}  __attribute__((__packed__)) map_robot_data_t;

/**
*  非链路数据，操作手可使用自定义控制器模拟键鼠操作选手端。 对应命令码ID为：0x0306
* Byte 0-1：
*     键盘键值：
*      bit 0-7：按键 1 键值
*      bit 8-15：按键 2 键值
* Byte 2-3：
*      bit 0-11：鼠标 X 轴像素位置
*      bit 12-15：鼠标左键状态
* Byte 4-5：
*      bit 0-11：鼠标 Y 轴像素位置
*      bit 12-15：鼠标右键状态
* Byte 6-7：
*     保留位
*/
typedef  struct
{
    uint16_t key_value;
    uint16_t x_position:12;
    uint16_t mouse_left:4;
    uint16_t y_position:12;
    uint16_t mouse_right:4;
    uint16_t reserved;
}  __attribute__((__packed__)) custom_client_data_t;

/**
*  选手端小地图接收的哨兵机器人或选择半自动控制方式的机器人通过常规链路发送的对方机器人的坐标数据。 对应命令码ID为：0x0307
*     小地图左下角为坐标原点，水平向右为 X 轴正
*     方向，竖直向上为 Y 轴正方向。显示位置将按
*     照场地尺寸与小地图尺寸等比缩放，超出边界
*     的位置将在边界处显示
* Byte 0：
*     1：到目标点攻击
*     2：到目标点防守
*     3：移动到目标点
* Byte 1-2：
*     路径起点 x 轴坐标，单位：dm
* Byte 3-4：
*     路径起点 y 轴坐标，单位：dm
* Byte 5-53：
*     路径点 x 轴增量数组，单位：dm
* Byte 54-102：
*     路径点 y 轴增量数组，单位：dm
* Byte 103-104：
*     发送者 ID
*/
typedef  struct
{
    uint8_t intention;
    uint16_t start_position_x;
    uint16_t start_position_y;
    int8_t delta_x[49];
    int8_t delta_y[49];
    uint16_t sender_id;
}  __attribute__((__packed__)) map_data_t;

/**
*  己方机器人可通过常规链路向己方任意选手端发送自定义的消息，该消息会在己方选手端特定位置显示。 对应命令码ID为：0x0308
* Byte 0-1：
*     发送者 ID
* Byte 2-3：
*     接收者 ID
* Byte 4-33：
*     字符
*/
typedef  struct
{
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];
}  __attribute__((__packed__)) custom_info_t;

/**
 * @brief 裁判系统接收初始化
 */
void Referee_system_Init(uint8_t *  rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
/**
 * @brief 裁判系统接收数据帧解包
 */
void Referee_Data_Unpack();
/**
 * @brief 裁判系统数据更新并保存
 */
void Referee_Data_Solve(uint8_t* referee_data_frame);
/**
 * @brief 线程入口函数
 */
void referee_thread_entry(void *argument);
#endif //REFEREE_SYSTEM_H
