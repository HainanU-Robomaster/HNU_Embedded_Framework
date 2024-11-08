/*
* Change Logs:
* Date            Author          Notes
* 2023-09-24      ChuShicheng     first version
*/
#include "chassis_task.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "robot.h"

//#define CLOSE_ALL_MOTOR
#define SLIP_DETECTION // 是否使能打滑检测
/* -------------------------------- 线程间通讯话题相关 ------------------------------- */
// 订阅
MCN_DECLARE(ins_topic);
static McnNode_t ins_topic_node;
static struct ins_msg ins;
MCN_DECLARE(chassis_cmd);
static McnNode_t chassis_cmd_node;
static struct chassis_cmd_msg chassis_cmd;
// 发布
MCN_DECLARE(chassis_fdb);
static struct chassis_fdb_msg chassis_fdb_data;

static void chassis_sub_init(void);
static void chassis_pub_push(void);
static void chassis_sub_pull(void);

/* --------------------------------- 电机控制相关 --------------------------------- */
#define LEFT    0
#define RIGHT   1
#define FRONT   0
#define BACK    1
#define LEFT_FRONT  0
#define RIGHT_FRONT 1
#define RIGHT_BACK  2
#define LEFT_BACK   3
#define LEN_LEN_LOW     0.16f // 单位：m
#define LEN_LEN_MID     0.24f // 单位：m
#define LEN_LEN_HIG     0.32f // 单位：m
#define POSITION_X_LIMIT 2.5f // 位置误差限幅
#define FORCE_LIMIT 200.0f // 支持力限幅
/* 髋关节电机零点偏移 需要提前测出 */
#define HT_OFFSET_LF -0.28999f  // 左上电机 -0.3279
#define HT_OFFSET_RF  0.26667f  // 右上电机 0.3366
#define HT_OFFSET_LB  0.35411f  // 左下电机 0.3221
#define HT_OFFSET_RB -0.31330f  // 右下电机 -0.3366
#define ROLL_OFFSET  -0.28f     // TODO：目前C板陀螺仪安装误差，后续可能变化
#define LEG_LENGHT_F  30.0f     // TODO: 腿长控制前馈，根据机体质量调整
float len_feed = LEG_LENGHT_F;
#define LEG_MASS      0.824f    // 腿部质量
/*跳跃相关*/
#define JUMP_TORQUE_PRESS    30.0f   // 跳跃时下压扭矩，正负需对应各电机确定
#define JUMP_TORQUE_SHRINK   22.0f   // 跳跃时回缩扭矩，正负需对应各电机确定
/* 髋关节电机扭矩限制 */
#define HT_OUTPUT_LIMIT 30.0f
float ht_output_limit = HT_OUTPUT_LIMIT;
#define HT_INIT_OUT 2.0f   // 电机初始化时的输出扭矩,确保能撞到限位
#define LK_OUTPUT_LIMIT 3.5f
float lk_output_limit = LK_OUTPUT_LIMIT;
#define LK_TOR_TO_CUR 406.21344  // LK9025 扭矩转换电流系数
/* 当前lk9025使用多电机模式控制 */
#define ROLL_PID_MAXOUT 60

static pid_obj_t *follow_pid; // 用于底盘跟随云台计算vw
static pid_obj_t *theta_pid;  // 双腿角度协调控制
static pid_obj_t *yaw_pid;    // 航向角控制， 输出为vx的补偿，与vx期望累积
static pid_obj_t *roll_pid;   // 横滚角控制
static float yaw_target;
static float pos_x_offset[2]; // 底盘x位移的偏置，倒地自起后重置

static void leg_calc();
static float vx_change_limit(float vx_cmd, float vx_now);

/* 没有加入LQR前测试VMC用 */
static struct leg_controller_t{
    pid_obj_t *length_pid;
    pid_obj_t *angle_pid;
}leg_controller[2];

static leg_obj_t *leg[2] = {NULL};
static float ft_l[2], ft_r[2];  // FT = [PendulumForce PendulumTorque]
static float vmc_out_l[2];  // vmc计算得出的扭矩值 左边腿
static float vmc_out_r[2];  // vmc计算得出的扭矩值 右边腿
static float jump_out_l[2]; // 跳跃附加扭矩值 左边腿
static float jump_out_r[2]; // 跳跃附加扭矩值 右边腿
static float jump_start_time;  // 跳跃开始时间
static float jump_dt_time;     // 起跳后的间隔时间
static float is_jumping;       // 跳跃标志位
static float jump_flag;        // 切入跳跃模式只跳一次
static float support_force[2];  //左右腿的地面支持力 用于离地检测
float len_ref = 0.16f;
static float angle_ref = /*1.57f*/1.57f;

/* 髋关节电机 HT04 实例 */
static ht_motor_object_t *ht_motor[4];
/* 驱动电机 LK9025 实例 */
static lk_motor_object_t *lk_motor[2];

static int chassis_motor_init(void);
/* 髋关节电机初始化，撞限位，设置零点 */
/**
 * @brief 髋关节电机初始化,零点获取
 * @note ht04为单编码器，每次上电通过撞限位设置零点并减去固定偏移量
 */
static void leg_init_get_zero();
/* 使能底盘所有电机 */
static void motor_enable();
/* 失能底盘所有电机 */
static void motor_relax();

/* --------------------------------- LQR控制相关 -------------------------------- */
/* LQR 反馈矩阵（由 matlab 生成），3组分别对应不同腿长 */
//Bot :LQRKbuf[0][]
//Mid :LQRKbuf[1][]
//Bot :LQRKbuf[2][]
//安装云台前参数
//static float LQR_k[3][12]=
//        {
//                //Order: K00 K01 K02 K03 K04 K05 K10 K11 K12 K13 K14 K15
//                {-10.6825,  -0.9177,  -1.1850,  -1.7416,   7.3019,   0.9677,   15.6096,   1.0837,   2.3166,   3.1649,  15.0278,   1.0024},	//K BOT 160
//                {-13.8348,  -1.4194,  -1.3110,  -1.9504,   5.8304,   0.8424,   15.4082,   1.1319,   1.6091,   2.2124,  18.6314,   1.4545},	//K MID 240
//                {-16.0400,  -1.8837,  -1.3636,  -2.0567,   4.7952,   0.7471,   14.7615,   1.1021,   1.1713,   1.6184,  20.4058,   1.6923},	//K TOP 320
//        }; // Ts=1.424ms Q=diag([1 1 500 100 5000 1]) R=[240 0;0 25]

//安装云台后
// (郑)
// static float LQR_k[3][12]=
//         {
//                 {-14.8446,   -1.0341,   -1.1017,   -1.8418,    8.7788,    1.1898,   25.6510,    1.6674,    2.8894,    4.5455,   14.7041,    1.1533},	//K BOT 160
//                 {-19.3345,   -1.6403,   -1.2814,   -2.1545,    7.0545,    1.0217,   24.8938,    1.7897,    2.0581,    3.2504,   20.0129,    1.8501},	//K MID 240
//                 {-22.3391,   -2.1924,   -1.3580,   -2.3039,    5.7755,    0.8876,   23.3362,    1.7671,    1.5156,    2.4006,   22.6439,    2.2150},	//K TOP 320
//         }; // Ts=1.424ms Q=diag([1 1 500 100 5000 1]) R=[240 0;0 25]
//static float LQR_k[3][12] =
//        {
//                {-10.4976, -1.0562, -1.1585, -1.7413, 7.7090, 1.2488, 17.5371, 2.0506, 2.5947, 3.6668, 13.5962, 1.2738}, // 替换后的 K BOT 160
//                {-13.4572, -1.5280, -1.2777, -1.9252, 6.3431, 1.0870, 17.9355, 1.9838 ,2.0199 ,2.8392, 17.3738, 1.8306}, // K BOT 240 [60 40 450 1 5000 1]
//                {-15.1573, -1.7839, -1.1963, -1.8960, 4.9468, 0.9224, 17.3352, 1.5197, 1.4588, 2.1008, 17.0028, 2.0054}, // K BOT 320
//        };//能用

//static float LQR_k[3][12]=
//        {
//                {-10.0020, -0.9133, -1.0638,-1.5270,7.6617,1.2262,17.5983,1.6476,2.6729,3.5556,13.3460,1.2006}, // K BOT 160 [100 1 500 100 5000 1]
////                {  -11.8948,-0.9146,-0.6782,-1.1550,6.6621,1.1664,24.3873,2.3787,3.6497,5.7367,6.0845,0.2744}, // K BOT 160 [60 40 450 1 5000 1]
//                {-12.8744, -1.3209, -1.1211, -1.7588, 5.9249, 1.0449, 18.0335, 1.5066, 1.9438, 2.7914, 14.7395, 1.6264}, // K BOT 240 [60 40 450 1 5000 1]
//                {-15.1573, -1.7839, -1.1963, -1.8960, 4.9468, 0.9224, 17.3352, 1.5197, 1.4588, 2.1008, 17.0028, 2.0054}, // K BOT 320
//        }; // Ts=1.5ms Q=diag([60 40 450 1 5000 1]) R=[240 0;0 24]//也能用,潘磊标定参数前

static float LQR_k[3][12]=
        {
                {  -23.4204 , -1.5991  ,-1.0524 , -2.1569   ,7.6023  , 0.7704,31.0960    ,2.4048    ,2.7157    ,5.3720  , 12.3657  ,  0.4830}, // K BOT 160 [100 1 500 100 5000 1]
                {-12.8744, -1.3209, -1.1211, -1.7588, 5.9249, 1.0449, 18.0335, 1.5066, 1.9438, 2.7914, 14.7395, 1.6264}, // K BOT 240 [60 40 450 1 5000 1]
                {-15.1573, -1.7839, -1.1963, -1.8960, 4.9468, 0.9224, 17.3352, 1.5197, 1.4588, 2.1008, 17.0028, 2.0054}, // K BOT 320
        }; // Ts=1.5ms Q=diag([1 1 100 500 5000 1]) R=[240 0;0 25]

/* [T Tp(髋)] */
static float LQROutBuf[2][2]={0};
static float LQRXerrorBuf[2][6]={0};
static float LQRXObsBuf[2][6]={0};
static float LQRXRefBuf[2][6]={0}; /*LQRXObsBuf[0][2] - LQRXRefBuf[0][2]*/

static arm_matrix_instance_f32 MatLQRObs_L  = {6, 1, LQRXObsBuf[LEFT]};
static arm_matrix_instance_f32 MatLQRObs_R  = {6, 1, LQRXObsBuf[RIGHT]};
static arm_matrix_instance_f32 MatLQRRef_L  = {6, 1, LQRXRefBuf[LEFT]};
static arm_matrix_instance_f32 MatLQRRef_R  = {6, 1, LQRXRefBuf[RIGHT]};
static arm_matrix_instance_f32 MatLQRNegK   = {2, 6, (float*)LQR_k[0]};
static arm_matrix_instance_f32 MatLQRErrX_L = {6, 1, LQRXerrorBuf[LEFT]};
static arm_matrix_instance_f32 MatLQRErrX_R = {6, 1, LQRXerrorBuf[RIGHT]};
static arm_matrix_instance_f32 MatLQROutU_L = {2, 1, LQROutBuf[LEFT]};
static arm_matrix_instance_f32 MatLQROutU_R = {2, 1, LQROutBuf[RIGHT]};

/*Calculate X. Output is u (T,Tp)`*/
static void LQR_cal()
{
	//Calculate error
	arm_mat_sub_f32(&MatLQRObs_L, &MatLQRRef_L, &MatLQRErrX_L);
    //Calculate error
    arm_mat_sub_f32(&MatLQRObs_R, &MatLQRRef_R, &MatLQRErrX_R);

    // TODO:加入斜坡控制，避免较大振荡, 目前的限制是不合理的，反而会让系统无法快速收敛
    // 对位移误差进行限幅
//    LIMIT_MIN_MAX(LQRXerrorBuf[LEFT][2], -POSITION_X_LIMIT, POSITION_X_LIMIT);
//    LIMIT_MIN_MAX(LQRXerrorBuf[RIGHT][2], -POSITION_X_LIMIT, POSITION_X_LIMIT);

    //Calculate output value
    arm_mat_mult_f32(&MatLQRNegK, &MatLQRErrX_L, &MatLQROutU_L);
    //Calculate output value
    arm_mat_mult_f32(&MatLQRNegK, &MatLQRErrX_R, &MatLQROutU_R);
}

static pid_obj_t *length_pid[2];

/* ------------------------------- 打滑检测卡尔曼滤波部分 ------------------------------ */
//TODO:后续考虑移入观测线程，建立整车观测器
static KalmanFilter_t chassis_kf[2];

/**
*   |     x     |
*   |    vx     |
*   |    ax     |
*/
static void chassis_kf_init(void)
{
    static float P_Init[9] =
    {
        10, 0, 0,
        0, 30, 0,
        0, 0, 10,
    };
    static float F_Init[9] =
    {
        1, 0.0015, 0.5*0.0015*0.0015,
        0, 1, 0.0015,
        0, 0, 1,
    };
    static float Q_Init[9] =
    {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1,
    };
    // static float Q_Init[9] =
    // {
    //     0.25*0.0015*0.0015*0.0015*0.0015, 0.5*0.0015*0.0015*0.0015, 0.5*0.0015*0.0015,
    //     0.5*0.0015*0.0015*0.0015,         0.0015*0.0015,            0.0015,
    //     0.5*0.0015*0.0015,                0.0015,                   1,
    // };
    // 设置最小方差
    static float state_min_variance[3] = {0.03, 0.005, 0.1};

    //  电机编码器测得位移，电机编码器测得速度，加速度计解算得出速度
    static uint8_t measurement_reference[3] = {1,2,2};

    static float measurement_degree[3] = {1,1,1};
    // 根据measurement_reference与measurement_degree生成H矩阵如下（在当前周期全部测量数据有效情况下）

//    static float mat_R_diagonal_elements[2] = {28000,7000}; // 方差即为影响置信度，方差越大置信度越低  /* 打滑检测：800000 1000 */1
//    //根据mat_R_diagonal_elements生成R矩阵如下（在当前周期全部测量数据有效情况下）
    static float mat_R_diagonal_elements[3] = {20000, 5000, 20000}; // 方差即为影响置信度，方差越大置信度越低  /* 打滑检测：800000 1000 */1
    //根据mat_R_diagonal_elements生成R矩阵如下（在当前周期全部测量数据有效情况下）

    // 开启自动调整
    chassis_kf[LEFT].UseAutoAdjustment = 1;
    Kalman_Filter_Init(&chassis_kf[LEFT], 3, 0, 3);

    // 设置矩阵值
    memcpy(chassis_kf[LEFT].P_data, P_Init, sizeof(P_Init));
    memcpy(chassis_kf[LEFT].F_data, F_Init, sizeof(F_Init));
    memcpy(chassis_kf[LEFT].Q_data, Q_Init, sizeof(Q_Init));
    memcpy(chassis_kf[LEFT].MeasurementMap, measurement_reference, sizeof(measurement_reference));
    memcpy(chassis_kf[LEFT].MeasurementDegree, measurement_degree, sizeof(measurement_degree));
    memcpy(chassis_kf[LEFT].MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
    memcpy(chassis_kf[LEFT].StateMinVariance, state_min_variance, sizeof(state_min_variance));

    chassis_kf[RIGHT].UseAutoAdjustment = 1;
    Kalman_Filter_Init(&chassis_kf[RIGHT], 3, 0, 3);

    // 设置矩阵值
    memcpy(chassis_kf[RIGHT].P_data, P_Init, sizeof(P_Init));
    memcpy(chassis_kf[RIGHT].F_data, F_Init, sizeof(F_Init));
    memcpy(chassis_kf[RIGHT].Q_data, Q_Init, sizeof(Q_Init));
    memcpy(chassis_kf[RIGHT].MeasurementMap, measurement_reference, sizeof(measurement_reference));
    memcpy(chassis_kf[RIGHT].MeasurementDegree, measurement_degree, sizeof(measurement_degree));
    memcpy(chassis_kf[RIGHT].MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
    memcpy(chassis_kf[RIGHT].StateMinVariance, state_min_variance, sizeof(state_min_variance));
}

static float imu_motion_velocity[2];
//float vx_kf_dt = 0.003;
static void chassis_kf_update(void)
{
    static float _kf_dt, _kf_start;
    if(_kf_start != 0) // 避免第一次计算错误
        _kf_dt = (dwt_get_time_ms() - _kf_start) / 1000.0f;
    else
        _kf_dt = 0.003f;
    _kf_start = dwt_get_time_ms();
    imu_motion_velocity[LEFT] += ins.motion_accel_b[0] * _kf_dt;  // 机体绝对加速度积分得出速度
    imu_motion_velocity[RIGHT] += ins.motion_accel_b[0] * _kf_dt;  // 机体绝对加速度积分得出速度
    if(lk_motor[LEFT]->measure.speed_rads == 0)
        imu_motion_velocity[LEFT] = 0;
    if(lk_motor[RIGHT]->measure.speed_rads == 0)
        imu_motion_velocity[RIGHT] = 0;
    chassis_kf[LEFT].MeasuredVector[0] = lk_motor[LEFT]->measure.total_angle * WHEEL_RADIUS;
    chassis_kf[LEFT].MeasuredVector[1] = imu_motion_velocity[LEFT] + leg[LEFT]->d_phi0*leg[LEFT]->PendulumLength + ins.gyro[2] * 0.25f;
    chassis_kf[LEFT].MeasuredVector[2] = lk_motor[LEFT]->measure.speed_rads * WHEEL_RADIUS;
    Kalman_Filter_Update(&chassis_kf[LEFT]);

    chassis_kf[RIGHT].MeasuredVector[0] = lk_motor[RIGHT]->measure.total_angle * WHEEL_RADIUS;
    chassis_kf[RIGHT].MeasuredVector[1] = imu_motion_velocity[RIGHT] + leg[RIGHT]->d_phi0*leg[RIGHT]->PendulumLength - ins.gyro[2] * 0.25f;
    chassis_kf[RIGHT].MeasuredVector[2] = lk_motor[RIGHT]->measure.speed_rads * WHEEL_RADIUS;
    Kalman_Filter_Update(&chassis_kf[RIGHT]);
}

// static float dphi_last[2], gyro_last;
//     static float calu_dt, calu_start;

// static void chassis_kf_update(void)
// {
//     if(calu_start != 0) // 避免第一次计算错误
//         calu_dt = (dwt_get_time_ms() - calu_start) / 1000.0f;
//     else
//         calu_dt = 0.001f; 
//     calu_start = dwt_get_time_ms();

//     chassis_kf[LEFT].MeasuredVector[0] = lk_motor[LEFT]->measure.total_angle * WHEEL_RADIUS;
//     chassis_kf[LEFT].MeasuredVector[1] = lk_motor[LEFT]->measure.speed_rads * WHEEL_RADIUS;
//     chassis_kf[LEFT].MeasuredVector[2] = ins.motion_accel_b[0];
//                                         + (leg[LEFT]->d_phi0-dphi_last[LEFT])*leg[LEFT]->PendulumLength
//                                         + (ins.gyro[2]-gyro_last)*0.25f;
    
//     Kalman_Filter_Update(&chassis_kf[LEFT]);
//     // observe_vx[LEFT] = *observe_data[LEFT];

//     chassis_kf[RIGHT].MeasuredVector[0] = lk_motor[RIGHT]->measure.total_angle * WHEEL_RADIUS;
//     chassis_kf[RIGHT].MeasuredVector[1] = lk_motor[RIGHT]->measure.speed_rads * WHEEL_RADIUS;
//     chassis_kf[RIGHT].MeasuredVector[2] = ins.motion_accel_b[0];
//                                          + (leg[RIGHT]->d_phi0-dphi_last[RIGHT])*leg[RIGHT]->PendulumLength
//                                          + (ins.gyro[2]-gyro_last)*0.25f;
//     Kalman_Filter_Update(&chassis_kf[RIGHT]);
//     // observe_vx[RIGHT] = *observe_data[RIGHT];

//     dphi_last[LEFT] = leg[LEFT]->d_phi0;
//     dphi_last[RIGHT] = leg[RIGHT]->d_phi0;
//     gyro_last = ins.gyro[2];
// }

/* --------------------------------- 底盘运动学解算 --------------------------------- */
/* 根据宏定义选择的底盘类型使能对应的解算函数 */
#ifdef BSP_CHASSIS_OMNI_MODE
static void omni_calc(struct chassis_cmd_msg *cmd, int16_t* out_speed);
static void (*chassis_calc_moto_speed)(struct chassis_cmd_msg *cmd, int16_t* out_speed) = omni_calc;
#endif /* BSP_CHASSIS_OMNI_MODE */
#ifdef BSP_CHASSIS_MECANUM_MODE
void mecanum_calc(struct chassis_cmd_msg *cmd, int16_t* out_speed);
void (*chassis_calc_moto_speed)(struct chassis_cmd_msg *cmd, int16_t* out_speed) = mecanum_calc;
#endif /* BSP_CHASSIS_MECANUM_MODE */
static void absolute_cal(struct chassis_cmd_msg *cmd, float angle);
/**
 * @brief 底盘跳跃处理
 */
static void jumping_control(void);

/* --------------------------------- 底盘线程入口 --------------------------------- */
static float chassis_dt;

static void update_LQR_obs() {
    static uint8_t x_ref_update_flag;  // 用于位移介入暂存介入时的期望位置
    /*  更新观测矩阵 */
    LQRXObsBuf[LEFT][0] = PI / 2 - leg[LEFT]->PendulumRadian - ins.pitch * DEGREE_2_RAD;
    LQRXObsBuf[LEFT][1] = -leg[LEFT]->d_phi0 - ins.gyro[1] * DEGREE_2_RAD;
#ifdef SLIP_DETECTION
    LQRXObsBuf[LEFT][2] = chassis_kf[LEFT].FilteredValue[0] - pos_x_offset[LEFT];
    LQRXObsBuf[LEFT][3] = chassis_kf[LEFT].FilteredValue[1];
    // LQRXObsBuf[LEFT][2] = lk_motor[LEFT]->measure.total_angle * WHEEL_RADIUS - pos_x_offset[LEFT];
#else
    LQRXObsBuf[LEFT][2] = lk_motor[LEFT]->measure.total_angle * WHEEL_RADIUS - pos_x_offset[LEFT];
    LQRXObsBuf[LEFT][3] = lk_motor[LEFT]->measure.speed_rads * WHEEL_RADIUS;
#endif /*SLIP_DETECTION*/
    LQRXObsBuf[LEFT][4] = ins.pitch * DEGREE_2_RAD/* + 0.064*/;
    LQRXObsBuf[LEFT][5] = ins.gyro[1] * DEGREE_2_RAD;

    LQRXObsBuf[RIGHT][0] = PI / 2 - leg[RIGHT]->PendulumRadian - ins.pitch * DEGREE_2_RAD;;
    LQRXObsBuf[RIGHT][1] = -leg[RIGHT]->d_phi0 - ins.gyro[1] * DEGREE_2_RAD;
#ifdef SLIP_DETECTION
    LQRXObsBuf[RIGHT][2] = chassis_kf[RIGHT].FilteredValue[0] - pos_x_offset[RIGHT];
    LQRXObsBuf[RIGHT][3] = chassis_kf[RIGHT].FilteredValue[1];
    // LQRXObsBuf[RIGHT][2] = lk_motor[RIGHT]->measure.total_angle * WHEEL_RADIUS - pos_x_offset[RIGHT];
#else
    LQRXObsBuf[RIGHT][2] = lk_motor[RIGHT]->measure.total_angle * WHEEL_RADIUS - pos_x_offset[RIGHT];
    LQRXObsBuf[RIGHT][3] = lk_motor[RIGHT]->measure.speed_rads * WHEEL_RADIUS;
#endif /*SLIP_DETECTION*/
    LQRXObsBuf[RIGHT][4] = ins.pitch * DEGREE_2_RAD;
    LQRXObsBuf[RIGHT][5] = ins.gyro[1] * DEGREE_2_RAD;

    //计算位置目标
    //  TODO：目前位移项error一直为0，并没有有效利用，需要完善
//    LQRXRefBuf[LEFT][2]  += chassis_cmd.vx * 0.001f*0.003f /*+ (chassis_cmd.vw*0.001f*0.25f)*/ + yaw_pid->Output;  // cmd更新频率约为3ms,单位为米
//    LQRXRefBuf[RIGHT][2] += chassis_cmd.vx * 0.001f*0.003f /*- (chassis_cmd.vw*0.001f*0.25f)*/ - yaw_pid->Output;  // cmd更新频率约为3ms,单位为米
    LQRXRefBuf[LEFT][2]  += chassis_cmd.vx * 0.001f*0.003f  /*+ (chassis_cmd.vw*0.0001f*0.08f)*/- yaw_pid->Output;  // cmd更新频率约为3ms,单位为米
    LQRXRefBuf[RIGHT][2] += chassis_cmd.vx * 0.001f*0.003f  /*- (chassis_cmd.vw*0.0001f*0.08f)*/+ yaw_pid->Output;  // cmd更新频率约为3ms,单位为米
    
    // 期望线速度
/*    LQRXRefBuf[LEFT][3]  = vx_change_limit((chassis_cmd.vx/1000), LQRXObsBuf[LEFT][3]) +yaw_pid->Output;   // 米每秒
    LQRXRefBuf[RIGHT][3] = vx_change_limit((chassis_cmd.vx/1000), LQRXObsBuf[RIGHT][3]) -yaw_pid->Output;  // 米每秒*/
//    LQRXRefBuf[LEFT][3]  = (chassis_cmd.vx/1000) + yaw_pid->Output;   // 米每秒
//    LQRXRefBuf[RIGHT][3] = (chassis_cmd.vx/1000) - yaw_pid->Output;   // 米每秒

    //更新航向角期望
//    yaw_target += chassis_cmd.vw * 0.001f; // cmd更新频率为1ms,单位为度
    yaw_target += chassis_cmd.vw * 0.0006f; // cmd更新频率为1ms,单位为度

}

void chassis_task_init(void)
{
    chassis_sub_init();
    chassis_motor_init();
#ifdef SLIP_DETECTION
    chassis_kf_init();
#endif /* SLIP_DETECTION */
}

float roll_pid_kp = 15,roll_pid_kd=0.05f;

/**
 * @brief 底盘控制任务,在RTOS中应该设定为200hz运行
 */
void chassis_control_task(void)
{
    /* 更新该线程所有的订阅者 */
    chassis_sub_pull();

    // 切换腿长姿态
    switch (chassis_cmd.leg_level)
    {
    case LEG_LOW:
        /* 更改腿长 */
        leg[LEFT]->length_ref = LEN_LEN_LOW;
        leg[RIGHT]->length_ref = LEN_LEN_LOW;
        /* 切换对应的 K 矩阵 */
        MatLQRNegK.pData = (float*)LQR_k[0];
        break;
    case LEG_MID:
        leg[LEFT]->length_ref = LEN_LEN_MID;
        leg[RIGHT]->length_ref = LEN_LEN_MID;
        MatLQRNegK.pData = (float*)LQR_k[1];
        break;
    case LEG_HIG:
        leg[LEFT]->length_ref = LEN_LEN_HIG;
        leg[RIGHT]->length_ref = LEN_LEN_HIG;
        MatLQRNegK.pData = (float*)LQR_k[1];
        break;
    default:
        leg[LEFT]->length_ref = LEN_LEN_LOW;
        leg[RIGHT]->length_ref = LEN_LEN_LOW;
        MatLQRNegK.pData = (float*)LQR_k[0];
        break;
    }

    switch (chassis_cmd.ctrl_mode)
    {
    case CHASSIS_RELAX:
        motor_relax();
        chassis_fdb_data.stand_state = CAHSSIS_IS_FELL;
        /* 倒地自起后重置yaw期望和x位移的offset */
        yaw_target = ins.yaw_total_angle;
        pos_x_offset[LEFT] = chassis_kf[LEFT].FilteredValue[0];
        pos_x_offset[RIGHT] = chassis_kf[RIGHT].FilteredValue[0];
            roll_pid->Kp = 0;
            roll_pid->Ki = 0;
            roll_pid->Kd = 0.0;
        roll_pid->MaxOut = 0;
        break;
    case CHASSIS_INIT:
        motor_enable();
        leg_init_get_zero();
        /* 倒地自起后重置yaw期望和x位移的offset */
        yaw_target = ins.yaw_total_angle;
        pos_x_offset[LEFT] = chassis_kf[LEFT].FilteredValue[0];
        pos_x_offset[RIGHT] = chassis_kf[RIGHT].FilteredValue[0];
        break;
    case CHASSIS_RECOVERY:
        motor_enable();
        if(usr_abs(LQRXObsBuf[LEFT][4]) < 0.05f && usr_abs(LQRXObsBuf[RIGHT][4]) < 0.05f)
            chassis_fdb_data.stand_state = CAHSSIS_IS_STAND;
        else
            chassis_fdb_data.stand_state = CAHSSIS_IS_FELL;

        /* 保持腿长，便于倒地自起 */
        leg[LEFT]->length_ref = 0.1;
        leg[RIGHT]->length_ref = 0.1;
        /* 倒地自起后重置yaw期望和x位移的offset */
        yaw_target = ins.yaw_total_angle;
        pos_x_offset[LEFT] = chassis_kf[LEFT].FilteredValue[0];
        pos_x_offset[RIGHT] = chassis_kf[RIGHT].FilteredValue[0];
//         pos_x_offset[LEFT] = lk_motor[LEFT]->measure.total_angle * WHEEL_RADIUS;
//         pos_x_offset[RIGHT] = lk_motor[RIGHT]->measure.total_angle * WHEEL_RADIUS;
        LQRXRefBuf[LEFT][2] = 0;
        LQRXRefBuf[RIGHT][2] = 0;
        //TODO: 处于该模式下，应该屏蔽遥控器等控制
        roll_pid->Kp = 0;
        roll_pid->Ki = 0;
        roll_pid->Kd = 0.0;
        roll_pid->MaxOut = 0;
        break;
    case CHASSIS_FOLLOW_GIMBAL:
        // motor_enable();
        roll_pid->Kp = roll_pid_kp;
        roll_pid->Ki = 0;
        roll_pid->Kd = roll_pid_kd;
        roll_pid->MaxOut = ROLL_PID_MAXOUT;
        break;
    case CHASSIS_SPIN:

        break;
    case CHASSIS_OPEN_LOOP:
        motor_enable();
        roll_pid->Kp = roll_pid_kp;
        roll_pid->Ki = 0;
        roll_pid->Kd = roll_pid_kd;
        roll_pid->MaxOut = ROLL_PID_MAXOUT;
        /* 更改腿长 */
//        leg[LEFT]->length_ref = len_ref;
//        leg[RIGHT]->length_ref = len_ref;
        /* 切换对应的 K 矩阵 */
//        MatLQRNegK.pData = (float*)LQR_k[0];
        break;
    // case CHASSIS_STAND_MID:
    //     motor_enable();
    //     leg[LEFT]->length_ref = len_ref;
    //     leg[RIGHT]->length_ref = len_ref;
    //     MatLQRNegK.pData = (float*)LQR_k[1];
    //     break;
    // case CHASSIS_STAND_HIG:
    //     //TODO: 添加对应的遥控切换键位
    //     motor_enable();
    //     leg[LEFT]->length_ref = len_ref;
    //     leg[RIGHT]->length_ref = len_ref;
    //     MatLQRNegK.pData = (float*)LQR_k[2];
    //     break;
    case CHASSIS_JUMP:
        motor_enable();
        jumping_control();
        break;
    case CHASSIS_STOP:
        ht_motor_disable_all();
        motor_relax();
        break;
    case CHASSIS_FLY:
        break;
    case CHASSIS_AUTO:
        break;
    default:
        motor_relax();
        break;
    }

#ifdef SLIP_DETECTION
    chassis_kf_update();
#endif /*SLIP_DETECTION*/
    leg_calc(); // 保证稳定的运算频率，不受模式影响
    chassis_fdb_data.lk_l = lk_motor[LEFT]->measure;
    chassis_fdb_data.lk_r = lk_motor[RIGHT]->measure;
    /* 更新发布该线程的msg */
    chassis_pub_push();
}

/* --------------------------------- 电机控制相关 --------------------------------- */
static void leg_init_get_zero()
{
    // 首先各个电机给定一个适当的力矩，并持续，确保撞到限位
    chassis_fdb_data.leg_state = LEG_BACK_STEP;
    osDelay(2000);
    // 撞到限位后，控制电机在此处设置零点
    for (uint8_t i = 0; i < 4; i++)
    {
        ht_motor[i]->set_mode(ht_motor[i], CMD_ZERO_POSITION);
    }
    // 电机零点设置完成，正常零点为减去各偏移量
    chassis_fdb_data.leg_state = LEG_BACK_IS_OK;
    // 该函数仅在每次重新上电执行
}
static void motor_enable()
{
    ht_motor_enable_all();  // 海泰所有电机进入 motor 模式
    for (uint8_t i = 0; i < 4; i++)
    {
        ht_motor_set_type(ht_motor[i], MOTOR_ENALBED);
    }
    for (uint8_t i = 0; i < 2; i++)
    {
        lk_motor_enable(lk_motor[i]);
    }
}

static void motor_relax()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        ht_motor_set_type(ht_motor[i], MOTOR_STOP);
    }
    for (uint8_t i = 0; i < 2; i++)
    {
        lk_motor_relax(lk_motor[i]);
    }
}

static float control_dt[4];
static float control_start[4];

/* 1 号电机 */
static ht_motor_para_t ht_control_1(ht_motor_measure_t measure)
{
    control_dt[0] = dwt_get_time_us() - control_start[0];
    control_start[0] = dwt_get_time_us();
    static ht_motor_para_t set;
    float send_t;

    // 每次上电归中电机给定一个适当的力矩，并持续，确保撞到限位

    if(chassis_cmd.ctrl_mode == CHASSIS_INIT)
    {
        send_t = HT_INIT_OUT;
    }
    else
    {
        send_t = vmc_out_l[FRONT] + jump_out_l[FRONT];
#ifdef CLOSE_ALL_MOTOR
        send_t = 0;
#endif
    }
    LIMIT_MIN_MAX(send_t, -ht_output_limit, ht_output_limit);
    {
        set.p = 0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0;
        set.t = send_t; // 正负没问题
    }
    return set;
}
/* 2 号电机 */
static ht_motor_para_t ht_control_2(ht_motor_measure_t measure)
{
    control_dt[1] = dwt_get_time_us() - control_start[1];
    control_start[1] = dwt_get_time_us();
    static ht_motor_para_t set;
    float send_t;

    // 每次上电归中电机给定一个适当的力矩，并持续，确保撞到限位
    if(chassis_cmd.ctrl_mode == CHASSIS_INIT)
    {
        send_t = -HT_INIT_OUT;
    }
    else
    {
        send_t = -vmc_out_r[FRONT] + jump_out_r[FRONT];
#ifdef CLOSE_ALL_MOTOR
        send_t = 0;
#endif
    }

    LIMIT_MIN_MAX(send_t, -ht_output_limit, ht_output_limit);
    {
        set.p = 0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0;
        set.t = send_t; // 正负没问题
    }
    return set;
}
/* 3 号电机 */
static ht_motor_para_t ht_control_3(ht_motor_measure_t measure)
{
    control_dt[2] = dwt_get_time_us() - control_start[2];
    control_start[2] = dwt_get_time_us();
    static ht_motor_para_t set;
    float send_t;

    // 每次上电归中电机给定一个适当的力矩，并持续，确保撞到限位
    if((chassis_cmd.ctrl_mode == CHASSIS_INIT) || (chassis_cmd.ctrl_mode == CHASSIS_RECOVERY))
    {
        send_t = HT_INIT_OUT;
    }
    else
    {
        send_t = -vmc_out_r[BACK] + jump_out_r[BACK];
#ifdef CLOSE_ALL_MOTOR
        send_t = 0;
#endif
    }

    LIMIT_MIN_MAX(send_t, -ht_output_limit, ht_output_limit);
    {
        set.p = 0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0;
        set.t = send_t; // 正负没问题
    }
    return set;
}
/* 4 号电机 */
static ht_motor_para_t ht_control_4(ht_motor_measure_t measure)
{
    control_dt[3] = dwt_get_time_us() - control_start[3];
    control_start[3] = dwt_get_time_us();
    static ht_motor_para_t set;
    float send_t;

    // 每次上电归中电机给定一个适当的力矩，并持续，确保撞到限位
    if((chassis_cmd.ctrl_mode == CHASSIS_INIT) || (chassis_cmd.ctrl_mode == CHASSIS_RECOVERY))
    {
        send_t = -HT_INIT_OUT;
    }
    else
    {
        send_t = vmc_out_l[BACK] + jump_out_l[BACK];
#ifdef CLOSE_ALL_MOTOR
        send_t = 0;
#endif
    }

    LIMIT_MIN_MAX(send_t, -ht_output_limit, ht_output_limit);
    {
        set.p = 0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0;
        set.t = send_t; // 正负没问题
    }
    return set;
}
/* 底盘每个电机对应的控制函数 */
static void *ht_control[4] =
        {
                ht_control_1,
                ht_control_2,
                ht_control_3,
                ht_control_4,
        };

static void ht_motor_init()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        motor_config_t ht_motor_config = {
                .motor_type = HT04,
                .can_id = 2,
                .tx_id = i+1,
                .rx_id = 0x10, // TODO
        };
        ht_motor[i] = ht_motor_register(&ht_motor_config, ht_control[i]);
    }
/*    motor_config_t ht_motor_config = {
            .motor_type = HT04,
            .can_name = CAN_CHASSIS,
            .tx_id = 1,
            .rx_id = 0x10, // TODO
    };
    ht_motor[0] = ht_motor_register(&ht_motor_config, ht_control[0]);
    ht_motor_config.tx_id = 2;
    ht_motor_config.can_name = CAN_GIMBAL,
    ht_motor[1] = ht_motor_register(&ht_motor_config, ht_control[1]);
    ht_motor_config.tx_id = 3;
    ht_motor_config.can_name = CAN_GIMBAL,
    ht_motor[2] = ht_motor_register(&ht_motor_config, ht_control[2]);
    ht_motor_config.tx_id = 4;
    ht_motor_config.can_name = CAN_CHASSIS,
    ht_motor[3] = ht_motor_register(&ht_motor_config, ht_control[3]);*/
}

static int16_t lk_control_l(lk_motor_measure_t measure){
    static int16_t set;

    LIMIT_MIN_MAX(LQROutBuf[LEFT][0], -lk_output_limit, lk_output_limit);
#ifndef CLOSE_ALL_MOTOR
    if(chassis_cmd.ctrl_mode == CHASSIS_INIT)
    {
        set = 0;
    }
    else
    {
        set = (int16_t)(LQROutBuf[LEFT][0] * LK_TOR_TO_CUR);
    }
#endif
    return set;
}

static int16_t lk_control_r(lk_motor_measure_t measure){
    static int16_t set;

    LIMIT_MIN_MAX(LQROutBuf[RIGHT][0], -lk_output_limit, lk_output_limit);
#ifndef CLOSE_ALL_MOTOR
    if(chassis_cmd.ctrl_mode == CHASSIS_INIT)
    {
        set = 0;
    }
    else
    {
        set = (int16_t)(LQROutBuf[RIGHT][0] * LK_TOR_TO_CUR);
    }
#endif
    return set;
}

static void *lk_control[2] =
        {
                lk_control_l,
                lk_control_r,
        };

static void lk_motor_init()
{
    motor_config_t motor_config = {
            .motor_type = MF9025,
            .can_id = 1,
            .tx_id = 0x141,
            .rx_id = 0x141,
    };
    lk_motor[RIGHT] = lk_motor_register(&motor_config, lk_control[RIGHT]);

    motor_config_t motor_config_2 = {
            .motor_type = MF9025,
            .can_id = 1,
            .tx_id = 0x142,
            .rx_id = 0x142,
    };
    lk_motor[LEFT] = lk_motor_register(&motor_config_2, lk_control[LEFT]);
}

/**
 * @brief 底盘初始化（注册底盘电机及其控制器初始化等）
 */
static int chassis_motor_init(void)
{
    ht_motor_init();

    leg_config_t leg_config =
            {
                    /*单位m*/
                    0.15f,  // l4=l1
                    0.250f, // l3=l2
                    0.11f   //电机间距
                    /* TODO: 改为宏定义 */
            };
    leg[LEFT] = leg_register(&leg_config);
    leg[RIGHT] = leg_register(&leg_config);

    //TODO
    lk_motor_init();

    pid_config_t length_pid_config = INIT_PID_CONFIG(800, 1, 200, 4, 300,
                                                   (PID_Integral_Limit | PID_DerivativeFilter));
    length_pid[LEFT] = pid_register(&length_pid_config);
    length_pid[RIGHT] = pid_register(&length_pid_config);

    // TODO: 两腿协调 PD 控制，航向角控制，横滚角补偿控制
    /* 两腿协调 PD 控制 */
//    pid_config_t theta_pid_config = INIT_PID_CONFIG(40, 0, 1, 0, 20, PID_Integral_Limit);
    pid_config_t theta_pid_config = INIT_PID_CONFIG(35, 0, 2, 0, 20, PID_Integral_Limit);
    theta_pid = pid_register(&theta_pid_config);
    /* 航向角 PD 控制 */
    /* 底盘开环参数 */
//    pid_config_t yaw_pid_config = INIT_PID_CONFIG(0.0002, 0, 0.0002, 0, 1, PID_Integral_Limit); // P: 0.004
//    pid_config_t yaw_pid_config = INIT_PID_CONFIG(0.0001, 0, 0.0003, 0, 1, PID_Integral_Limit); // P: 0.004
    pid_config_t yaw_pid_config = INIT_PID_CONFIG(0.00008, 0, 0.0005, 0, 1, PID_Integral_Limit); // P: 0.004
    /* 跟随云台参数 */
//    pid_config_t yaw_pid_config = INIT_PID_CONFIG(0.00005, 0, 0.0001, 0, 1, PID_Integral_Limit); // P: 0.004
    yaw_pid = pid_register(&yaw_pid_config);
    /* 横滚角 PD 控制 */
    pid_config_t roll_pid_config = INIT_PID_CONFIG(15, 0, 0.05, 0, ROLL_PID_MAXOUT, PID_Integral_Limit);
    roll_pid = pid_register(&roll_pid_config);

    return 0;
}

/* --------------------------------- 底盘解算控制 --------------------------------- */
#ifdef BSP_CHASSIS_OMNI_MODE
/**
 * @brief 全向轮底盘运动解算
 *
 * @param cmd cmd 底盘指令值，使用其中的速度
 * @param out_speed 底盘各轮速度
 */
static void omni_calc(struct chassis_cmd_msg *cmd, int16_t* out_speed)
{
    int16_t wheel_rpm[4];
    float wheel_rpm_ratio;

    wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * 3.14159f) * CHASSIS_DECELE_RATIO * 1000;

    //限制底盘各方向速度
    VAL_LIMIT(cmd->vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s
    VAL_LIMIT(cmd->vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
    VAL_LIMIT(cmd->vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s

    wheel_rpm[0] = ( cmd->vx + cmd->vy + cmd->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;//left//x，y方向速度,w底盘转动速度
    wheel_rpm[1] = ( cmd->vx - cmd->vy + cmd->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;//forward
    wheel_rpm[2] = (-cmd->vx - cmd->vy + cmd->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;//right
    wheel_rpm[3] = (-cmd->vx + cmd->vy + cmd->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;//back

    memcpy(out_speed, wheel_rpm, 4*sizeof(int16_t));//copy the rpm to out_speed
}
#endif /* BSP_CHASSIS_OMNI_MODE */

#ifdef BSP_CHASSIS_MECANUM_MODE

/**
 * @brief 麦克纳姆轮底盘运动解算
 *
 * @param cmd cmd 底盘指令值，使用其中的速度
 * @param out_speed 底盘各轮速度
 */
void mecanum_calc(struct chassis_cmd_msg *cmd, int16_t* out_speed)
{
    static float rotate_ratio_f = ((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF;
    static float rotate_ratio_b = ((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF;
    static float wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * CHASSIS_DECELE_RATIO);

    int16_t wheel_rpm[4];
    float max = 0;

    //限制底盘各方向速度
    VAL_LIMIT(cmd->vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s
    VAL_LIMIT(cmd->vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
    VAL_LIMIT(cmd->vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s

    wheel_rpm[0] = ( cmd->vx - cmd->vy + cmd->vw * rotate_ratio_f) * wheel_rpm_ratio;
    wheel_rpm[1] = ( cmd->vx + cmd->vy + cmd->vw * rotate_ratio_f) * wheel_rpm_ratio;
    wheel_rpm[2] = (-cmd->vx + cmd->vy + cmd->vw * rotate_ratio_b) * wheel_rpm_ratio;
    wheel_rpm[3] = (-cmd->vx - cmd->vy + cmd->vw * rotate_ratio_b) * wheel_rpm_ratio;

    memcpy(out_speed, wheel_rpm, 4 * sizeof(int16_t));
}
#endif /* BSP_CHASSIS_MECANUM_MODE */

#ifdef BSP_CHASSIS_LEG_MODE
static uint8_t isTouchingGround;
/**
 * @brief 轮腿底盘运动解算
 *
 * @param cmd cmd 底盘指令值，使用其中的速度
 * @param out_speed 底盘各轮力矩
 */
static void leg_calc()
{
    float len_pid_out;
    // 左腿解算
    leg[LEFT]->input_leg_angle(leg[LEFT], /*-0.42*/(ht_motor[LEFT_BACK]->measure.total_angle - HT_OFFSET_LB), /*3.6*/PI + (ht_motor[LEFT_FRONT]->measure.total_angle - HT_OFFSET_LF));
    leg[LEFT]->resolve(leg[LEFT]);
    leg[LEFT]->get_leg_spd(leg[LEFT], ht_motor[LEFT_FRONT]->measure.speed_rads, ht_motor[LEFT_BACK]->measure.speed_rads);

    // 右腿解算
    leg[RIGHT]->input_leg_angle(leg[RIGHT], -(ht_motor[RIGHT_BACK]->measure.total_angle - HT_OFFSET_RB),  PI - (ht_motor[RIGHT_FRONT]->measure.total_angle - HT_OFFSET_RF));
    leg[RIGHT]->resolve(leg[RIGHT]);
    leg[RIGHT]->get_leg_spd(leg[RIGHT], -ht_motor[RIGHT_FRONT]->measure.speed_rads, -ht_motor[RIGHT_BACK]->measure.speed_rads);

    update_LQR_obs();
    LQR_cal();

    /* 双腿角度协调控制 */
    pid_calculate(theta_pid, leg[LEFT]->PendulumRadian - leg[RIGHT]->PendulumRadian, 0);
    /* 航向角控制 */
//    pid_calculate(yaw_pid, ins.yaw_total_angle, yaw_target);

    pid_calculate(yaw_pid, ins.yaw_total_angle, yaw_target);


//    pid_calculate(yaw_pid, chassis_cmd.offset_angle, 0);
    /* 横滚角控制 */
    pid_calculate(roll_pid, ins.roll, chassis_cmd.vy + ROLL_OFFSET);

    len_pid_out = pid_calculate(length_pid[LEFT], leg[LEFT]->PendulumLength, leg[LEFT]->length_ref);
    LIMIT_MIN_MAX(len_pid_out, -300, 300);
    //根据离地状态计算左右腿推力，若离地则不考虑roll轴PID输出和前馈量
//    leftForce = legLengthPID.output + ((groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? 6-rollPID.output : 0);
    ft_l[0] = len_pid_out + roll_pid->Output + len_feed;
    LIMIT_MIN_MAX(ft_l[0], -FORCE_LIMIT, FORCE_LIMIT);
    ft_l[1] = LQROutBuf[LEFT][1] + theta_pid->Output;

    len_pid_out = pid_calculate(length_pid[RIGHT], leg[RIGHT]->PendulumLength, leg[RIGHT]->length_ref);
    LIMIT_MIN_MAX(len_pid_out, -300, 300);
    ft_r[0] = len_pid_out - roll_pid->Output + len_feed;
    LIMIT_MIN_MAX(ft_r[0], -FORCE_LIMIT, FORCE_LIMIT);
    ft_r[1] = LQROutBuf[RIGHT][1] - theta_pid->Output;

    /* 离地检测，计算两腿地面支持力 */
//    TODO：目前离地检测还存在问题 ins.acc 存在问题，可能需要卡尔曼滤波
    leg[LEFT]->support_force = ft_l[0] + LEG_MASS * 9.8f + LEG_MASS * (ins.motion_accel_b[2] - leg[LEFT]->dd_l0);
    leg[RIGHT]->support_force = ft_r[0] + LEG_MASS * 9.8f + LEG_MASS * (ins.motion_accel_b[2] - leg[RIGHT]->dd_l0);
    isTouchingGround = (leg[LEFT]->support_force < 15) && (leg[RIGHT]->support_force < 15); //判断当前瞬间是否接地

    leg[LEFT]->VMC_cal(leg[LEFT], ft_l, vmc_out_l);
    leg[RIGHT]->VMC_cal(leg[RIGHT], ft_r, vmc_out_r);
}

/**
 * @brief 底盘跳跃处理
 * @note 进入跳跃模式时记录当前时间，并将jump输出力矩加上跳跃下压力矩值，持续0.2s,开始跳跃0，15s后加上跳跃回缩力矩值，持续0.4s,落地后跳跃结束
 */
// TODO: 结合离地检测进行优化（进阶玩法：空中轮子充当动量轮调整姿态
// TODO: 跳跃力矩和持续时间还需继续优化
// TODO: 跳跃前需要增加机体pitch轴范围判断，或起跳前通过改写k矩阵，使得机体pitch轴更快收敛，充分准备起跳，减少空中发撒
// TODO：目前通过多状态量判断跳跃是否结束，以及切入跳跃模式仅跳跃一次，需要进一步优化，可以结合cmd线程
static void jumping_control(void)
{
    float current_time = dwt_get_time_ms();
    float dt;
    if(chassis_cmd.last_mode != CHASSIS_JUMP)
    {
        jump_flag = 1;
    }

    if(jump_flag)
    {

    if(!is_jumping)
    {
        is_jumping = 1;
        jump_start_time = current_time;
    }
    dt = current_time - jump_start_time;

    jump_out_l[FRONT] = -JUMP_TORQUE_PRESS * (dt<200) + JUMP_TORQUE_SHRINK * (dt>150 && dt<300);
    jump_out_l[BACK]  =  JUMP_TORQUE_PRESS * (dt<200) - JUMP_TORQUE_SHRINK * (dt>150 && dt<300);
    jump_out_r[FRONT] =  JUMP_TORQUE_PRESS * (dt<200) - JUMP_TORQUE_SHRINK * (dt>150 && dt<300);
    jump_out_r[BACK]  = -JUMP_TORQUE_PRESS * (dt<200) + JUMP_TORQUE_SHRINK * (dt>150 && dt<300);

    if (dt > 400)
    {
        is_jumping = 0;  // 跳跃结束
        jump_flag = 0;  // 跳跃结束
    }
    }
}
#endif /* BSP_CHASSIS_MECANUM_MODE */

/**
 * @brief  根据当前腿长限幅加速度,添加斜坡
 * @param  vx_cmd: 期望速度
 * @param  vx_now: 当前速度
 */
static float vx_change_limit(float vx_cmd, float vx_now)
{
    //根据当前腿长计算速度斜坡步长(腿越短越稳定，加减速斜率越大)
    float legLength = (leg[LEFT]->PendulumLength + leg[RIGHT]->PendulumLength) / 2;
    float speed_step = legLength * (-2.5) + 1;
    float vx_change = vx_cmd - vx_now;
    float vx_limited; // 限制后的速度,返回值

    //计算速度斜坡，斜坡值更新到target.speed
    if(fabsf(vx_change) < fabsf(speed_step))
    {
        vx_limited = vx_cmd;
    }
    else
    {
        if(vx_change > 0)
            vx_limited = vx_now + speed_step;
        else
            vx_limited = vx_now - speed_step;
    }

    return vx_limited;
}

/**
  * @brief  将云台坐标转换为底盘坐标
  * @param  cmd 底盘指令值，使用其中的速度
  * @param  angle 云台相对于底盘的角度
  */
static void absolute_cal(struct chassis_cmd_msg *cmd, float angle)
{
    float angle_hd = -(angle/RADIAN_COEF);
    float vx = cmd->vx;
    float vy = cmd->vy;

    //保证底盘是相对摄像头做移动
    cmd->vx = vx * cos(angle_hd) + vy * sin(angle_hd);
    //cmd->vy = -vx * sin(angle_hd) + vy * cos(angle_hd);
    // 平步可执行 vy 为 0
    cmd->vy = 0;
}

/**
 * @brief chassis 线程中所有订阅者初始化
 */
static void chassis_sub_init(void)
{
    ins_topic_node = mcn_subscribe(MCN_HUB(ins_topic), NULL, NULL);
    chassis_cmd_node = mcn_subscribe(MCN_HUB(chassis_cmd), NULL, NULL);
}

/**
 * @brief chassis 线程中所有发布者推送更新话题
 */
static void chassis_pub_push(void)
{
    mcn_publish(MCN_HUB(chassis_fdb), &chassis_fdb_data);
}

/**
 * @brief chassis 线程中所有订阅者获取更新话题
 */
static void chassis_sub_pull(void)
{
    if (mcn_poll(ins_topic_node))
    {
        mcn_copy(MCN_HUB(ins_topic), ins_topic_node, &ins);
    }

    if (mcn_poll(chassis_cmd_node))
    {
        mcn_copy(MCN_HUB(chassis_cmd), chassis_cmd_node, &chassis_cmd);
    }
}
