/*
* Change Logs:
* Date            Author          Notes
* 2023-09-05      ChuShicheng     first version
*/

#include "example_task.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"

#define DBG_TAG   "rm.task"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>
#define YAW_MOTOR 0
#define PITCH_MOTOR 1
static struct gimbal_controller_t{
    pid_obj_t *speed_pid;
    // pid_obj_t *angle_pid;
}gimbal_controlelr[3];
static dji_motor_object_t *shoot_motor1;
static dji_motor_object_t *shoot_motor2;
static dji_motor_object_t *shoot_motor3;
// static dji_motor_object_t *shoot_motor4;
static float shoot_motor_ref[3];
// static float reflection;

static rt_int16_t shoot_control1(dji_motor_measure_t measure){
    static rt_int16_t set = 0;
    set = pid_calculate(gimbal_controlelr[0].speed_pid, measure.speed_rpm,shoot_motor_ref[0] );
    return set;
}
static rt_int16_t shoot_control2(dji_motor_measure_t measure){
    static rt_int16_t set = 0;
    set = pid_calculate(gimbal_controlelr[1].speed_pid, measure.speed_rpm, shoot_motor_ref[1]);
    return set;
}
static rt_int16_t shoot_control3(dji_motor_measure_t measure){
    static rt_int16_t set = 0;
    set = pid_calculate(gimbal_controlelr[2].speed_pid, measure.speed_rpm, shoot_motor_ref[2]);
    return set;
}
//
// static rt_int16_t shoot_control4(dji_motor_measure_t measure){
//     static rt_int16_t set = 0;
//     set = pid_calculate(gimbal_controlelr[3].speed_pid, measure.speed_rpm, shoot_motor_ref[3]);
//     return set;
// }

static void example_init()
{
    pid_config_t shoot1_speed_config = {
            .Kp = 10, // 4.5
            .Ki = 0,  // 0
            .Kd = 0,  // 0
            .IntegralLimit = 3000,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            .MaxOut = 12000,
    };
    pid_config_t shoot2_speed_config = {
        .Kp = 10, // 4.5
        .Ki = 0,  // 0
        .Kd = 0,  // 0
        .IntegralLimit = 3000,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .MaxOut = 12000,
};
    pid_config_t shoot3_speed_config = {
        .Kp = 10, // 4.5
        .Ki = 0,  // 0
        .Kd = 0,  // 0
        .IntegralLimit = 3000,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .MaxOut = 12000,
};
//     pid_config_t shoot4_speed_config = {
//         .Kp = 10, // 4.5
//         .Ki = 0,  // 0
//         .Kd = 0,  // 0
//         .IntegralLimit = 3000,
//         .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
//         .MaxOut = 12000,
// };

    gimbal_controlelr[YAW_MOTOR].speed_pid = pid_register(&shoot1_speed_config);
    gimbal_controlelr[PITCH_MOTOR].speed_pid = pid_register(&shoot2_speed_config);
    gimbal_controlelr[2].speed_pid = pid_register(&shoot3_speed_config);
    // gimbal_controlelr[3].speed_pid = pid_register(&shoot4_speed_config);

    motor_config_t shoot1_motor_config = {
            .motor_type = M3508,
            .can_name = CAN_CHASSIS,
            .rx_id = 0x208,
            .controller = &gimbal_controlelr[0],
    };
    motor_config_t shoot2_motor_config = {
        .motor_type = M2006,
        .can_name = CAN_CHASSIS,
        .rx_id = 0x207,
        .controller = &gimbal_controlelr[1],
};
    motor_config_t shoot3_motor_config = {
        .motor_type = M2006,
        .can_name = CAN_CHASSIS,
        .rx_id = 0x206,
        .controller = &gimbal_controlelr[2],
};

    shoot_motor1 = dji_motor_register(&shoot1_motor_config, shoot_control1);
    shoot_motor2 = dji_motor_register(&shoot2_motor_config, shoot_control2);
    shoot_motor3 = dji_motor_register(&shoot3_motor_config,shoot_control3);

}

void example_thread_entry(void *argument)
{
    static float example_dt;
    static float example_start;
    static float ref_yaw,ref_pitch,ref_load;
    ref_yaw=500;
    ref_pitch=500;
    ref_load=500;

    example_init();
    LOG_I("Example Task Start");
    for (;;)
    {
        example_start = dwt_get_time_ms();
        // for(int i=0;i<4;i++) {
        //     shoot_motor_ref[i]=reflection;
        // }
        // reflection= 100;
        shoot_motor_ref[YAW_MOTOR]=ref_pitch;
        shoot_motor_ref[PITCH_MOTOR]=ref_yaw;//+:back(high),-:toward,(high)
        shoot_motor_ref[2]=ref_load;
        // shoot_motor_ref[2]=-1000;
        // shoot_motor_ref[3]=1000;



        example_dt = dwt_get_time_ms() - example_start;
        if (example_dt > 1)
            LOG_E("Example Task is being DELAY! dt = [%f]", &example_dt);


        rt_thread_delay(1);
    }
}