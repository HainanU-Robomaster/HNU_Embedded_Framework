//
// Created by arcueid on 25-1-9.
//

#ifndef RTTHREAD_LIGHT_H
#define RTTHREAD_LIGHT_H

#include <rtthread.h>

#ifdef BSP_USING_WS2812
#include "ws2812.h"
#endif /* BSP_USING_WS2812 */

//对应颜色宏定义在rm_config.h结尾
struct light_ops{
    rt_err_t (*light_init)(void);                                               //初始化
    rt_err_t (*light_write_rgb_to_node)(uint32_t color, uint16_t Pos);          //点亮指定位置灯,color:亮灯颜色,Pos:亮灯位置,0~LIGHT_NUM
    rt_err_t (*light_clear_rgb_to_node)(uint16_t Pos);                          //熄灭指定位置灯，Pos：灭灯位置
    rt_err_t (*light_clear_rgb_to_all)(void);                                   //熄灭所有灯
};

#endif //RTTHREAD_LIGHT_H
