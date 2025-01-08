/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-30     xph    first version
 */

#ifndef _SENSOR_TI_INA226__H
#define _SENSOR_TI_INA226__H
#include <rtthread.h>
#include <rtdevice.h>
#if defined(RT_VERSION_CHECK)
    #if (RTTHREAD_VERSION >= RT_VERSION_CHECK(5, 0, 2))
        #define RT_SIZE_TYPE   rt_ssize_t
    #else
        #define RT_SIZE_TYPE   rt_size_t
    #endif
#endif
#include "rtthread.h"
#include "board.h"
#include "rtdevice.h"


typedef struct _rt_ina226_device_t
{
    struct rt_i2c_bus_device *bus;
    rt_uint8_t addr;
    
}rt_ina226_device_t;

typedef struct _power_monitor_data_t
{
    float ma;
    float mv;
    float mw;
}power_monitor_data_t;

#define RT_SENSOR_CTRL_GET_POWER_MONITOR_DATA    (RT_SENSOR_CTRL_USER_CMD_START+1)

int rt_hw_ina226_init(const char *name, struct rt_sensor_config *cfg);

#endif
















