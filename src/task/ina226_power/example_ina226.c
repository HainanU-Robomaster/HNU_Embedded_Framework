/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-30     xph    first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include "example_ina226.h"
#include "rm_module.h"
//#include "ti_ina226_sensor_v1.h"
//#include "ina226.h"



/****************************
 *         A0   A1  addr
 *         0    0   0x40
 *         0    1   0x41
 *         1    0   0x44
 *         1    1   0x45
 *************************/
#define INA226_ADDR (0x40)
power_monitor_data_t power_monitor_data;
float volts,volts1,current;
void ina226_thread_entry(void *argument)
{
    rt_device_t dev = RT_NULL;
    struct rt_sensor_data sensor_data;
    char ma[10],mv[10],mw[10];


    rt_size_t res;
    dev = rt_device_find("i2c2");
    if (dev == RT_NULL)
    {
        rt_kprintf("can not find device : ina226 \n");
        return;
    }

    if (rt_device_open(dev, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("open device failed!\n");
        return;
    }

    while (1)
    {
        res = RT_EOK;
        if (res != RT_EOK)
        {
            rt_kprintf("read data failed! result is %d\n", res);
            rt_device_close(dev);
            return;
        }
        else
        {
            power_monitor_data_t *hData = (power_monitor_data_t *)&power_monitor_data;
            ina226_get_current(&(hData->ma));
            ina226_get_bus_voltage(&(hData->mv));
            hData->mw = hData->mv * hData->ma / 1e3;
            res = RT_EOK;
        }
        rt_thread_mdelay(1);
    }
}



static int rt_hw_ina226_port(void)
{
    struct rt_sensor_config cfg;
    cfg.intf.user_data = (void *)INA226_ADDR;
    cfg.intf.dev_name = "i2c2";
    cfg.mode = RT_SENSOR_MODE_POLLING  ;
    rt_hw_ina226_init("ina226", &cfg);
    return RT_EOK;
}

INIT_COMPONENT_EXPORT(rt_hw_ina226_port);
