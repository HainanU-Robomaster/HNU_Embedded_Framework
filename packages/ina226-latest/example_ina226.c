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
#include "ti_ina226_sensor_v1.h"

#include "stdio.h"
#include "ina226.h"



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
static void ina226_thread_entry(void *parameter)
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
            // ina226_get_power(&(hData->mw));
            hData->mw = hData->mv * hData->ma / 1e3;
            res = RT_EOK;
         //   ina226_get_bus_voltage(&volts);
           // ina226_get_shunt_voltage(&volts1);
            ina226_get_current1(&current);


//            sprintf(ma,"%f", power_monitor_data.ma);
//            sprintf(mv,"%f", power_monitor_data.mv);
//            sprintf(mw,"%f", power_monitor_data.mw);
            rt_kprintf("current : %s mA,voltage : %s V ,power : %s mW\n", ma, mv, mw);
        }
        rt_thread_mdelay(1);
    }
}

static int ina226_example(void)
{
    rt_thread_t ina226_thread;
    ina226_thread = rt_thread_create(
            "ina226",
            ina226_thread_entry,
            RT_NULL,
            1024,
            RT_THREAD_PRIORITY_MAX / 2,
            10);
    if (ina226_thread != RT_NULL)
    {
        rt_thread_startup(ina226_thread);
    }
    else
    {
        return -RT_ERROR;
    }
    return RT_EOK;
}

INIT_APP_EXPORT(ina226_example);

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
