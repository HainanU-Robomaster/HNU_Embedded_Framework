/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-02-02     xph    first version
 */

#include "ina226.h"

static ina226_device_t ina226_device = {0};


int ina226_init(i2c_func i2c_read, i2c_func i2c_write, uint8_t addr)
{
  uint16_t val = 0x8000;
  uint16_t man_id = 0, die_id = 0;
  ina226_device.i2c_read = i2c_read;
  ina226_device.i2c_write = i2c_write;
  ina226_device.addr = addr;
  
  i2c_read(INA226_REG_MANID, &man_id, 2);
  i2c_read(INA226_REG_DIEID, &die_id, 2);
  man_id = __LEu16(&man_id);
  die_id = __LEu16(&die_id);
  if (man_id != 0x5449)
  {
    return INA226_STATUS_ERROR;
  }
  if ((die_id & 0xfff0) != 0x2260)
  {
    return INA226_STATUS_ERROR;
  }
  // reset device
  val = __LEu16(&val);
  i2c_write(INA226_REG_CONFIG,&val, 2U);
  rt_thread_mdelay(20);
  val = 0;
  // Set config register:
  val = AVG_64 << 9;      // Samples
  val |= CT_1100US << 6; // Bus conversion time
  val |= CT_1100US << 3; // Shunt conversion time
  val |=  INA226_CONF_MODE_CONT_SHUNT_BUS;
  val = __LEu16(&val);
  i2c_write(INA226_REG_CONFIG,&val, 2U);
  ina226_set_shunt_resistance(INA226_DEFAULT_SHUNT_OHMS);
  val = (uint16_t)(5.12f / (INA226_DEFAULT_SHUNT_OHMS * INA226_CURRENT_LSB));
	val = __LEu16(&val);
  i2c_write(INA226_REG_CALIB, &val, 2U);
  return INA226_STATUS_OK;
}

int  ina226_get_bus_voltage(float *volts)
{
  int16_t val = 0;
  ina226_device.i2c_read(INA226_REG_BUSVOLTS, (uint16_t *)&val, 2U);
  val = __LEu16(&val);
  // 理论值是1.25 但是需要结合实际情况来调节
  *volts = val * 1.176 ; // 1.25mV per LSB 
  return INA226_STATUS_OK;
}

int  ina226_get_shunt_voltage(float *volts)
{
  int16_t val = 0 ;
  ina226_device.i2c_read(INA226_REG_SHUNTVOLTS, (uint16_t *)&val, 2U);
  val = __LEu16(&val);
  *volts = val * 2.5 / 1e3; // 2.5uV per LSB
  return INA226_STATUS_OK;
}
/**
 * 读取寄存器来获取电流值，这种方式是芯片内部自己计算电流
*/
int  ina226_get_current(float *ampere)
{
  int16_t val = 0 ;
  ina226_device.i2c_read(INA226_REG_CURRENT, (uint16_t *)&val, 2U);
  val = __LEu16(&val);
  // 理论值是*1.00 但是需要结合实际情况来校准
  *ampere = val * INA226_CURRENT_LSB * 1.1705;
  return INA226_STATUS_OK;
}

/**
 * 读取采样电阻的电压值，结合采样电阻值，自己计算采样电流
*/
int  ina226_get_current1(float *ampere)
{
  float shunt_volts = 0;
  if (ina226_get_shunt_voltage(&shunt_volts) != INA226_STATUS_OK)
  {
    return INA226_STATUS_ERROR;
  }
  // 理论值是*1.00 但是需要结合实际情况来校准
  *ampere = (shunt_volts  / ina226_device.shunt_resistance ) * 1.1705  ;
  return INA226_STATUS_OK;
}

int ina226_get_power(float *power)
{
  int16_t val = 0 ;
  ina226_device.i2c_read(INA226_REG_POWER, (uint16_t *)&val, 2U);
  val = __LEu16(&val);
  *power = val * INA226_CURRENT_LSB * 25;
  return INA226_STATUS_OK;
}

int  ina226_set_shunt_resistance(float ohms)
{
  ina226_device.shunt_resistance = ohms;
  return INA226_STATUS_OK;
}

int ina226_get_shunt_resistance(float *ohms)
{
  *ohms = ina226_device.shunt_resistance;
  return INA226_STATUS_OK;
}


