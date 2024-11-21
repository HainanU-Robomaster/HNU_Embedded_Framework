/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-30     xph    first version
 */

#ifndef _INA226__H
#define _INA226__H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"
#include "board.h"
#include "rtdevice.h"

// INA226 I2C address
#define INA226_I2C_ADDR (0x40)

// Registers
#define INA226_REG_CONFIG (0x00)
#define INA226_REG_MANID (0xfe)
#define INA226_REG_DIEID (0xff)
#define INA226_REG_SHUNTVOLTS (0x01)
#define INA226_REG_BUSVOLTS (0x02)
#define INA226_REG_POWER (0x03)
#define INA226_REG_CURRENT (0x04)
#define INA226_REG_CALIB (0x05)
#define INA226_REG_MANID (0xfe)
#define INA226_REG_DIEID (0xff)


#define  INA226_CONF_MODE_OFF                 (0)
#define  INA226_CONF_MODE_SINGLE_SHUNT        (1)
#define  INA226_CONF_MODE_SINGLE_BUS          (2)
#define  INA226_CONF_MODE_SINGLE_SHUNT_BUS    (3)
#define  INA226_CONF_MODE_POWERDOWN           (4)
#define  INA226_CONF_MODE_CONT_SHUNT          (5)
#define  INA226_CONF_MODE_CONT_BUS            (6)
#define  INA226_CONF_MODE_CONT_SHUNT_BUS      (7)

#define  INA226_DEFAULT_SHUNT_OHMS            (0.01)  // Ohms
#define  INA226_CURRENT_LSB                   (0.02) // mA

#define INA226_STATUS_OK (0U)
#define INA226_STATUS_ERROR (-1U)

// convert value at addr to little-endian (16-bit)
#define __LEu16(addr)                            \
  (((((uint16_t)(*(((uint8_t *)(addr)) + 1)))) | \
    (((uint16_t)(*(((uint8_t *)(addr)) + 0))) << 8U)))

// convert value at addr to little-endian (32-bit)
#define __LEu32(addr)                                   \
  (((((uint32_t)(*(((uint8_t *)(addr)) + 3)))) |        \
    (((uint32_t)(*(((uint8_t *)(addr)) + 2))) << 8U) |  \
    (((uint32_t)(*(((uint8_t *)(addr)) + 1))) << 16U) | \
    (((uint32_t)(*(((uint8_t *)(addr)) + 0))) << 24U)))

  enum ina226_conf_avg
  {
    AVG_1 = 0, // Default
    AVG_4 = 1,
    AVG_16 = 2,
    AVG_64 = 3,
    AVG_128 = 4,
    AVG_256 = 5,
    AVG_512 = 6,
    AVG_1024 = 7
  };

  enum ina226_conf_ct
  {
    CT_140US = 0,
    CT_204US = 1,
    CT_332US = 2,
    CT_588US = 3,
    CT_1100US = 4, // Default
    CT_2116US = 5,
    CT_4156US = 6,
    CT_8244US = 7,
  };

  typedef int (*i2c_func)(uint8_t addr, uint16_t *data, uint8_t data_len);

  typedef struct _ina226_device_t
  {
    i2c_func i2c_read;
    i2c_func i2c_write;
    uint8_t addr;
    float shunt_resistance;
  } ina226_device_t;

int ina226_init(i2c_func i2c_read, i2c_func i2c_write, uint8_t addr);
int  ina226_get_bus_voltage(float *volts);
int  ina226_get_shunt_voltage(float *volts);
int  ina226_get_current(float *ampere);
int  ina226_get_current1(float *ampere);
int ina226_get_power(float *power);
int  ina226_set_shunt_resistance(float ohms);
int ina226_get_shunt_resistance(float *ohms);

#ifdef __cplusplus
}
#endif

#endif
