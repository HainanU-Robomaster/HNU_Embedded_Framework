#ifndef _MYIIC__H
#define _MYIIC__H

#include "main.h"

#define IIC_SCL_PORT    GPIOB
#define IIC_SCL_PIN     GPIO_PIN_13

#define IIC_SDA_PORT    GPIOB
#define IIC_SDA_PIN     GPIO_PIN_12

#define IIC_SCL_H HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_SET)   //SCL H
#define IIC_SCL_L HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_RESET) //SCL H
#define IIC_SDA_H HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_SET)   //SDA_H
#define IIC_SDA_L HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_RESET) //SDA_L
#define READ_SDA HAL_GPIO_ReadPin(IIC_SDA_PORT, IIC_SDA_PIN)                   //输入SDA
#define SDA_READ IIC_SDA_PORT->IDR &GPIO_IDR_IDR2

extern uint8_t i2c_addr;

void i2c_init_port(void);
int i2c_read(uint8_t reg, uint16_t *data, uint8_t data_len);
int i2c_write(uint8_t reg, uint16_t *data, uint8_t data_len);



#endif
