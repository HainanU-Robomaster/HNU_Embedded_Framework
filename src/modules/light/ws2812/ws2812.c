//
// Created by arcueid on 25-1-9.
//

#include "ws2812.h"
#include <drv_spi.h>
#include "rm_config.h"
#include "light.h"

#define DBG_TAG   "light.ws2812"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* ------------------------------- 灯板LED控制 ------------------------------ */
static struct rt_spi_device* led_dev;  /*灯板ledspi设备*/
static rt_err_t ws2812_init();
static void ws2812_update();
static void ws2812_creat_data(uint8_t R, uint8_t G, uint8_t B);
static rt_err_t ws2812_write_rgb_to_node(uint32_t color, uint16_t Pos);
static rt_err_t ws2812_clear_rgb_to_node(uint16_t Pos);
static rt_err_t ws2812_clear_rgb_to_all();
static void cmd_rgb_control(int argc, char **argv);




static uint8_t buffer[RGB_BIT*LIGHT_NUM];
static uint8_t RGB_BIT_Buffer[RGB_BIT];


/**
 * @brief  LED灯条初始化，默认上电全部熄灭
 */
static rt_err_t ws2812_init(){

    rt_hw_spi_device_attach(SPI_LED, SPI_LED_DEVICE, GET_PIN(B,12)/*, GPIO_PIN_12*/);
    led_dev=(struct rt_spi_device *) rt_device_find(SPI_LED_DEVICE);
    if(led_dev == RT_NULL)
    {
        LOG_E("Can't find cover servo pwm device!");
        return RT_ERROR;
    }
    struct rt_spi_configuration ws2812_spi_config;

    ws2812_spi_config.mode=RT_SPI_MASTER | RT_SPI_MODE_3 | RT_SPI_MSB;
    ws2812_spi_config.max_hz=5.25*1000*1000;
    ws2812_spi_config.data_width=8;

    led_dev->config.max_hz=ws2812_spi_config.max_hz;
    led_dev->config.mode=ws2812_spi_config.mode;
    led_dev->config.data_width=ws2812_spi_config.data_width;

    if(led_dev->bus->owner=RT_NULL){

    }
//    led_dev->bus->owner=led_dev;
    rt_spi_configure((struct rt_spi_device *) led_dev,&ws2812_spi_config);

    memset((void *)buffer, WS2812_0, sizeof(buffer));
    memset((void *)RGB_BIT_Buffer, WS2812_0, sizeof(RGB_BIT_Buffer));
    ws2812_clear_rgb_to_all();

    return RT_EOK;
}

/**
 * @brief  更新led灯条函数,在ws2812_write_rgb_to_node或ws2812_clear_rgb_to_node之后调用
 */
static void ws2812_update()
{
    rt_spi_send(led_dev, buffer, RGB_BIT * LIGHT_NUM);
}
/**
 * @brief  点亮灯条上指定位置LED
 * @param  color LED颜色
 * @param  Pos   LED位置,0~LIGHT_NUM-1
 */
static rt_err_t ws2812_write_rgb_to_node(uint32_t color, uint16_t Pos)
{
    uint8_t R, G, B;
    uint16_t i;

    R = (color >> 16) & 0x00FF;
    G = (color >> 8) & 0x0000FF;
    B = (color) & 0x0000FF;

    ws2812_creat_data(R, G, B);
    if (Pos < LIGHT_NUM && Pos >= 0)
    {
        memcpy((void *)(buffer + RGB_BIT * Pos), (void *)RGB_BIT_Buffer, RGB_BIT);
        ws2812_update();

        return RT_EOK;
    }
    else
    {
        rt_kprintf("Error: Pos is out of range.\n");

        return RT_ERROR;
    }
}
/**
 * @brief  熄灭灯条上指定位置LED
 * @param  Pos LED位置,0~LIGHT_NUM-1
 */
static rt_err_t ws2812_clear_rgb_to_node(uint16_t Pos)
{
    if (Pos < LIGHT_NUM && Pos >= 0)
    {
        memset((void *)(buffer + RGB_BIT * Pos), WS2812_0, RGB_BIT);
        ws2812_update();
        return RT_EOK;
    }
    else
        return RT_ERROR;
}
/**
 * @brief  熄灭灯条上所有LED
 */
static rt_err_t ws2812_clear_rgb_to_all()
{
    for (uint16_t i = 0; i < LIGHT_NUM; i++)
    {
        ws2812_clear_rgb_to_node(i);
    }
    ws2812_update();
    return RT_EOK;
}

static void ws2812_creat_data(uint8_t R, uint8_t G, uint8_t B) {
    uint8_t temp[RGB_BIT] = {0};
    for (uint8_t i = 0; i < 8; i++) {
        temp[7 - i] = (G & 0x01) ? WS2812_1 : WS2812_0;
        G = G >> 1;
    }
    for (uint8_t i = 0; i < 8; i++) {
        temp[15 - i] = (R & 0x01) ? WS2812_1 : WS2812_0;
        R = R >> 1;
    }
    for (uint8_t i = 0; i < 8; i++) {
        temp[23 - i] = (B & 0x01) ? WS2812_1 : WS2812_0;
        B = B >> 1;
    }
    memcpy((void *) RGB_BIT_Buffer, temp, RGB_BIT);
}

struct light_ops light_ops = {
        .light_init=ws2812_init,
        .light_clear_rgb_to_all=ws2812_clear_rgb_to_all,
        .light_clear_rgb_to_node=ws2812_clear_rgb_to_node,
        .light_write_rgb_to_node=ws2812_write_rgb_to_node,
};