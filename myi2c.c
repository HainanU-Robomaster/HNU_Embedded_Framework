#include "myi2c.h"

uint8_t i2c_addr;

static void delay_nns(uint16_t ms) //30纳秒ns  根据手册要用到IIC的HS高速模式
{
  while (--ms)
  {
    /* code */
  }
  
}

// static void delay_nms(uint16_t ms) //毫秒
// {
//   uint16_t i;
//   uint32_t M = 0; //720W
//   for (i = 0; i < ms; i++)
//     for (M = 12000; M > 0; M--)
//       ;
// }

// static void delay_nus(uint16_t us) //微秒
// {
//   uint16_t i;
//   uint16_t M = 0; //720W
//   for (i = 0; i < us; i++)
//     for (M = 72; M > 0; M--)
//       ;
// }

void i2c_init_port(void)
{
  GPIO_InitTypeDef GPIO_Initure;
  __HAL_RCC_GPIOB_CLK_ENABLE(); //使能GPIOA时钟

  GPIO_Initure.Pin = IIC_SDA_PIN | IIC_SCL_PIN; //
  GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;      //推挽输出
  GPIO_Initure.Pull = GPIO_PULLUP;              //上拉
  GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;    //高速
  HAL_GPIO_Init(IIC_SCL_PORT, &GPIO_Initure);

  IIC_SDA_H;
  IIC_SCL_H;
}

static void sda_in(void)
{
  GPIO_InitTypeDef GPIO_Initure;
  GPIO_Initure.Pin = IIC_SDA_PIN;            //
  GPIO_Initure.Mode = GPIO_MODE_INPUT;       //输入
  GPIO_Initure.Pull = GPIO_PULLUP;           //上拉
  GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH; //高速
  HAL_GPIO_Init(IIC_SDA_PORT, &GPIO_Initure);
}

static void sda_out(void)
{
  GPIO_InitTypeDef GPIO_Initure;
  GPIO_Initure.Pin = IIC_SDA_PIN;            //
  GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;   //输入
  GPIO_Initure.Pull = GPIO_PULLUP;           //上拉
  GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH; //高速
  HAL_GPIO_Init(IIC_SDA_PORT, &GPIO_Initure);
}

static void i2c_start(void)
{
  sda_out();
  IIC_SDA_H;
  IIC_SCL_H;
  delay_nns(5);
  IIC_SDA_L; //START:when CLK is high,DATA change form high to low
  delay_nns(5);
  IIC_SCL_L; //钳住I2C总线，准备发送或接收数据
  delay_nns(5);
}

static void i2c_stop(void)
{
  sda_out();
  IIC_SCL_L;
  IIC_SDA_L; //STOP:when CLK is high DATA change form low to high
  delay_nns(5);
  IIC_SCL_H;
  delay_nns(5);
  IIC_SDA_H; //发送I2C总线结束信号
  delay_nns(5);
}

static void i2c_ack(void)
{
  sda_out();
  IIC_SDA_L;
  delay_nns(5);
  IIC_SCL_H;
  delay_nns(5);
  IIC_SCL_L;
  delay_nns(5);
  IIC_SDA_H;
}

static void i2c_nack(void)
{
  sda_out();
  IIC_SDA_H;
  delay_nns(5);
  IIC_SCL_H;
  delay_nns(5);
  IIC_SCL_L;
  delay_nns(5);
  IIC_SDA_L;
}

static uint8_t i2c_wait_ack(void)
{
  uint8_t ucErrTime = 0;
  sda_in();
  IIC_SDA_H;
  delay_nns(5);
  IIC_SCL_H;
  delay_nns(5);
  while (READ_SDA)
  {
    ucErrTime++;
    delay_nns(1);
    if (ucErrTime > 250)
    {
      i2c_stop();
      return 1;
    }
  }
  IIC_SCL_L; //时钟输出0
  return 0;
}

static void i2c_send_byte(uint8_t txd)
{
  sda_out();
  IIC_SCL_L; //拉低时钟开始数据传输
  for (uint8_t i = 0; i < 8; i++)
  {
    if (txd & 0x80)
      IIC_SDA_H;
    else
      IIC_SDA_L;
    txd <<= 1;

    IIC_SCL_H;
    delay_nns(5);
    IIC_SCL_L;
    delay_nns(5);
  }
}

static uint8_t i2c_read_byte(unsigned char ack)
{
  uint8_t TData = 0, i;
  IIC_SDA_H;
  sda_in();
  for (i = 0; i < 8; i++)
  {
    IIC_SCL_H;
    delay_nns(5);
    TData = TData << 1;
    if (READ_SDA == GPIO_PIN_SET)
    {
      TData |= 0x01;
    }
    IIC_SCL_L;
    delay_nns(5);
  }
  if (!ack)
    i2c_nack();
  else
    i2c_ack();
  return TData;
}

int i2c_read(uint8_t reg, uint16_t *data, uint8_t data_len)
{
  uint8_t i = 0, *point = (uint8_t *)data;
  i2c_start();
  i2c_send_byte(i2c_addr);
  i2c_wait_ack();
  i2c_send_byte(reg);
  i2c_wait_ack();
  i2c_stop();

  i2c_start();
  i2c_send_byte(i2c_addr + 1);
  i2c_wait_ack();
  for (i = 0; i < (data_len - 1); i++)
  {
    point[i] = i2c_read_byte(1);
  }
  point[data_len - 1] = i2c_read_byte(0);
  i2c_stop();
  return 0;
}

int i2c_write(uint8_t reg, uint16_t *data, uint8_t data_len)
{
  uint8_t i = 0, *point = (uint8_t *)data;
  i2c_start();
  i2c_send_byte(i2c_addr);
  i2c_wait_ack();

  i2c_send_byte(reg);
  i2c_wait_ack();

  for (i = 0; i < data_len; i++)
  {
    i2c_send_byte(point[i]);
    i2c_wait_ack();
  }
  i2c_stop();
  return 0;
}
