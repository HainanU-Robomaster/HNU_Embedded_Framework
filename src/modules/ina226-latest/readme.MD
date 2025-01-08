# INA226

## 简介

INA226 是一款分流/功率监视器， 具有 I2C™或SMBUS 兼容接口。 该器件 监视分流压降和总线电源电压。 可编程校准值、 转换时间和取平均值功能与内部乘法器相结合， 可实现电流值（单位为安培） 和功率值（单位为瓦） 的直接读取。

INA226 可在 0V 至 36V 的共模总线电压范围内感测电流， 与电源电压无关。 该器件由一个 2.7V 至 5.5V 的单电源供电， 电源电流典型值为 330μA。 该器件的额定工作温度范围为 –40°C 至 125°C， I2C 兼容接口上具有多达 16 个可编程地址。



更多信息请查看官方提供的[数据手册](https://www.ti.com.cn/product/cn/INA226)。

## 特性和优势

- 感测的总线电压范围：0V 至 36V
- 报告电流、电压和功率
- 高精度：
    - ±0.1% 系统增益误差（最大值）
    - 10uV 偏移（最大值）
- 可配置的取平均值选项
- 16 个可编程地址
- 由 2.7V 至 5.5V 电源供电
- 10 引脚 DGS 超薄小外形尺寸 (VSSOP) 封装

  
## 基于RT-thread Sensor框架使用说明

### 依赖

- RT-Thread 4.0.0+
- Sensor 组件
- IIC 驱动：INA226使用 IIC 进行数据通讯，需要系统 IIC 驱动支持；

### 获取软件包

使用 INA226 软件包需要在 RT-Thread 的包管理中选中它，具体路径如下：

```
RT-Thread online packages --->
    peripheral libraries and drivers --->
        sensors drivers --->
            [*] INA226: a INA226 package for rt-thread.package
                   [*] Enable INA226 example 
                    Version (latest)  --->
```

**Version**：软件包版本选择

### 使用软件包

刚才的menuconfig默认创建了一个使用例子，编译下载后，使用msh命令行可以打印出当前测得的电压电流和功率值。具体使用方式清参考 **example_INA226.c**

```
 \ | /
- RT -     Thread Operating System
 / | \     4.0.3 build Feb 20 2021
 2006 - 2020 Copyright by rt-thread team
[I/sensor.ina226] ina226 init success. 0
[I/sensor] rt_sensor init success
current : 0.000000 mA,voltage : 2.856504 V ,power : 0.000000 mW
msh >current : 127.280167 mA,voltage : 2.856504 V ,power : 363.576294 mW
current : 127.209938 mA,voltage : 2.856504 V ,power : 363.375702 mW
current : 126.999252 mA,voltage : 2.856504 V ,power : 362.773865 mW
current : 127.046066 mA,voltage : 2.856504 V ,power : 362.907593 mW
current : 127.069481 mA,voltage : 2.856504 V ,power : 362.974457 mW
current : 127.069481 mA,voltage : 2.856504 V ,power : 362.974457 mW


```
### 注意事项
- 如果读取的数值为0，或者是初始化失败，请注意模拟IIC的IO口输入输出配置
- 如果初始化失败，则记得检查
  - IIC引脚配置是否正确
  - IIC slave 地址配置是否正确

## 裸机或者基于RTOS开发使用说明

这个模式下只需要将 **INA226.c** 和 **INA226.h** 加入工程中即可。

软件包本身不包含I2C驱动，所以需要用户自己实现I2C驱动并实现如下类型的两个读写函数
```
typedef int (*i2c_func)(uint8_t addr, uint16_t *data, uint8_t data_len);
```
以STM32 HAL库为例，实现的驱动函数如下：
```
static int i2c_read(uint8_t addr, uint16_t *data, uint8_t data_len)
{
	return HAL_I2C_Mem_Read(&hi2c1, CHIP_ADDRESS, (uint16_t)addr, 1, (uint8_t *)data, data_len, 100);
}

static int i2c_write(uint8_t addr, uint16_t *data, uint8_t data_len)
{
    return HAL_I2C_Mem_Write(&hi2c1, CHIP_ADDRESS, (uint16_t)addr, 1, (uint8_t *)data, data_len, 100);
}
```
然后调用INA226初始化函数，并将上一步实现的IIC读写函数传入初始化参数中。
```
ret = INA226_init(rt_i2c_read_data, rt_i2c_write_data, device.addr);
if (0 == ret)
{
    LOG_I("INA226 init success.");
}
else
{
    LOG_E("INA226 init error. %d", ret);
}
```

接着就可以在需要的地方调用读取函数来获取传感器的电压电流和功率值。
```
float ma,mv,mw;
ina226_get_current(&ma);
ina226_get_bus_voltage(&mv);
ina226_get_power(&mw);

```


## 联系人信息
- [xph](https://github.com/xupenghu)
- [邮箱](xupenghu@outlook.com)
- [项目地址](https://github.com/xupenghu/ina226)




