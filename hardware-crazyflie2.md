# Crazyflie 2.0

Crazyflie 是由Bitcraze AB发起的微型四旋翼飞行器项目. Crazyflie 2 (CF2) 的概述在以下网址: https://www.bitcraze.io/crazyflie-2/

![](images/hardware/hardware-crazyflie2.png)

## 概要

> ** 主要硬件手册网址: https://wiki.bitcraze.io/projects:crazyflie2:index **

  * 主处理器芯片: STM32F405RG
    * CPU: 168 MHz主频  ARM Cortex M4内核  内置单精度浮点处理器
    * RAM: 192 KB SRAM
  * nRF51822 无线通信模块 与 电源管理芯片
  * MPU9250 加速度计 / 陀螺仪/ 磁力计
  * LPS25H 气压计


## 加载程序

设置完 PX4开发环境后, 按照以下步骤在CF2上运行PX4软件程序:

1. 下载PX4源代码 [Bootloader](https://github.com/PX4/Bootloader)

2. 用 `make crazyflie_bl`编译

3. 用 DFU模式运行CF2:
	- 确保初始化时未上电
	- 按住按钮
	- 插入usb端口
	- 一秒后,蓝灯应当开始闪烁并且五秒后闪烁更快
	- 松开按钮

4. 用dfu-util加载bootloader: `sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000 -D crazyflie_bl.bin` 并且当完成时拔出CF2的接口
	- 如果成功完成,  当再次插入接口时，那么黄灯此时应当闪烁

5. 下载固件 [Firmware](https://github.com/PX4/Firmware)

6. 用 `make crazyflie_default upload`编译

7. 当提示插入设备时, 插入CF2: 黄灯开始闪烁代表进入bootloader模式. 然后红灯常亮代表加载程序过程开始.

8. 等待完成

9. 完工! 通过QGC校准传感器信息

##无线通信

板载的nRF模块可以通过蓝牙或者以专有的2.4GHz北欧ESB协议连接到飞控板上

- 推荐使用 [Crazyradio PA](https://www.bitcraze.io/crazyradio-pa/).
- 为了让CF2即刻飞行, Crazyflie手机应用可以通过蓝牙连接

使用官方的Bitcraze **Crazyflie手机应用**

- 通过蓝牙连接
- 通过设置1 or 2来改变模式
- 通过QGC校准传感器信息


通过 **MAVLink**连接

- 使用与GCS兼容的crazyradio PA
- 浏览 [cfbridge](https://github.com/dennisss/cfbridge) 网页以了解如何通过无线以任何的用户协议连接至地面站


## 飞行

{% youtube %}https://www.youtube.com/watch?v=oWk0RRIzF-4{% endyoutube %}
