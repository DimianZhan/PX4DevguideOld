# 板外控制（Offboard Control）

> ** 警告 ** 板外控制非常危险。开发者请务必在板外飞行开始之前确保做好了充足的准备、测试和采取了安全防护措施。

板外控制的思想是能够利用在自驾仪之外运行的软件控制 px4 飞行栈。这是通过MAVLink协议完成的，尤其是其中的[SET_POSITION_TARGET_LOCAL_NED](http://mavlink.org/messages/common#SET_POSITION_TARGET_LOCAL_NED) 和 [SET_ATTITUDE_TARGET](http://mavlink.org/messages/common#SET_ATTITUDE_TARGET) 消息。

## 板外控制固件安装
在开始板外控制模式之前，在固件上需要做出两处设置。

### 1. 映射一个遥控器开关用于触发板外控制模式
在 qGroundcontrol 里面找到  RC_MAP_OFFB_SW 参数，通过这个参数可以设置计划用来触发外部控制模式的遥控器通道。这种映射非常有用，它可以使得在当退出板外控制模式时可以进入位置控制模式。

由于可以通过MAVLink消息来触发板外控制模式，这个步骤不是强制性的。但是这个方法更为安全。

### 2. 打开板载计算机的接口
寻找 [SYS_COMPANION](https://pixhawk.org/firmware/parameters#system) 参数并且将它设置为 921600 (建议) 或者 57600。 这个参数将会触发 Telem2 接口上触发以特定波特率 (921600 8N1 or 57600 8N1) 传输的MAVLink流，该MAVLink流带有针对板上模式的数据流。

如需了解更多这类数据流的信息，请在 [source code](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_main.cpp) 当中查阅 "MAVLINK_MODE_ONBOARD"。

## 硬件安装

通常，有三种方案可用于建立板外控制的通信链路。

### 1. 串行通信无线模块
1. 一个串行通信无线模块连接到自驾仪的UART接口上
2. 另一个串行通信无线模块连接到地面站计算机上

样例无线模块包括
* [Lairdtech RM024](http://www.lairdtech.com/products/rm024)
* [Digi International XBee Pro](http://www.digi.com/products/xbee-rf-solutions/modules)

{% mermaid %}
graph TD;
  gnd[Ground Station] --MAVLink--> rad1[Ground Radio];
  rad1 --RadioProtocol--> rad2[Vehicle Radio];
  rad2 --MAVLink--> a[Autopilot];
{% endmermaid %}

### 2.板载处理器
在无人机上搭载的小型计算机与自驾仪通过 UART 串口转 USB 接口的适配器相连。取决于出了给自驾仪发送指令之外所需要执行的板上处理任务的不同，板载处理器可以有许多可选的型号。

小型低功耗的板载处理器有如下种类:
* [Odroid C1+](http://www.hardkernel.com/main/products/prdt_info.php?g_code=G143703355573) or [Odroid XU4](http://www.hardkernel.com/main/products/prdt_info.php?g_code=G143452239825)
* [Raspberry Pi](https://www.raspberrypi.org/)
* [Intel Edison](http://www.intel.com/content/www/us/en/do-it-yourself/edison.html)

大高功耗计算机有如下一些种类：
* [Intel NUC](http://www.intel.com/content/www/us/en/nuc/overview.html)
* [Gigabyte Brix](http://www.gigabyte.com/products/list.aspx?s=47&ck=104)
* [Nvidia Jetson TK1](https://developer.nvidia.com/jetson-tk1)

{% mermaid %}
graph TD;
  comp[Companion Computer] --MAVLink--> uart[UART Adapter];
  uart --MAVLink--> Autopilot;
{% endmermaid %}

### 3. 利用板载处理器与 WIFI 连接到 ROS （***建议***）
这种方法中，搭载在无人机上的小型计算机在通过 UART 串口转 USB 接口的适配器相连接的同时，通过 WIFI 网络连接到运行着 ROS 的地面站相连接。上述任意的计算机配合 WIFI 适配器都可以用于此方案。例如，NUC D34010WYB 带有半高的 MINI PCI-E 接口并且可以使用 [Intel Wifi Link 5000](http://www.intel.com/products/wireless/adapters/5000/) 适配器。


{% mermaid %}
	graph TD
	subgraph Ground  Station
	  gnd[ROS Enabled Computer] --- qgc[qGroundControl]
	end
	gnd --MAVLink/UDP--> w[WiFi];
	qgc --MAVLink--> w;
	subgraph Vehicle
	  comp[Companion Computer] --MAVLink--> uart[UART Adapter]
	uart --- Autopilot
	end
	w --- comp
{% endmermaid %}
