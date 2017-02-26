# 板载计算机在Pixhawk 系列上的使用

将板载计算机 ( Raspberry Pi, Odroid, Tegra K1 等 )与 Pixhawk 系列自驾仪相连均可使用如下的方法：利用 `TELEM2` 串行通信接口进行连接。连接所用的消息格式是 [MAVLink](http://mavlink.org).

## Pixhawk 设置

设置 `SYS_COMPANION` 参数 ( 在 System 组当中) 为如下参数之一。

> **注意** 改变这个参数需要自驾仪重启来应用更改。

  * `0` 代表禁止 TELEM2 上的 MAVLink 输出 （默认）；
  * `921600` 为使能 MAVLink 以 921600 baud 的速率输出，格式为 8N1 ( 建议 ) ；
  * `57600` 为使能 MAVLink 以 57600 baud 的速率输出，格式为8N1 ；
  * `157600` 为使能 MAVLink 以  *OSD* 模式以 57600 baud 的速率输出；
  * `257600` to enable MAVLink in listen-only mode at 57600 baud

## 板载计算机的设置

为了可以接收 MAVLink 消息，板载计算机需要运行一些用于与串口通信的软件。最常用的选择包括如下几种：

  * [MAVROS](ros-mavros-installation.md) 用于与 ROS 节点通信；
  * [C/C++ example code](https://github.com/mavlink/c_uart_interface_example)用于连接自定义模式
  * [MAVProxy](http://mavproxy.org) 用于在串口和 UDP 之间进行路由

## 硬件设置

根据如下提示连接串行通信接口。所有的 Pixhawk 串行通信接口都使用 3.3V 电平，并且兼容5V 的电平。

> ** 警告 ** 市面上许多板载计算机的 UART 串行接口硬件上只支持 1.8V 的电平，3.3V 的电平会将接口烧掉。需要使用电平转换器。在大多数情况下可用的硬件串行通信接口已经有了一些默认设置的功能（例如调制解调或者控制台），在使用前需要在 *Linux 当中重新设置* 。 

较为保险的方法是使用带 FTDI 芯片 USB 串口转换器并且按照如下方式进行连接。这种方法能够起作用并且容易实现。

| TELEM2 |         | FTDI    |        |
--- | --- | ---
|1         | +5V (红)|         | 不要连接!   |
|2         | Tx  (输出)| 5       | FTDI RX (黄) ( 输入 )   |
|3         | Rx  (输入) | 4       | FTDI TX (橙) (输出)  |
|4         | CTS (输入) |6       | FTDI RTS (绿) (输出) |
|5         | RTS (输出)|2       | FTDI CTS (棕) (输入) |
|6         | GND     | 1       | FTDI GND (黑)   |

## Linux 上的软件安装 

在 Linux 当中 USB FTDI 默认显示为 `\dev\ttyUSB0` 。如果有另一个 FTDI 模块连接到USB 上或者连接了Arduino，它将被注册为  `\dev\ttyUSB1`。为了避免先后插入的两个模块不被混淆，我们建议根据 USB 设备供应商和产品ID上从 `ttyUSBx` 建立更为直观的符号连接。

用 `lsusb` 命令可以得到供应商和产品ID。

```sh
$ lsusb

Bus 006 Device 002: ID 0bda:8153 Realtek Semiconductor Corp.
Bus 006 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 005 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 004 Device 002: ID 05e3:0616 Genesys Logic, Inc.
Bus 004 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 003 Device 004: ID 2341:0042 Arduino SA Mega 2560 R3 (CDC ACM)
Bus 003 Device 005: ID 26ac:0011
Bus 003 Device 002: ID 05e3:0610 Genesys Logic, Inc. 4-port hub
Bus 003 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 002 Device 001: ID 1d6b:0001 Linux Foundation 1.1 root hub
Bus 001 Device 002: ID 0bda:8176 Realtek Semiconductor Corp. RTL8188CUS 802.11n WLAN Adapter
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
```

Arduino 是 `Bus 003 Device 004: ID 2341:0042 Arduino SA Mega 2560 R3 (CDC ACM)`

Pixhawk 是 `Bus 003 Device 005: ID 26ac:0011`
 
> 如果不能找到您的设备，请拔出，执行 `lsusb` ，然后再插入，重新执行 `lsusb` 就能偶看到添加的设备了。

因此，我们可以在叫 `/etc/udev/rules.d/99-pixhawk.rules` 的文件当中建立一个新的 UDEV 规则，根据如下的内容，更改 idVendor 和 idProduct 为上面查询到的内容。 

```sh
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", SYMLINK+="ttyArduino"
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0011", SYMLINK+="ttyPixhawk"
```

最后在 **重启** 之后就能够明确的知道哪个设备是啥了。这时候您可以在脚本当中用 `/dev/ttyPixhawk` 替换 `/dev/ttyUSB0` 。

> 请通过 `usermod` 命令确保您在  `tty` 和 `dialout` 用户组当中以避免需要用 root 权限来执行脚本。

```sh
usermod -a -G tty ros-user
usermod -a -G dialout ros-user
```
