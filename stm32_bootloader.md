# STM32 引导程序
 用于加载 PX4的引导程序可以从GitHub的[Bootloader](https://github.com/px4/bootloader)库中获得。
## 支持的硬件
  * FMUv1 (PX4FMU, STM32F4)
  * FMUv2 (Pixhawk 1, STM32F4)
  * FMUv3 (Pixhawk 2, STM32F4)
  * FMUv4 (Pixracer 3 and Pixhawk 3 Pro, STM32F4)
  * FMUv5 (Pixhawk 4, STM32F7)
  * TAPv1 (TBA, STM32F4)
  * ASCv1 (TBA, STM32F4)
##  编译引导程序
```bash
git clone https://github.com/PX4/Bootloader.git
cd Bootloader
make
```
运行make后，编译产生一系列可执行文件（格式为elf）并保存在Bootloader文件下，所得可执行文件支持所有PX4硬件。
## 烧写引导程序
>重要信息：对于一些允许JTAG/SWD的飞控板必须保证正确的上电顺序。请严格按照以下步骤完成。以下说明适用于Blackmagic / Dronecode仿真。其他JTAG仿真不同于前面两个，但步骤类似。试图刷写bootloader的编程人员应该具备这些知识。如果你不知道知道这些，你可能真的应该考虑，你是否真的需要修改bootloader引导加载程序。
  * 断开JTAG cable
  * 连接USB电源cable
  * 连接JTAG cable
#### 使用正确的串口
  * On LINUX: ```/dev/serial/by-id/usb-Black_Sphere_XXX-if00```
  * On MAC OS: Make sure to use the cu.xxx port, not the tty.xxx port: ```tar ext /dev/tty.usbmodemDDEasdf```

```bash
arm-none-eabi-gdb
  (gdb) tar ext /dev/serial/by-id/usb-Black_Sphere_XXX-if00
  (gdb) mon swdp_scan
  (gdb) attach 1
  (gdb) mon option erase
  (gdb) mon erase_mass
  (gdb) load tapv1_bl.elf
        ...
        Transfer rate: 17 KB/sec, 828 bytes/write.
  (gdb) kill
```
### J-Link
J-Link相关说明看[J-Link GDB server](https://www.segger.com/jlink-gdb-server.html).
#### 前提条件
[下载 J-Link 软件](https://www.segger.com/downloads/jlink#)并按照Segger说明安装。
#### 运行JLink GDB服务
FMUv1:
```bash
JLinkGDBServer -select USB=0 -device STM32F405RG -if SWD-DP -speed 20000
```

AeroFC:
```bash
JLinkGDBServer -select USB=0 -device STM32F429AI -if SWD-DP -speed 20000
```
#### 链接GDB
```bash
arm-none-eabi-gdb
  (gdb) tar ext :2331
  (gdb) load aerofcv1_bl.elf
```
### 故障排除
运行命令时，如果提示上述命令没有被找到，可能是你没有使用Black magic或者软件已经过期。此时，请先升级on-probe软件。

如果提示下方错误消息：

```Error erasing flash with vFlashErase packet```

Disconnect the target (while leaving JTAG connected) and run 

断开连接（保持JTAG连接状态）并运行下列命令：

```bash
mon tpwr disable
swdp_scan
attach 1
load tapv1_bl.elf
```

此时将断开飞控板电源并尝试再次写入。
