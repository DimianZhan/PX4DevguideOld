# STM32 Bootloader
# STM32引导程序
The code for the PX4 bootloader is available from the Github [Bootloader](https://github.com/px4/bootloader) repository.
#用于加载PX4的引导程序可以从GitHub的[Bootloader](https://github.com/px4/bootloader)库中获得。
## Supported Boards
#  支持的硬件
  * FMUv1 (PX4FMU, STM32F4)
  * FMUv2 (Pixhawk 1, STM32F4)
  * FMUv3 (Pixhawk 2, STM32F4)
  * FMUv4 (Pixracer 3 and Pixhawk 3 Pro, STM32F4)
  * FMUv5 (Pixhawk 4, STM32F7)
  * TAPv1 (TBA, STM32F4)
  * ASCv1 (TBA, STM32F4)

## Building the Bootloader
##  编译引导程序
```bash
git clone https://github.com/PX4/Bootloader.git
cd Bootloader
make
```

After this step a range of elf files for all supported boards are present in the Bootloader directory.
#运行make后，编译产生一系列可执行文件（格式为elf）并保存在Bootloader文件下，所得可执行文件支持所有PX4硬件。
## Flashing the Bootloader
## 烧写引导程序
> IMPORTANT: The right power sequence is critical for some boards to allow JTAG / SWD access. Follow these steps exactly as described. The instructions below are valid for a Blackmagic / Dronecode probe. Other JTAG probes will need different but similar steps. Developers attempting to flash the bootloader should have the required knowledge. If you do not know how to do this you probably should reconsider if you really need to change anything about the bootloader.
> 
>重要信息：对于一些允许JTAG/SWD的飞控板必须保证正确的上电顺序。请严格按照以下步骤完成。以下说明适用于Blackmagic / Dronecode仿真。其他JTAG仿真不同于前面两个，但步骤类似。试图刷写bootloader的编程人员应该具备这些知识。如果你不知道知道这些，你可能真的应该考虑，你是否真的需要修改bootloader引导加载程序。
  * Disconnect the JTAG cable
  * 断开JTAG cable
  * Connect the USB power cable
  * 连接USB电源cable
  * Connect the JTAG cable
  * 连接JTAG cable

### Black Magic / Dronecode Probe

#### Using the right serial port
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

These instructions are for the [J-Link GDB server](https://www.segger.com/jlink-gdb-server.html).
J-Link相关说明看[J-Link GDB server](https://www.segger.com/jlink-gdb-server.html).
#### Prerequisites
#### 前提条件
[Download the J-Link software](https://www.segger.com/downloads/jlink#) from the Segger website and install it according to their instructions.
[下载 J-Link 软件](https://www.segger.com/downloads/jlink#)并按照Segger说明安装。
#### Run the JLink GDB server
#### 运行JLink GDB服务
FMUv1:
```bash
JLinkGDBServer -select USB=0 -device STM32F405RG -if SWD-DP -speed 20000
```

AeroFC:
```bash
JLinkGDBServer -select USB=0 -device STM32F429AI -if SWD-DP -speed 20000
```

#### Connect GDB
#### 链接GDB
```bash
arm-none-eabi-gdb
  (gdb) tar ext :2331
  (gdb) load aerofcv1_bl.elf
```

### Troubleshooting
### 故障排除
If any of the commands above are not found, you are either not using a Blackmagic probe or its software is outdated. Upgrade the on-probe software first.

运行命令时，如果提示上述命令没有被找到，可能是你没有使用Black magic或者软件已经过期。此时，请先升级on-probe软件。

If this error message occurs:

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

This will disable target powering and attempt another flash cycle.

此时将断开飞控板电源并尝试再次写入。
