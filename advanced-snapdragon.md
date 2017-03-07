# 骁龙飞控进阶

## 骁龙飞控连接方式

### 通过 FTDI

将小调试板插在 Snapdragon 的主板上，使用 FTDI 线将其与主机连接。

如果在 Linux 平台，在控制台中输入如下命令：

```
screen /dev/ttyUSB0 115200
```

将 USB0 更改为当前识别到的 USB 端口。 通过 `/dev/` 或 `/dev/serial/by-id` 检查。



### 通过 ADB (Android Debug Bridge)

将骁龙通过 USB2.0 与主机连接，并用供电模块供电。当骁龙启动后，蓝色的 LED 灯将会缓慢的闪烁（呼吸）。

通过 adb 确认飞控板已经被检测到：

```
adb devices
```

如果无法显示设备，很可能是 USB 设备权限的问题。根据提示

输入以下命令获取一个 shell:

```
adb shell
```

## 升级骁龙

此步骤需要 Intrynsic 提供的 Flight_BSP 压缩文件，使用主板序列号注册后可以获得该文件。



### 升级/替换 Linux 镜像

<aside class="caution">烧写 Linux 镜像将会清除骁龙飞控板上的全部内容，进行此步骤前务必做好你的工作备份！</aside>

通过 adb 确认飞控板已经被检测到：

```
adb devices
```

然后重启飞控板进入fastboot模式：

```
adb reboot bootloader
```

通过 fastboot 确认飞控板已经被检测到：

```
fastboot devices
```


从Intrinsyc下载最新的板级支持包(BSP)：

```
unzip Flight_3.1.1_BSP_apq8074-00003.zip
cd BSP/binaries/Flight_BSP_4.0
./fastboot-all.sh
```

若 `recovery`， `update` 或 `factory` 部分出现错误，属于正常情况。


### 升级 ADSP 固件

部分 PX4 栈在 ADSP（骁龙 8074 的 DSP 端） 上运行。底层的操作系统 QURT 需要单独升级。

<aside class="caution">如果在升级 ADSP 固件期间出现问题， 你的骁龙可能会被刷成砖！仔细的按照下列步骤操作，能在大部分情况下防止你的飞控板被刷成砖。</aside>

首先，如果你的BSP还没有达到3.1.1版本，那么先[升级 Linux 镜像](#upgradingreplacing-the-linux-image)！


#### 防止刷成砖

为了防止 ADSP 固件的问题而导致系统卡死在启动过程中，请在升级之前完成以下改动：

通过 `screen` 或者 `adb shell` 直接修改骁龙板上的文件：

```sh
vim /usr/local/qr-linux/q6-admin.sh
```

或者将文件加载到本地并在本地用你自己的编辑器修改它：

将文件加载到本地：
```sh
adb pull /usr/local/qr-linux/q6-admin.sh
```

编辑：

```sh
gedit q6-admin.sh
```

然后推送回去：

```sh
adb push q6-admin.sh /usr/local/qr-linux/q6-admin.sh
adb shell chmod +x /usr/local/qr-linux/q6-admin.sh
```

注释掉会导致启动卡死的 while 循环：

```
# Wait for adsp.mdt to show up
#while [ ! -s /lib/firmware/adsp.mdt ]; do
#  sleep 0.1
#done
```

以及：

```
# Don't leave until ADSP is up
#while [ "`cat /sys/kernel/debug/msm_subsys/adsp`" != "2" ]; do
#  sleep 0.1
#done
```

#### 推送最新的 ADSP 固件文件

从 Intrinsyc 下载文件：[Flight_3.1.1a_qcom_flight_controller_hexagon_sdk_add_on.zip](http://support.intrinsyc.com/attachments/download/691/Flight_3.1.1a_qcom_flight_controller_hexagon_sdk_add_on.zip) 。

并将文件复制到骁龙板上：

```
unzip Flight_3.1.1a_qcom_flight_controller_hexagon_sdk_add_on.zip
cd images/8074-eagle/normal/adsp_proc/obj/qdsp6v5_ReleaseG/LA/system/etc/firmware
adb push . /lib/firmware
```

然后重启一次，让固件能够应用起来：

```
adb reboot
```


## 串口

### 使用串口

现阶段的 QURT 无法支持所有的 POSIX 命令。因此需要用到一些自定义的 ioctl。

用来设置以及使用 UART 的 API 的描述信息包含在[dspal](https://github.com/PX4/dspal/blob/master/include/dev_fs_lib_serial.h)中。

## Wifi设置

<aside class="todo">这些是给经验丰富的开发者的说明。</aside>

连接上 Linux shell （参见 [console instructions](advanced-system-console.html#snapdragon-flight-wiring-the-console)）。

### 接入点模式

如果你想把骁龙设置为一个 wifi 接入点（AP 模式），那么修改文件：`/etc/hostapd.conf`并且设置：

```
ssid=EnterYourSSID
wpa_passphrase=EnterYourPassphrase
```

然后配置 AP 模式：

```
/usr/local/qr-linux/wificonfig.sh -s softap
reboot
```

### 站模式

如果你想让骁龙连上你现有的 wifi ，那么修改此文件：`/etc/wpa_supplicant/wpa_supplicant.conf` 并添加你的网络设置：


```
network={
    ssid="my existing network ssid"
    psk="my existing password"
}
```

然后配置站模式：

```
/usr/local/qr-linux/wificonfig.sh -s station
reboot
```


## 故障排除

### adb 不工作

- 检查 [权限](#usb-permissions)
- 确认你的 Micro-USB 线正常连接。
- 试试 USB 2.0 接口。
- 试试你电脑前面或后面的接口。


### USB 权限

1） 创建一个新的权限管理文件

```
sudo -i gedit /etc/udev/rules.d/51-android.rules
```

粘贴以下内容，这将使 ADB 能够接入大多数已知设备：


```
SUBSYSTEM=="usb", ATTRS{idVendor}=="0bb4", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0e79", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0502", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0b05", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="413c", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0489", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="091e", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="18d1", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0bb4", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="12d1", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="24e3", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="2116", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0482", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="17ef", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1004", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="22b8", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0409", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="2080", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0955", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="2257", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="10a9", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1d4d", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0471", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="04da", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="05c6", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1f53", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="04e8", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="04dd", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0fce", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0930", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="19d2", MODE="0666", GROUP="plugdev"
```

为文件设置正确的权限：

```
sudo chmod a+r /etc/udev/rules.d/51-android.rules
```

重启守护进程：

```
sudo udevadm control --reload-rules
sudo service udev restart
sudo udevadm trigger
```

如果 ADB 还是无法工作，看看 [在 StackOverflow 上的这个回答](http://askubuntu.com/questions/461729/ubuntu-is-not-detecting-my-android-device#answer-644222).


### 主板无法启动 / 无限重启 / 变砖

如果使用串口控制台仍能连上飞控板，并得到一个提示符，如︰


```
root@linaro-developer:~#
```

你可以输入如下命令进入 fastboot（引导装载程序）模式：


```
reboot2fastboot
```

如果串口控制台不可用，你可以尝试使用 Micro USB，并输入如下命令：


```
adb wait-for-device && adb reboot bootloader
```


然后重新给主板上电。如果你运气够好，adb 会短暂地连上主板并使主板进入 fastboot 模式。

使用下列命令检查主板是否处于 fastboot 模式：

```
fastboot devices
```

一旦成功进入 fastboot 模式，就可以尝试通过[上述步骤](#upgradingreplacing-the-linux-image)升级 Android/Linux 镜像。

如果你的主板正好是一块 [P2 板](#do-i-have-a-p1-or-p2-board)，你应该可以在启动骁龙的时候，通过短接 J3 标识符旁边的两个引脚（在角落的孔和 SD 卡槽之间的两个互相垂直的引脚），将骁龙重置到恢复镜像。


如果所有尝试都无效，你可能需要向 Intrinsyc 寻求帮助。



### 设备上没有剩余空间

有时 `make eagle_default upload` 会上传失败：

```
failed to copy 'px4' to '/home/linaro/px4': No space left on device
```

如果 ramdumps 占满硬盘就会产生这个错误。要清理硬盘，请执行：



```
rm -rf /var/log/ramdump/*
```

同样的，log 文件也可能会占满空间。要删除 log 文件，请执行：



```
rm -rf /root/log/*
```

### 未定义的 PLT 符号

#### _FDtest

如果在启动 px4 程序时看到 mini-dm 有如下输出，那么说明你需要[升级ADSP固件](#updating-the-adsp-firmware)：

```
[08500/03]  05:10.960  HAP:45:undefined PLT symbol _FDtest (689) /libpx4muorb_skel.so  0303  symbol.c
```

#### 其他

如果你改动了源码，类似于添加了函数后，显示`undefined PLT symbol ...`，那说明链接失败了。

- 你的函数声明与函数定义一一对应吗?
- 你的代码真的被编译了吗？模块被添加到 [cmake config](https://github.com/PX4/Firmware/blob/master/cmake/configs/qurt_eagle_default.cmake) 了吗？

- （添加的）文件包含在`CMakeLists.txt`中吗？
- 试着将它添加到POSIX编译里再进行编译。POSIX链接器会将编译/链接时出现的链接错误显示出来。




### krait 启动时更新参数 XXX 失败

```
ERROR [platforms__posix__px4_layer] krait update param 297 failed
ERROR [platforms__posix__px4_layer] krait update param 646 failed

px4 starting.
ERROR [muorb] Initialize Error calling the uorb fastrpc initalize method..
ERROR [muorb] ERROR: FastRpcWrapper Not Initialized
```

启动 px4 时如果出现上述错误，请尝试：
- [更新 Linux 镜像](#upgradingreplacing-the-linux-image)
- 以及 [更新 ADSP 固件](#updating-the-adsp-firmware)。操作时尝试使用 Linux 实体系统而不是虚拟机。已有[报告](https://github.com/PX4/Firmware/issues/5303)称使用虚拟机更新可能会不起作用。
- 然后 [重新编译 px4 软件](http://dev.px4.io/starting-building.html#building-px4-software)，首先彻底删除你已有的固件仓库再[按此所述](http://dev.px4.io/starting-building.html#compiling-on-the-console)重新克隆。
- 最后 [重新编译运行 px4](http://dev.px4.io/starting-building.html#qurt--snapdragon-based-boards)
- 确保设置了 `/usr/local/qr-linux/q6-admin.sh` 的可执行权限：
  `adb shell chmod +x /usr/local/qr-linux/q6-admin.sh`


### ADSP 重启

如果 mini-dm 控制台突然显示大段初始化输出，那说明 ADSP 端已经崩溃了。崩溃的原因并不明显，有可能是一些段错误，空指针异常等。

mini-dm 控制台的输出看起来通常是像这样：


```
[08500/02]  20:32.332  Process Sensor launched with ID=1   0130  main.c
[08500/02]  20:32.337  mmpm_register: MMPM client for USM ADSP core 12  0117  UltrasoundStreamMgr_Mmpm.cpp
[08500/02]  20:32.338  ADSP License DB: License validation function with id 164678 stored.  0280  adsp_license_db.cpp
[08500/02]  20:32.338  AvsCoreSvc: StartSvcHandler Enter  0518  AdspCoreSvc.cpp
[08500/02]  20:32.338  AdspCoreSvc: Started successfully  0534  AdspCoreSvc.cpp
[08500/02]  20:32.342  DSPS INIT  0191  sns_init_dsps.c
[08500/02]  20:32.342  INIT DONE  0224  sns_init_dsps.c
[08500/02]  20:32.342  Sensors Init : waiting(1)  0160  sns_init_dsps.c
[08500/02]  20:32.342  INIT DONE  0224  sns_init_dsps.c
[08500/02]  20:32.342  THRD CREATE: Thread=0x39 Name(Hex)= 53, 4e, 53, 5f, 53, 4d, 47, 52  0186  qurt_elite_thread.cpp
[08500/02]  20:32.343  THRD CREATE: Thread=0x38 Name(Hex)= 53, 4e, 53, 5f, 53, 41, 4d, 0  0186  qurt_elite_thread.cpp
[08500/02]  20:32.343  THRD CREATE: Thread=0x37 Name(Hex)= 53, 4e, 53, 5f, 53, 43, 4d, 0  0186  qurt_elite_thread.cpp
[08500/02]  20:32.343  THRD CREATE: Thread=0x35 Name(Hex)= 53, 4e, 53, 5f, 50, 4d, 0, 0  0186  qurt_elite_thread.cpp
[08500/02]  20:32.343  THRD CREATE: Thread=0x34 Name(Hex)= 53, 4e, 53, 5f, 53, 53, 4d, 0  0186  qurt_elite_thread.cpp
[08500/02]  20:32.343  THRD CREATE: Thread=0x33 Name(Hex)= 53, 4e, 53, 5f, 44, 45, 42, 55  0186  qurt_elite_thread.cpp
[08500/02]  20:32.343  Sensors Init : ///////////init once completed///////////  0169  sns_init_dsps.c
[08500/02]  20:32.342  loading BLSP configuration  0189  blsp_config.c
[08500/02]  20:32.343  Sensors DIAG F3 Trace Buffer Initialized  0260  sns_init_dsps.c
[08500/02]  20:32.345  INIT DONE  0224  sns_init_dsps.c
[00053/03]  20:32.345  Unsupported algorithm service id 0  0953  sns_scm_ext.c
[08500/02]  20:32.346  INIT DONE  0224  sns_init_dsps.c
[08500/02]  20:32.347  INIT DONE  0224  sns_init_dsps.c
[08500/02]  20:32.347  INIT DONE  0224  sns_init_dsps.c
[08500/02]  20:32.546  HAP:159:unable to open the specified file path  0167  file.c
[08500/04]  20:32.546  failed to open /usr/share/data/adsp/blsp.config  0204  blsp_config.c
[08500/04]  20:32.546  QDSP6 Main.c: blsp_config_load() failed  0261  main.c
[08500/02]  20:32.546  Loaded default UART-BAM mapping  0035  blsp_config.c
[08500/02]  20:32.546  UART tty-1: BAM-9  0043  blsp_config.c
[08500/02]  20:32.546  UART tty-2: BAM-6  0043  blsp_config.c
[08500/02]  20:32.546  UART tty-3: BAM-8  0043  blsp_config.c
[08500/02]  20:32.546  UART tty-4: BAM-2  0043  blsp_config.c
[08500/02]  20:32.546  UART tty-5: BAM N/A  0048  blsp_config.c
[08500/02]  20:32.546  UART tty-6: BAM N/A  0048  blsp_config.c
[08500/02]  20:32.547  HAP:111:cannot find /oemconfig.so  0141  load.c
[08500/03]  20:32.547  HAP:4211::error: -1: 0 == dynconfig_init(&conf, "security")   0696  sigverify.c
[08500/02]  20:32.548  HAP:76:cannot find /voiceproc_tx.so  0141  load.c
[08500/02]  20:32.550  HAP:76:cannot find /voiceproc_rx.so  0141  load.c
```

### P1 板还是 P2 板？

骁龙板上的丝印类似于：


```
1DN14-25-
H9550-P1
REV A
QUALCOMM
```

如果你看到**H9550**，说明你有一块P2板！

**请忽略上面的-P1。**

可能是由于 P1 板没有出厂分区/镜像，因此不能恢复到出厂状态。

