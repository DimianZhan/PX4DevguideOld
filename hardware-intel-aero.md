# Intel Aero

Aero是一款无人机开发平台. 其中一部分是Intel Aero开发板 (如下所示), 在一个Quad-core CPU上运行着linux系统. 他和FMU部分进行连接, 在NuttX上运行px4.


![](images/hardware/hardware-intel-aero.png)

## 介绍

主要文档在以下网址 https://github.com/intel-aero/meta-intel-aero/wiki. 它包括了如何设置，升级，以及连接至开发板的介绍.并且它也讲解了如何在linux端进行开发.

以下介绍了如何加载程序并与FMU进行连接。


## 加载程序

在设置完PX4开发环境后, 按照以下几个步骤在FMU板上安装px4程序:

1. 进行一次在Aero上的所有软件的全面升级 (https://github.com/intel-aero/meta-intel-aero/wiki/Upgrade-To-Latest-Software-Release)

2. 下载固件 [Firmware](https://github.com/PX4/Firmware)

3. 用`make aerofc-v1_default`进行编译

4. 设置主机名为 (下面的 IP建议你用WIFI连接): 
```
export AERO_HOSTNAME=192.168.1.1`
```
5. 用  `make aerofc-v1_default upload`语句进行升级


## 通过网络连接QGroundControl

1. 确保你能通过板载WIFI或者USB网络连接至开发板

2. 通过ssh连接到开发板并确保 mavlink 已经连上.系统默认当开发板启动时他应当自动开始连接. 它也可以手动启动连接:
```
/etc/init.d/mavlink_bridge.sh start
```

3. 启动QGroundControl并且应当自动连接.

4. 如果不用 QGroundControl,你可以用这个替代 [NuttX shell](advanced-system-console.md#mavlink-shell) :
```
./Tools/mavlink_shell.py 0.0.0.0:14550
```



