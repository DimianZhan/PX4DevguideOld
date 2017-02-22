# 全软件 (Software in the Loop, SITL) 仿真 

全软件仿真是指在仅在主机上通过运行仿真软件，模拟完整的系统和其中的自动驾驶仪。该系统通过本地网络与仿真软件连接。其架构如下所示：

{% mermaid %}
graph LR;
  仿真软件-->MAVLink;
  MAVLink-->仿真系统;
{% endmermaid %}

## 进行全软件仿真

首先确认按照 [仿真准备](starting-installing.md) 在主机系统上安装了必要的组件, 然后启动: 在兼容POSIX的系统上，使用 make 指令可以迅捷地编译并运行仿真软件。

```sh
make posix_sitl_default jmavsim
```

接下来会显示 PX4 的 shell 信息:

```sh
[init] shell id: 140735313310464
[init] task name: px4

______  __   __    ___ 
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

Ready to fly.


pxh>
```

## 关键的文件

  * 启动脚本名字为 `rcS_SIM_AIRFRAME`，保存在路径 [posix-configs/SITL/init](https://github.com/PX4/Firmware/tree/master/posix-configs/SITL/init) 下，默认为 `rcS_jmavsim_iris`。
  * 系统根目录在编译生成的文件夹下： `build_posix_sitl_default/src/firmware/posix/rootfs/`

## 起飞

仿真软件 [jMAVSim](http://github.com/PX4/jMAVSim.git) 的3D场景窗口会弹出：

![](images/sim/jmavsim.png)

一旦完成初始化，仿真软件会显示无人机起始位置的坐标 (`telem> home: 55.7533950, 37.6254270, -0.00`)。 你可以通过输入指令使无人机起飞：

```sh
pxh> commander takeoff
```

> **提示** 通过地面站软件 QGroundControl (QGC) 可以支持手柄或摇杆的操作. 如果要实现手动控制，将系统切换为手动飞行模式 (比如飞行模式切换为 POSCTL, 位置控制)。 在 QGC 的参数设置界面中使能摇杆操作。

## 仿真一架Wifi接入的无人机

可以编译生成另一种环境，在本地网络中仿真Wifi接入的无人机：

```sh
make broadcast jmavsim
```

就像现实中Wifi接入的无人机一样，仿真软件会在本地网络中广播它的网络地址。

## 扩展和定制

如果需要扩展或者定制仿真界面，可以修改路径 `Tools/jMAVSim`  下的文件。可在 Github 网站上获取代码： [jMAVSim repository](https://github.com/px4/jMAVSim)。

> **提示** 编译系统会强制所有被依赖的组件进行 hash 校验，仿真软件也不例外。当部分文件被改动以后，需要在固件库中进行新的提交以更新文件在库中的注册信息，并且新的注册操作不会对文件本身的修改产生任何影响。在主机系统中进行操作： `git add Tools/jMAVSim` 并提交。之后仿真软件的 hash 校验会被更新。

## 与 ROS 的接口

这套仿真系统可以模拟一个真实的移动平台，[与 ROS 进行对接](simulation-ros-interface.md)。
