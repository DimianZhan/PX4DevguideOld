# Gazebo 仿真
> 译者注：本文中的若干部分设计到实际编程过程中的仿真调试，本人没有相关经验，为了避免误导读者，译者不确定的地方保留原文不翻译。

[Gazebo](http://gazebosim.org) 是一个适用于 3D 环境自主机器人仿真的软件，使用 Gazebo 并不一定需要 ROS 等其他环境，当然你也可以配合 Gazebo 使用 ROS 和 SITL。

{% youtube %}https://www.youtube.com/watch?v=qfFF9-0k4KA&vq=hd720{% endyoutube %}


{% mermaid %}
graph LR;
  Gazebo-->Plugin;
  Plugin-->MAVLink;
  MAVLink-->SITL;
{% endmermaid %}

## 安装

整个过程需要同时安装 Gazebo 和 我们提供的仿真插件。

> ** 提示 ** 我们推荐您使用 Gazebo 7 （至少是 Gazebo 6 以上的版本）。如果您在 Linux 下使用 ROS Jade 之前的版本，请确保一定要卸载掉 ROS 自带的老版本 Gazebo （`sudo apt-get remove ros-indigo-gazebo`）。

### Mac OS

Mac OS requires Gazebo 7 which in turn requires xquartz and doesn't run without OpenCV.
> 译者注：译者不了解 Mac 以及其上相关软件的依赖关系，不敢乱翻译。

```sh
brew cask install xquartz
brew install homebrew/science/opencv
brew install gazebo7
```

### Linux

PX4 SITL 使用 Gazebo 来进行仿真，且不依赖 ROS，当然这个仿真环境也可以[与 ROS 对接](simulation-ros-interface.md)，就像其他飞控代码。

#### ROS 用户

如果您打算在 ROS 下使用 PX4，请按照 [Gazebo 7 版本升级向导](http://gazebosim.org/tutorials?tut=ros_wrapper_versions#Gazebo7.xseries)操作，为 ROS 单独配置 Gazebo。

#### 正常安装

请按照 [Linux 下安装说明](http://gazebosim.org/tutorials?tut=install_ubuntu&ver=7.0&cat=install)来安装 Gazebo 7.

确保同时安装好了 `gazebo7` 和 `libgazebo7-dev`。

## 运行仿真

从 PX4 固件源码的文件夹内启动飞行器仿真（支持四旋翼，固定翼和垂直起降固定翼，也支持光流仿真）：

> ** 提示 ** 您可以执行下面的命令使得在不关闭 Gazebo 的情况下重启 PX4.

### 四旋翼

```sh
cd ~/src/Firmware
make posix_sitl_default gazebo
```

### 带光流的四旋翼

```sh
make posix gazebo_iris_opt_flow
```

### 3DR Solo 飞机

```sh
make posix gazebo_solo
```

![](/assets/gazebo_solo.png)

### 标准固定翼

```sh
make posix gazebo_plane
```

![](/assets/gazebo_plane.png)

### 标准垂直起降固定翼

```sh
make posix_sitl_default gazebo_standard_vtol
```

![](/assets/gazebo_standard_vtol.png)

### （尾部）立式垂直起降固定翼

```sh
make posix_sitl_default gazebo_tailsitter
```

![](/assets/gazebo_tailsitter.png)

## 修改仿真环境地图

默认情况下的地图是在 [worlds](https://github.com/PX4/sitl_gazebo/tree/367ab1bf55772c9e51f029f34c74d318833eac5b/worlds) 文件夹下的 iris.world 文件，地图中默认环境使用有高度差的大地，某些情况下这种地面结构会导致距离传感器失灵，如果遇到这种无法预料的情况，可以在 iris.model 中将大地模型从 `uneven_ground` 改为 `asphalt_plane`。
> 译者注：译者本人未遇到此种情况，因此不知道此处翻译是否准确。

## 起飞

> ** 提示 ** 请参考[安装文件和源码](http://dev.px4.io/starting-installing-mac.html)以防出现莫名其妙的问题。

之前的步骤将会打开 PX4 命令窗口:

```sh
[init] shell id: 140735313310464
[init] task name: px4

______  __   __    ___
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.


pxh>
```

> ** 提示 ** 右键点击四旋翼模型将可以在菜单栏内开启跟随模式这样能很方便的始终在窗口中展示飞机模型。

![](images/sim/gazebo.png)

一旦 PX4 完成初始化，它将会在命令窗口中输出起飞地点的位置坐标。接下来便可以使用以下命令让飞机起飞：

```sh
pxh> commander takeoff
```

> ** 提示 ** 通过 QGroundControl (QGC) 软件可以使用摇杆，若要使用手动操作模式，请将飞机设置为手动模式（例如 POSCTL, 定点模式）。启用摇杆的选项在 QGC 的 preferences 菜单中。
> 译者注：此处原文提到了两种操作杆，由于译者本人并不了解，故将其均翻译为摇杆。

## 单独开启 Gazebo 和 PX4

对于有特殊需求的开发环境，可能会要求单独开启 Gazebo 和 PX4，甚至从 IDE 中启动它们。

除了运行已经在 CMake 中配置好参数的 PX4，您还可以通过配置合适的参数和模型来执行 `sitl_run.sh` 启动 PX4。
> 译者注：这句看不太懂，因此保留原句如下   
In addition to the existing cmake targets that run `sitl_run.sh` with parameters for px4 to load the correct model it creates a launcher targets named `px4_<mode>` that is a thin wrapper around original sitl px4 app. This thin wrapper simply embeds app arguments like current working directories and the path to the model file.

> PS: `Tools/gazebo_sitl` 文件夹下有相关的地图的模型，launch 文件夹下有相关的启动文件，大伙儿看着用就好→_→

### 如何使用 IDE 启动

  * 通过终端启动 Gazebo（或其他仿真器） 的服务端和客户端可视化界面：
```
make posix_sitl_default gazebo_none_ide
```
  * 在您的 IDE 中选中想要调试的目标 `px4_<mode>`（例如 `px4_iris`）
  * 从 IDE 中直接启动调试，这种方法明显提高了开发效率，因为仿真环境（例如 Gazebo）一直运行在后台，这样您只需要重新运行 PX4 进程就好。

## 拓展和自定义

若要拓展或自定义仿真接口，您可以编辑 `Tools/sitl_gazebo` 文件夹内的文件。相关文件的[代码仓库](https://github.com/px4/sitl_gazebo)存放在 Github 上。

> ** 提示（此处译者没有试验过，保留） ** The build system enforces the correct GIT submodules, including the simulator. It will not overwrite changes in files in the directory.

## 与 ROS 对接

仿真环境可以 [与 ROS 对接](simulation-ros-interface.md)，就行前文中真实环境的对接一样。
