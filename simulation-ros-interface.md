# 仿真环境与 ROS 的交互

仿真环境下的自驾仪通过 14557 端口进行 MAVLink 协议交互，因此把 mavros 包配置到这个端口，能够像实际飞行时一样接收所有数据。

## 启动 MAVROS

如果项目需求使用 ROS，上文所说的 MAVLink 将可以通过 [mavros](ros-mavros-offboard.md) 包与 ROS 交互，您需要按照如下所述配置 mavros 的 `fcu_url` 参数使其连接到一个特定的地址（`fcu_url` 是 SITL 仿真时的 IP 地址与端口参数）。

<div class="host-code"></div>

```sh
roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"
```

如果连接目标是本机，使用这个 URL：

<div class="host-code"></div>

```sh
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```


## 为 ROS 配置 Gazebo 环境

Gazebo ROS SITL 的仿真环境已知与 Gazebo 6 和 Gazebo 7 兼容，使用如下命令来安装它们：

```sh
sudo apt-get install ros-$(ROS_DISTRO)-gazebo7-ros-pkgs    // 推荐
```
or
```sh
sudo apt-get install ros-$(ROS_DISTRO)-gazebo6-ros-pkgs
```
> 译者注：Ubuntu 16.04 系统下建议使用 apt install 命令代替 apt-get install

## 与 ROS 配合使用 Gazebo

有些情况下会有整合 Gazebo 和 ROS 的需求，使其能够直接将传感器数据通过 ROS 以话题的形式发布出来，例如如果需要为仿真器环境添加 ROS 中的激光雷达插件，则必须使用整合的 Gazebo ROS 包来完成。
> 译者注：如果要配合使用 ROS 和 Gazebo，请不要直接启动 Gazebo，而应该使用 launch 文件来启动 gazebo_ros 这个包。

这里有几个配合使用 ROS 和 Gazebo 的示例 launch 文件：

  * [posix_sitl.launch](https://github.com/PX4/Firmware/blob/master/launch/posix_sitl.launch): 单独启动 SITL 仿真
  * [mavros_posix_sitl.launch](https://github.com/PX4/Firmware/blob/master/launch/mavros_posix_sitl.launch): 同时启动 SITL 仿真 和 mavros 包

为了正确的在 ROS 下使用 SITL 仿真，需要重新配置 ROS 的环境变量，相关脚本如下：

（备注：如果只是编译 mavros 包或其他 ROS 包，只要 source 一下工作空间中的 setup.bash 就好）

```sh
cd <Firmware_clone>
make posix_sitl_default gazebo
source ~/catkin_ws/devel/setup.bash    // (备注)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 posix_sitl.launch
```

在您的 launch 文件中引入上面我们提供的 launch 文件，可以在仿真环境中运行您自己的 ROS 程序。

### 这些脚本都干了些什么

(或者说，如何手动启动它们)

```sh
no_sim=1 make posix_sitl_default gazebo
```

这样启动仿真环境后，控制台看起来像这样：

```sh
[init] shell id: 46979166467136
[init] task name: px4

______  __   __    ___
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

Ready to fly.


INFO  LED::init
729 DevObj::init led
736 Added driver 0x2aba34001080 /dev/led0
INFO  LED::init
742 DevObj::init led
INFO  Not using /dev/ttyACM0 for radio control input. Assuming joystick input via MAVLink.
INFO  Waiting for initial data on UDP. Please start the flight simulator to proceed..
```

现在打开一个新终端执行以下命令将 `sitl_gazebo` 文件夹加入环境变量，确保您可以通过 Gazebo 菜单插入 Iris 飞机模型：

```sh
cd <Firmware_clone>
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default
```

现在您可以在使用 ROS 时通过任意方式打开 Gazebo 并插入 Iris 飞机模型，一旦模型被插入，它就会自动连接 PX4 程序。
> 译者注：这里的意思是指，插入 Iris 模型后，Gazebo 将会自动调用相关的 Plugin 来连接编译在本机的 PX4 程序（前文中的 posix 环境）。

```sh
roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/sitl_gazebo/worlds/iris.world
```
