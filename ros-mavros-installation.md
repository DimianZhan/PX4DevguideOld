
# MAVROS

ROS（机器人操作系统）下的 [mavros](http://wiki.ros.org/mavros#mavros.2BAC8-Plugins.sys_status) 包是一个对 MAVLink 协议端的拓展，mavros 可以使运行 ROS 的电脑与支持 MAVLink 的自驾仪或 GCS 相互通信。尽管 mavros 可以与任何支持 MAVLink 的固件交互，但本文只讨论在 PX4 固件下的使用情况。  

## 安装

mavros 包可以通过 apt-get 直接安装可执行文件，也可以编译源码后安装。但我们建议熟悉 ROS 的开发者从源码编译安装 mavros 包。


### 直接安装deb包 (Debian / Ubuntu)

mavros 从 v0.5 版本起支持在 x86 和 amd64 下从软件源中直接安装，v0.9 以后的版本更是增加了对 armhf 的支持；因此，简单使用 apt-get 便可以完成 mavros 的安装：
```sh
$ sudo apt-get install ros-indigo-mavros ros-indigo-mavros-extras
```
> 译者注：Ubuntu 16.04 系统下建议使用 apt install 命令代替 apt-get install

### 源码编译安装
**依赖库**

以下安装过程假定您已经正确建立了一个用于编译 ROS 软件包的 catkin 工作空间，其路径为 `~/catkin_ws`， 建立这个工作空间的方法是：
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin init
```

您必须使用 ROS 中的 python 工具 `wstool, rosinstall, 和 catkin_tools` 来进行此次编译及安装，如果之前没有安装过这几款工具，请使用以下命令来安装：
```sh
$ sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools
```

注意，虽然可以使用 catkin_make 编译工作空间，但是我们推荐使用更通用和友好的 catkin_tools 来代替前者。

如果这是你第一次使用 wstool ，则需要执行以下命令初始化包目录：
```sh
$ wstool init ~/catkin_ws/src
```

现在，一切都已经准备就绪了，请按照以下操作来完成编译：
```sh
    # 1. 下载 mavros 的 Release 版本源码
$ rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
    # 备选: 开发版源码
$ rosinstall_generator --upstream-development mavros | tee /tmp/mavros.rosinstall

    # 2. 下载 mavlink 的 Release 版本源码
    # 备注： 如果以后需要更新 mavlink 的代码，可以从这一行以下执行
$ rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall

    # 3. 配置工作空间并设置安装路径（实际安装动作将在最后一句 build 命令执行完以后触发）
$ wstool merge -t src /tmp/mavros.rosinstall
$ wstool update -t src
$ rosdep install --from-paths src --ignore-src --rosdistro indigo -y

    # 4. 编译
$ catkin build
```
<aside class="note">
如果你在树莓派中安装 mavros ，编译过程可能会产生一个有关你操作系统的错误，解决办法是在 `rosdep install ...` 指令后附加 `--os=OS_NAME:OS_VERSION` 参数，其中`OS_NAME`是你系统类型，`OS_VERSION`是版本号，例如 `--os=debian:jessie`
</aside>
