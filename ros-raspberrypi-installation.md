# 树莓派 - ROS 安装

这个指南指导你如何在树莓派2上安装ROS-indigo作为Pixhawk的配套计算机。

## 前提
* 一个工作的树莓派，有显示器、键盘或者配置好的SSH连接
* 本指南假设您在RPi上安装了Raspbian “JESSIE”。 如果没有：[安装](https://www.raspberrypi.org/downloads/raspbian/)或[升级](http://raspberrypi.stackexchange.com/questions/27858/upgrade-to-raspbian-jessie)你的Raspbian Wheezy至Jessie。

## 安装
按照该[指南](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi)实际安装ROS Indigo。 注意：安装“ROS-Comm”版。 桌面版过于庞大	。

### 安装软件包时出错
如果你想要下载包（例如`sudo apt-get install ros-indigo-ros-tutorials`），你可能会遇到一个错误：“unable to locate package ros-indigo-ros-tutorials”。

如果是，请按照以下步骤操作：
转到您的catkin工作区（例如~/ros_catkin_ws）并更改软件包的名称。

```sh
$ cd ~/ros_catkin_ws

$ rosinstall_generator ros_tutorials --rosdistro indigo --deps --wet-only --exclude roslisp --tar > indigo-custom_ros.rosinstall
```

然后, 用wstool更新你的工作区。

```sh
$ wstool merge -t src indigo-custom_ros.rosinstall

$ wstool update -t src
```

然后 (仍然在你的工作区目录), source 并且 make 你的文件.

```sh
$ source /opt/ros/indigo/setup.bash

$ source devel/setup.bash

$ catkin_make
```
