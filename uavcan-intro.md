# UAVCAN 介绍

![](images/uavcan-logo-transparent.png)

[UAVCAN](http://uavcan.org) 是一种板载总线网络，可以使自动驾驶仪与各种设备进行通信。可以挂载在该总线下的硬件设备有：

  * 电机控制器
    * [Pixhawk 电调](https://pixhawk.org/modules/pixhawk_esc)
    * [SV2740 电调](https://github.com/thiemar/vectorcontrol)
  * 空速传感器
    * [Thiemar 空速传感器](https://github.com/thiemar/airspeed)
  * 全球卫星导航系统（GPS 和 GLONASS）信号接收机
    * [Zubax 卫星信号接收机](http://zubax.com/product/zubax-gnss)

与业余级的设备相比，UAVCAN 网络使用了可靠的差分信号并支持通过总线进行固件的升级。所有的电机控制器提供状态反馈并实现磁场矢量控制（field-oriented-control，FOC）。

## 升级节点固件

在检测到对应固件的时候，PX4 中间件将在网络节点上自动升级。相应的过程和要求参见 [UAVCAN 固件](uavcan-node-firmware.md)。

## 对电机控制器进行编号和配置

通过 [UAVCAN 节点编号与配置](uavcan-node-enumeration.md) ，每个电机控制器的节点编号和旋转方向就设置好了。该操作可由用户通过地面站软件 QGroundControl 完成。

## 可参考的链接

* [首页](http://uavcan.org)
* [详细介绍](http://uavcan.org/Specification)
* [实现和教程](http://uavcan.org/Implementations)
