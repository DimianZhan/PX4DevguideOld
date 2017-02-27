# QGroundControl

QGroundControl 是一个用于配置和驾驶基于 PX4 自驾仪的应用。它是跨平台的并且能够支持所有主要的操作系统：

  * 移动版：Android 和 ios（目前集中在平板电脑）
  * 桌面版：Windows，Linux，Mac OS
  
## 任务规划

为了创建一个新的任务，切换到规划标签，点击左上方的 + 图标和点击地图创建一个新的航点。在旁边会打开一个快捷菜单可以去调整航点。点击高亮的传送标签将这些信息发送到无人机。

![](images/gcs/planning-mission.png)

## 执行任务

切换到执行标签。地图上任务将会被可视化。点击当前的飞行模式以切换到 MISSION 模式并且点击 DISARMED 解锁无人机。如果无人机当前已经处于飞行状态下，它将会飞向任务中的第一段航程，然后跟随航线。

![](images/gcs/flying-mission.png)

## 设置参数

切换到安装标签。滚动左边的菜单直到底部，点击参数图标。当双击时参数可以被修改，这时会打开一个含有更详细解释的快捷菜单去编辑参数。

![](images/gcs/setting-parameter.png)

## 安装

可以在这个[网站](http://qgroundcontrol.com/downloads)下载 QGroundControl。

<aside class="tip">
Developers are advised to use the latest daily build instead of the stable release.
建议开发者使用最新的日常版本，而不是稳定版。
</aside>

## 利用源码编译

为了能够有和飞控代码相匹配的最新版本，推荐固件开发者使用源码编译。

根据 [QGroundControl 编译说明](https://github.com/mavlink/qgroundcontrol#obtaining-source-code)下载 QT 和 编译源代码。
