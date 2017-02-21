# 初始配置

在开始开发PX4之前，系统应当以默认配置进行初始配置，以确保硬件已经正确配备和测试。下方视频讲解了[Pixhawk硬件](hardware-pixhawk.md)与[QGroundControl](qgroundcontrol-intro.md)的安装过程。[此链接](airframes-architecture.md)为已支持的参考机架列表。

> **注意** [下载QGroundControl的DAILY BUILD](http://qgroundcontrol.com/downloads)并跟随下方视频教程组装你的飞机。任务规划，飞行与参数设定详见[QGroundControl教程](http://dev.px4.io/qgroundcontrol-intro.html)。

下面的视频介绍了一些设置选项。

{% tencentvideo %}https://v.qq.com/x/page/w0376wesku2.html{% endtencentvideo %}

## 遥控选项

PX4飞行栈并不强制要求有遥控系统。它也不强制要用一个独立的开关来选择飞行模式。

### 无遥控飞行

所有的遥控安装检查可以通过将`COM_RC_IN_MODE`设为`1`来禁用。这将会不允许手动模式飞行， but e.g. flying in 

### 单通道模式切换开关

替代使用多个开关的模式，在该模式下，系统接受单个通过作为模式切换开关。在[旧版wiki](https://pixhawk.org/peripherals/radio-control/opentx/single_channel_mode_switch)中对此有解释。

