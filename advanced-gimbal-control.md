# 云台控制设置

PX4 包含一个拥有多种输入输出方式的通用挂载/云台控制驱动器。任何输入都可以用来驱动任意的输出。

首先，使用 ‘vmount start’ 确保驱动器在运行，然后设置参数。

## 参数
所有的参数在 [src/drivers/vmount/vmount_params.c](https://github.com/PX4/Firmware/blob/master/src/drivers/vmount/vmount_params.c) 中有介绍。
其中最重要的就是输入（‘ MNT_MODE_IN ’）和输出( 'MNT_MODE_OUT' )模式。通常输入是不可用的。任何输入方式都可以用来驱动任意的可用输出。

如果是通过 mavlink 模式输入的话，手动遥控器输入也是开启的（‘ MNT_MAN_CONTROL ’）。它会一直保持开启，直到没有接受到 mavlink 信号或者 mavlink 直
接请求遥控器输入模式。



### 为辅助输出配置云台的混控

云台使用 #2 控制组合（见 [Mixing and actuators](concept-mixing.md) ）。下面是混控设置：

```
# 横滚
M: 1
O:      10000  10000      0 -10000  10000
S: 2 0  10000  10000      0 -10000  10000

# 俯仰
M: 1
O:      10000  10000      0 -10000  10000
S: 2 1  10000  10000      0 -10000  10000

# 航向
M: 1
O:      10000  10000      0 -10000  10000
S: 2 2  10000  10000      0 -10000  10000
```

添加你需要的设置参数到你的主要和辅助混控器。

## 测试
驱动器提供了一个简单的测试命令 —— 它首先需要通过 ‘vmount stop’ 来停止驱动器。下面是在软件在环（ SITL）环境中进行的测试，但是命令仍然是运行在真实的硬件
上的。

从
```
make posix gazebo_typhoon_h480
```
开始仿真（不需要改变任何参数配置），
确保在上锁状态，例如使用‘ commander takeoff ’，然后使用例如
···
vmount test yaw 30
···
命令来控制云台。需要注意，仿真的云台在自稳模式，所以如果通过 mavlink 命令的话，设置‘ stabilize ’标志位来退出自稳模式。

![Gazebo Gimbal Simulation](images/gazebo-gimbal-simulation.png)
