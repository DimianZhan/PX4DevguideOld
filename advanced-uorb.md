# uORB 消息机制

## 简介

uORB（*微型对象请求代理*）是一个异步的发布／订阅消息机制的 API，用来实现线程或进程间的通讯。 

请参照[教程](tutorial-hello-sky.md) 学习如何在 C++ 中使用该功能。

由于很多应用依赖该消息机制，uORB 会在系统启动时自动开始运行。
 `uorb start` 命令会启动该消息机制。使用 `uorb test` 命令可以启动单元测试。

## 添加一个新主题

为了添加一个新主题，你需要在 `msg/` 路径下添加一个新的 `.msg` 文件，并将该文件名添加到 `msg/CMakeLists.txt` 列表中。通过以上步骤，系统将会自动生成 C/C++ 代码。

让我们来看看现有的 `msg` 文件所支持的类型。一条消息可以被嵌套在其他消息内。
对于每一个生成的 C/C++ 结构，都会包含一个 `uint64_t timestamp` 成员。这个成员会被日志模块使用，所以如果要将消息记录进日志，请确保该成员被赋值。

要在代码中使用该消息，需要包含头文件：

```
#include <uORB/topics/topic_name.h>
```

在 `.msg` 文件中添加类似下面这一行，一条单一的消息定义可以被多个独立的主题实例使用。

```
# TOPICS mission offboard_mission onboard_mission
```
然后在代码中，通过如下的主题 ID 来使用新定义的主题：`ORB_ID(offboard_mission)`.


## 消息发布 (Publishing)

消息发布可以在系统的任何地方实现，包括中断上下文（通过 `hrt_call`  API 调用的函数）。然而公告消息 (advertising) 则只能在中断上下文外部实现。在同一个进程中，一条主题被发布之前，必须先公告。


## 主题列表与监听

<aside class="note">'listener'（监听）命令只能在 Pixracer (FMUv4) 和 Linux / OS X 平台上使用。
</aside>

通过列出文件句柄来查看主题列表：

```sh
ls /obj
```

调用 listener 来监听同一个主题的 5 条消息的内容：

```sh
listener sensor_accel 5
```

输出为若干次该主题的内容：

```sh
TOPIC: sensor_accel #3
timestamp: 84978861
integral_dt: 4044
error_count: 0
x: -1
y: 2
z: 100
x_integral: -0
y_integral: 0
z_integral: 0
temperature: 46
range_m_s2: 78
scaling: 0

TOPIC: sensor_accel #4
timestamp: 85010833
integral_dt: 3980
error_count: 0
x: -1
y: 2
z: 100
x_integral: -0
y_integral: 0
z_integral: 0
temperature: 46
range_m_s2: 78
scaling: 0
```


