# 架构概述

PX4主要分两层：一个自动驾驶仪软件解决方案[PX4飞行栈](concept-flight-stack.md)和一个支持所有类型自主机器人的通用机器人中间件[PX4中间件](concept-middleware.md)。

所有[机身](airframes-architecture.md)，事实上所有机器人系统包括船都共享一个单一的代码库。 完整的系统设计是[活性的](http://www.reactivemanifesto.org)，这意味着：

  * 所有功能都被划分成了可交换组件。
  * 通信是通过异步消息传递完成的。
  * 系统可以处理不同的工作负载。

除了这些运行时的考量外，它的模块设计实现了最大化的[可重用性](https://en.wikipedia.org/wiki/Reusability)。

## 高级软件架构
 
下图的每个方块都是一个独立的模块。它的代码，依赖性和运行时都是自包含的。 每个箭头代表了一个通过[uORB](advanced-uorb.md)来进行发布/订阅调用的链接。

> ** 信息 ** PX4的架构允许非常快速和方便地交换每一个块，即使在运行时。
 
控制器/混合器专用于特定的机身（例如多机，垂直起降机（VTOL）或飞机），但诸如 `commander `和 `navigator `这样的高级任务管理模块是在平台之间共享的。

![架构](images/diagrams/PX4_Architecture.png)

> ** 信息 ** 该图可以在 [这里](https://drive.google.com/file/d/0Byq0TIV9P8jfbVVZOVZ0YzhqYWs/view?usp=sharing) 更新，请打开 draw.io 进行编辑.

## 与地面控制站通信的体系架构
 

与地面控制站（GCS）的交互是通过“业务逻辑”应用程序来完成的，包括Commander（一般命令和控制，例如武装），Navigator（接受任务并将其转换为较低级别的导航原语（navigation primitives）） 以及mavlink应用，它接受MAVLink数据包并将其转换为板载uORB数据结构。 这种分离是明确设计的，以避免在系统中包含对MAVLink依赖。 MAVLink应用程序还使用大量的传感器数据和状态估计值，并将它们发送到地面控制站。

{% mermaid %}
graph TD;
  mavlink---commander;
  mavlink---navigator;
  position_estimator-->mavlink;
  attitude_estimator-->mavlink;
  mixer-->mavlink;
{% endmermaid %}
