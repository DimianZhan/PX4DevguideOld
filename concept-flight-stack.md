# PX4飞行栈 
PX4飞行栈是自主无人机引导、导航和控制算法的集合。它包含了固定翼、多旋翼和垂直起降（VTOL）机型的控制器以及位姿估测器。


## 估测和控制架构
下图为一个典型模块实现的示例。根据机型需要，其中的一些模块也可单独组成应用系统（例如，当某个特定机型需要模型预测器时）。

{% mermaid %}
graph TD;
  pos_ctrl-->att_ctrl;
  att_ctrl-->mixer;
  inertial_sensors-->attitude_estimator;
  inertial_sensors-->position_estimator;
  GPS-->position_estimator;
  computer_vision-->position_estimator;
  position_estimator-->navigator;
  position_estimator-->attitude_estimator;
  position_estimator-->pos_ctrl;
  attitude_estimator-->att_ctrl;
{% endmermaid %}
