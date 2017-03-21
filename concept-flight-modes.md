# Flight Modes
# 飞行模式
**Flight Modes** define the state of the system at any given time.  The user transitions between flight modes via switches on the remote control or the [ground control station](qgroundcontrol-intro.md).
**飞行模式**定义了系统在任何时间的状态。用户可以使用远程遥控器或者地面站来切换飞行模式。
## Flight Mode Quick Summary
## 飞行模式简要说明
  * **_MANUAL_**
  * **_手动模式_**
    * **Fixed wing aircraft/ rovers / boats:** 
    * **固定翼飞行器/ 越野车 / 船舶：**
        * **MANUAL:** The pilot's control inputs are passed directly to the output mixer.
        * **手动模式：** 飞手的控制输入直接传递到输出混控器。
        * **STABILIZED:** The pilot's inputs are passed as roll and pitch *angle* commands and a manual yaw command.
        * **稳定性：** 飞手的手动输入可以作为翻滚角、俯仰角和偏航角命令。
    * **Multirotors:**
    * **多旋翼飞行器：**
        * **ACRO:** The pilot's inputs are passed as roll, pitch, and yaw *rate* commands to the autopilot.  This allows the multirotor to become completely inverted.  Throttle is passed directly to the output mixer
        * **特技模式：** 飞手的手动操作转换为翻滚、俯仰和偏航角速度命令传送给自驾仪。这个模式下多旋翼飞行器可以完全翻转。油门直接传递到输出混控器。
        * **RATTITUDE** The pilot's inputs are passed as roll, pitch, and yaw *rate* commands to the autopilot if they are greater than the mode's threshold.  If not the inputs are passed as roll and pitch  *angle* commands and a yaw *rate* command.  Throttle is passed directly to the output mixer.
        * **混合模式：** 如果飞手的输入超过了设定的阈值，则将其转换成翻滚、俯仰、偏航角速度命令传送给自驾仪；如果输入没有超过阈值，则将其转换成翻滚、俯仰转角度以及偏航角速度命令。油门直接输出到混控器。
        * **ANGLE** The pilot's inputs are passed as roll and pitch *angle* commands and a yaw *rate* command.  Throttle is passed directly to the output mixer.
        * **角度模式：** 飞手的手动操作转换为翻滚、俯仰角度命令和偏航角速度命令。油门直接输出到混控器。
  * **_ASSISTED_**
  * **_辅助模式_**
    * **ALTCTL**
    * **高度控制**
      * **Fixed wing aircraft:** When the roll, pitch and yaw inputs (RPY) are all centered (less than some specified deadband range) the aircraft will return to straight and level flight and keep its current altitude. It will drift with the wind.
      * **固定翼飞行器：** 当翻滚、俯仰、偏航输入（RPY）都居中时（不超过一些特定的死区范围），飞行器将回到直线水平的飞行状态然后保持当前的高度。飞行器会随着风漂移。
      * **Multirotors:** Roll, pitch and yaw inputs are as in MANUAL mode. Throttle inputs indicate climb or sink at a predetermined maximum rate. Throttle has large deadzone.
      * **多旋翼飞行器：** 翻滚、俯仰、偏航输入与手动模式相同。油门输入表明以预定的最大速率上升或下降。油门死区很大。
    * **POSCTL**
    * **水平控制**
      * **Fixed wing aircraft:** Neutral inputs give level, flight and it will crab against the wind if needed to maintain a straight line.
      * **固定翼飞行器：** Neutral inputs 可使飞行器水平飞行，如果要保持沿直线飞行，飞机将会逆风飞行。
      * **Multirotors** Roll controls left-right speed, pitch controls front-back speed over ground. When roll and pitch are all centered (inside deadzone) the multirotor will hold position. Yaw controls yaw rate as in MANUAL mode. Throttle controls climb/descent rate as in ALTCTL mode.
      * **多旋翼飞行器：** 横滚控制飞机左右运动的速度，俯仰控制飞机前后运动的速度，速度都是以地面为参考系的。当横滚和俯仰的遥杆都居中时（在死区内），飞机会稳定不动。偏航控制的是偏航角速度，这一点跟手动模式相同。油门控制的是上升/下降速度，与高度控制模式相同。
  * **_AUTO_**
  * **_自动模式_**
    * **AUTO_LOITER**
    * **盘旋模式**
        * **Fixed wing aircraft:** The aircraft loiters around the current position at the current altitude (or possibly slightly above the current altitude, good for 'I'm losing it'). 
        * **固定翼飞行器：** 飞行器在当前位置保持当前高度（或者稍微高于当前高度，但比失去控制好）。
        * **Multirotors:**  The multirotor hovers / loiters at the current position and altitude.
        * **多旋翼飞行器：** 飞行器在当前的位置、高度悬停 / 盘旋。
    * **AUTO_RTL**
    * **返航模式**
        * **Fixed wing aircraft:** The aircraft returns to the home position and loiters in a circle above the home position.
        * **固定翼飞行器：** 飞行器返回“家”的位置然后在“家”的上方绕圈保持盘旋。
        * **Multirotors:** The multirotor returns in a straight line on the current altitude (if higher than the home position + loiter altitude) or on the loiter altitude (if higher than the current altitude), then lands automatically.
        * **多旋翼飞行器：** （如果飞机当前的高度高于起点位置的高度 + 悬停高度，）则飞机将沿直线返回当前位置；（如果飞机的悬停高度高于当前高度，）则飞机将沿直线返回悬停位置；然后自动着陆。
    * **AUTO_MISSION**
    * **任务模式**
        * **All system types:** The aircraft obeys the programmed mission sent by the ground control station (GCS). If no mission received, aircraft will LOITER at current position instead.
        * **所有系统类型：** 飞机按照地面控制台（GCS）发送的程序指令运动。如果没有接收到指令，飞机会在当前位置悬停。
  * **_OFFBOARD_**
  * **_外部控制_**
    In this mode the position, velocity or attitude reference / target / setpoint is provided by a companion computer connected via serial cable and MAVLink. The offboard setpoint can be provided by APIs like [MAVROS](https://github.com/mavlink/mavros) or [Dronekit](http://dronekit.io).
在这个模式下，飞机的位置，速度或者姿态的参考 / 目标 / 设定值由另一台通过串行线路与MAVLink连接的电脑提供。这些外部的设定值可以由MAVROS或者Dronekit这种应用程序接口提供。
## Flight Mode Evaluation Diagram
## 飞行模式评估图
![](images/diagrams/commander-flow-diagram.png)
