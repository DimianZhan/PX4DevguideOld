# MAVROS 板外控制例程

<aside class="caution">
对飞行器的板外控制 (OFFBOARD) 非常危险，如果你要在真实的载具上尝试，请一定确保有切换回手动控制 (MANUAL) 的办法，这样可以在危险发生前及时阻止，避免情况变得更糟糕。
</aside>

接下来的教程将基于 Gazebo 软件下的 Iris 飞机仿真环境，来运行一个基本的板外控制程序。执行此程序，将会控制四旋翼起飞至2米高，即视频中所展示的内容。

<video width="100%" autoplay="true" controls="true">
	<source src="images/sim/gazebo_offboard.webm" type="video/webm">
</video>

## 例程
Create the offb_node.cpp file in your ros package and paste the following inside it:
```C++
/**
 * @file offb_node.cpp
 * @简单的控制例程, 基于 mavros v0.14.2, px4 固件编写， 以 SITL 方式使用 Gazebo 测试
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    // 此处 setpoint 话题的发布频率必须大于 2Hz
    ros::Rate rate(20.0);

    // 等待 FCU 连接
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    // 在调用服务前先发送若干指令
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

```
## 例程讲解
```C++
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
```
`mavros` 栈的 `mavros_msgs` 包提供了在 ROS 下用于订阅话题和请求服务的消息类型，所有服务和话题以及相应的消息类型都记录在 [mavros wiki](http://wiki.ros.org/mavros) 中。

```C++
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
```
我们通过编写一个回调函数来接收并保存自驾仪的状态，用于后文中的判断连接状态、发送解锁指令和切换板外控制 (OFFBOARD) 飞行模式。

```C++
ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
```
接下来我们再发布一项话题、调用两项服务，发布的话题用于命令自驾仪飞行至指定位置，调用的服务分别用于解锁和切换飞行模式。需要注意的是，在您的电脑中，话题的 `mavros` 前缀可能和此处不同，这个前缀取决于你在 ROS 下 launch 文件中的设置。
```C++
// 此处 setpoint 话题的发布频率必须大于 2Hz
ros::Rate rate(20.0);
```
PX4 固件规定板外控制命令的超时时间为500毫秒，如果发生超时，自驾仪将会切换至进入板外控制前的那个模式，因此控制命令的发布频率**必须**大于 2 Hz 以避免超时。这也是推荐从姿态控制模式 (POSCTL) 进入板外控制 (OFFBOARD) 模式的原因，这样一来，如果飞行中从板外控制模式中退出，飞行器将会进入姿态控制模式，保持原先的位置并盘旋。

```C++
// 等待 FCU 连接
while(ros::ok() && current_state.connected){
    ros::spinOnce();
    rate.sleep();
}
```
Before publishing anything, we wait for the connection to be established between mavros and the autopilot. This loop should exit as soon as a heartbeat message is received.
```C++
geometry_msgs::PoseStamped pose;
pose.pose.position.x = 0;
pose.pose.position.y = 0;
pose.pose.position.z = 2;
```
尽管 PX4 固件实际在 NED 坐标系中控制飞机，但 mavros 将会帮我们把坐标变换到 ENU 下，或变换回 NED。这里我们将 Z 设置为2。
```C++
// 在调用服务前先发送若干指令
for(int i = 100; ros::ok() && i > 0; --i){
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
}
```
Before entering offboard mode, you must have already started streaming setpoints otherwise the mode switch will be rejected. Here, 100 was chosen as an arbitrary amount.
```C++
mavros_msgs::SetMode offb_set_mode;
offb_set_mode.request.custom_mode = "OFFBOARD";
```
我们将 `custom_mode` 设置为 `OFFBOARD`。 你可以访问 [支持的飞行模式](http://wiki.ros.org/mavros/CustomModes#PX4_native_flight_stack) 获取更多信息。
```C++
mavros_msgs::CommandBool arm_cmd;
arm_cmd.request.value = true;

ros::Time last_request = ros::Time::now();

while(ros::ok()){
		if( current_state.mode != "OFFBOARD" &&
				(ros::Time::now() - last_request > ros::Duration(5.0))){
				if( set_mode_client.call(offb_set_mode) &&
						offb_set_mode.response.success){
						ROS_INFO("Offboard enabled");
				}
				last_request = ros::Time::now();
		} else {
				if( !current_state.armed &&
						(ros::Time::now() - last_request > ros::Duration(5.0))){
						if( arming_client.call(arm_cmd) &&
								arm_cmd.response.success){
								ROS_INFO("Vehicle armed");
						}
						last_request = ros::Time::now();
				}
		}

		local_pos_pub.publish(pose);

		ros::spinOnce();
		rate.sleep();
}
```
其余的代码只是简单示意，在用户解锁后尝试切换为板外控制。我们将服务请求间隔设置为5秒，以避免请求过多挤爆自驾仪。接下来的代码继续以合适的频率发送控制指令。

<aside class="tip">
为了简明易懂，这里的例程经过精简再精简，但在实际的完整工程中，更常见的做法是使用多线程周期地发布话题。
</aside>
