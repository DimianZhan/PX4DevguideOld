# 模拟GPS
这一章告诉你如何用动作捕捉软件(mocap)去模拟GPS 。

步骤大致如下:
要有一台"VICON电脑"，安装了相关的软件 + [ROS](http://www.ros.org/) + [vicon_bridge](https://github.com/ethz-asl/vicon_bridge)，并且将数据通过网络发送到你自己的电脑。
在你自己的电脑安装好ROS 和 MAVROS。 MAVROS 有一个脚本可以把动作捕捉数据(mocap)模拟生成GPS数据。
你自己的电脑通过数传电台把数据发送到pixhawk。
*注*: VICON电脑和你自己的电脑可以是同一台。

## 准备工作
* 动作捕捉系统(MOCAP system) (在本例中, 用到的是VICON)
* 一台安装好 ROS, MAVROS 和 Vicon_bridge的电脑
* 3DR 数传电台

## 过程
### 第一步
确定你自己的电脑和"VICON 电脑" 在同一个网络当中(或者你需要一个无线网卡)。
在"VICON 电脑"中创建两个文件: "launch_fake_gps.sh" 和 "launch_fake_gps_distorted.sh" 

在"launch_fake_gps.sh"文件中添加以下两行代码，同时把 xxx.xxx.x.xxx 替换成你自己电脑的IP地址 (你可以在终端运行ifconfig来获取).
```sh
export ROS_MASTER_URI=http://xxx.xxx.x.xxx:11311
roslaunch vicon_bridge vicon.launch $@
```

接下来, 在"launch_fake_gps_distorted.sh" 文件中添加下面两行代码，然后把 xxx.xxx.x.xxx 替换成你自己电脑的IP地址。
```sh
export ROS_MASTER_URI=http://xxx.xxx.x.xxx:11311
rosrun vicon_bridge tf_distort $@
```

在动作捕捉系统中标记好你的飞行器，并创建一个模型(下面称为 yourModelName).

### 第二步
在你自己的电脑中运行：
```sh
$ roscore
```



### 第三步
 在"VICON电脑"中创建上述文件的目录下打开两个不同的终端并分别运行
```sh
$ sh launch_fake_gps.sh
```
和
```sh
$ sh launch_fake_gps_distorted.sh
```



### 第四步
在你自己的电脑中运行
```sh
$ rosrun rqt_reconfigure rqt_reconfigure
```
，会打开一个新的会话窗口，并选择"tf_distort"。通过这个工具，你可以编辑参数来修正动作捕捉数据。

我们使用以下参数来模拟GPS:
* publish rate = 5.0Hz
* tf_frame_in = vicon/yourModelName/yourModelName (例如: vicon/DJI_450/DJI_450)
* delay = 200ms
* sigma_xy = 0.05m
* sigma_z = 0.05m


### 第五步
使用QGroundControl连接你的 pixhawk. 选择 PARAMETERS -> System， 修改 SYS_COMPANION 为 257600 (使能 magic 模式;)).

接下来, 选择 PARAMETERS -> MAVLink，修改 MAV_USEHILGPS 为 1 (使能 HIL GPS ).

现在, 选择 PARAMETERS -> Attitude Q estimator， 修改 ATT_EXT_HDG_M 为 2 (通过动作捕捉来获取航向)。

接下来但不是最后, 选择 PARAMETERS -> Position Estimator INAV ，修改 change INAV_DISAB_MOCAP 为 1 (禁止从动作捕捉数据当中进行运动估计).

*注*: 如果你找不到以上说的参数, 可能在 PARAMETERS -> default Group 当中找到


### 第六步
接下来, 打开 "mocap_fake_gps.cpp"。 你可以在这里找到: yourCatkinWS/src/mavros/mavros_extras/src/plugins/mocap_fake_gps.cpp

在
```sh
mocap_tf_sub = mp_nh.subscribe("/vicon/DJI_450/DJI_450_drop", 1, &MocapFakeGPSPlugin::mocap_tf_cb, this);
```
中把 DJI_450/DJI_450 替换成你自己的模型名字 (例如： /vicon/yourModelName/yourModelname_drop). 这里的 "_drop" 会在后面解释。


### 第七步
在第五步中, 我们使能了从动作捕捉帧中选择航向。所以pixhawk不会使用原始的北东方向，而是使用了动作捕捉系统里面的航向。由于3DR数传电台的数率不是特别快，所以我们必须限制动作捕捉数据的频率，运行
```sh
$ rosrun topic_tools drop /vicon/yourModelName/yourModelName 9 10
```
意思是, 从ros topic /vicon/yourModelName/yourModelName 来的 10条消息中的9条都会被删除并发布在 "/vicon/yourModelName/yourModelName_drop" 主题之下。


### 第八步
通过3DR无线数传连接 pixhawk 的 TELEM2 接口，并且使用 USB 连接你的电脑。


### 第九步
进入 catkinWS 目录，并运行
```sh
$ catkin build
```
再运行
```sh
$ roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB0:57600
```
就是这么简单! 你的 pixhawk 现在可以收到GPS数据并且绿灯闪。
