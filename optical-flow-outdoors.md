# 光流的室外应用
----------------------------------------------------

这个页面将展示如何使用 PX4Flow 来进行位置估计和室外自动飞行。激光雷达（ LIDAR ）不是必要的但是会带来性能提升。

## 选择 LPE 估计器
--------------------------------------------------------

经过测试，LPE是唯一一个适用于基于光流的自动飞行的估计器。

使用 `SYS_MC_EST_GROUP = 1` 来选取和重置估计器。


## 硬件
--------------------------------------------------------
![](images/hardware/px4flow_offset.png)

*图 1：固定坐标系（与后文参数相关）*

![](images/hardware/px4flow.png)

*图 2： PX4Flow 光流处理器（摄像头和超声波测距）*

PX4Flow 需要对准地面并通过 I2C 接口和 pixhawk 连接。为了达到最优性能，请尽可能将 PX4Flow 安装在一个好的位置上并尽量隔震。（推荐放置于四旋翼的下方）

**注意：默认情况下，PX4Flow 的超声波指向（+Y方向）飞行器坐标系的+X方向（前向）。如果不是，你需要根据实际情况设置 SENS_FLOW_ROT 。**

![](images/hardware/lidarlite.png)

*图 3：激光雷达产品：Lidar Lite*

兼容的的激光雷达模块包括： Lidar-Lite (尚未量产) 和 [sf10a](http://www.lightware.co.za/shop/en/drone-altimeters/33-sf10a.html)。如何连接 LIDAR-Lite 请参考： [此页](https://pixhawk.org/peripherals/rangefinder?s[]=lidar) . sf10a 可以通过串口连接到 PX4 上。

![](images/hardware/flow_lidar_attached.jpg)

*图 : PX4Flow/ Lidar-Lite 安装在 DJI F450 四轴飞行器上*

![](images/flow/flow_mounting_iris.png)

*图 ：一架搭载了采用 LPE 估计器的 PX4Flow 模块（无激光雷达）的 Iris+ 四轴飞行器*

![](images/flow/flow_mounting_iris_2.png)

*图 ：一种对光流单元的防护方式。同时也有泡沫包住超声波单元以减小超声波单元采集的噪声，泡沫材料还能保护镜头模组。*


### 摄像头对焦

为了保证光流数据的质量，在特定高度处对光流相机进行对焦是非常重要的。对焦时，将一个表面有字的物体（比如一本书）放在地面上，然后通过usb连接 PX4Flow 并运行 QGroundControl。在设置界面里，选择 PX4Flow，你将看到相机画面。通过旋扭调节相机镜头的松紧进而找到焦点位置。

**注意：如果飞行高度超过 3 米，相机将对焦于无穷远处，因而飞更高时无需调整焦距。**

![](images/flow/flow_focus_book.png)

*图 ：使用书来在一定高度对焦相机，高度一般为 1-3 米。超过 3 米，相机将对焦于无穷远*


![](images/flow/flow_focusing.png)

*图 ：QGroundControl中的 px4flow 界面，可以用来对焦相机*

### 传感器参数

在 QGroundControl 中可以改变的参数有：
* SENS_EN_LL40LS
	设置为1时将激活 lidar-lite 测距模块
* SENS_EN_SF0X
	设置为1时将激活 Lightware 公司的测距模块（比如 sf02 和 sf10a）

## 局部定位估计器（LPE）
--------------------------------------------------------

LPE 是一个基于扩展卡尔曼滤波器的状态和速度估计器。它采用惯性导航并且和后文的 INAV 估计器相似，但是 LPE 会基于状态协方差动态地计算卡尔曼增益。LPE 能检测错误数据，这一点对于超声波模块的使用非常有帮助。因为当遇到软表面时，超声波的返回信号可能为空。

### 户外飞行视频
{% youtube %}https://www.youtube.com/watch?v=Ttfq0-2K434{% endyoutube %}


下面的轨迹点图取自一个使用光流模块的户外自动飞行视频。GPS 数据仅作为对照，并没有在飞控中被使用。GPS 和光流定位的数据之间的差值来源于初始位置的误差。初始位置假设位于 LPE_LAT 和 LPE_LON。

![](images/lpe/lpe_flow_vs_gps.png)

*图 4：基于采用LPE估计器的光流/超声模块的飞行活动数据*


### 参数

当传感器接入时，LPE 将自动融合激光雷达（LIDAR）和光流传感器的数据。


* LPE_FLOW_OFF_Z - 光流相机到飞行器质心的偏移，向下为正，默认为0。对于大部分典型配置可以设置为0。
* LPE_FLW_XY - 光流测量标准偏差（单位米）。
* LPW_FLW_QMIN - 测量值被接受时的最低数据质量.
* LPE_SNR_Z - 超声波测量标准偏差（单位米）。
* LPE_SNR_OFF_Z - 超声单元到飞行器质心的偏移。
* LPE_LDR_Z - 激光雷达标准偏差（单位米）。
* LPE_LDR_Z_OFF - 激光雷达到飞行器质心的偏移。
* LPE_GPS_ON - 如果 LPE_GPS_ON 设置为1，飞行器将无法在无GPS情况下飞行。无GPS时，该参数必须设为0，否则位置初始化将在等到 GPS 信号后才能进行。这是因为 GPS 高度初始化的优先级高于气压计高度初始化。

**注意：无 GPS 时，LPE_GPS_ON 必须设置为0。**

### 自动飞行参数

*告诉飞行器自己在那儿*

* LPE_LAT - 局部坐标系下，相对于（0,0）的纬度。
* LPE_LON - 局部坐标系下，相对于（0,0）的经度。

*使飞行器低空低速飞行*

* MPC_ALT_MODE - 设置为1可以激活地形跟随。
* LPE_T_Z - 这是地形噪声参数，如果环境多山，设为0.1；如果环境地面平整，设为0.01。
* MPC_XY_VEL_MAX - 设置为2以限制倾斜。
* MPC_XY_P - 调低为大约0.5以限制倾斜。
* MIS_TAKEOFF_ALT - 设置为2可以进行低空起飞。

*航点*

* 在3米及以下高度创建航点。
* 不要创建特别长距离的飞行平面，一般情况下每飞100米会产生1米的漂移。

**注意：在你第一次自动飞行时，建议先手动飞一遍航线，以确认光流处理器测量的和预期一致**
