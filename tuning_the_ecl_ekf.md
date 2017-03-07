# 使用 ecl EKF

本节教程回答了使用 ECL EKF 算法相关的常见问题。

## 什么是 ecl EKF?

ECL \(Estimation and Control Library，估计和控制库\) 使用扩展卡尔曼滤波算法来处理传感器测量值并提供以下状态的估计:

* 四元数，定义了从North（北），East（东），Down（地）地球坐标系到X，Y，Z机体坐标系的旋转。
* IMU中的速度 - North,East,Down \(m/s\)
* IMU中的位置 - North,East,Down \(m\)
* IMU 角度增量偏差（bias）估计 - X,Y,Z \(rad\)
* IMU 速度增量偏差（bias）估计 - X,Y,Z\(m/s\)
* 地球磁场分量 - North,East,Down \(gauss\)
* 机体坐标系磁场偏差 - X,Y,Z \(gauss\)
* 风速 - North,East \(m/s\)

EKF在一个延迟的‘融合时间范围（fusion time horizon）’内工作，这是考虑到涉及到IMU的每个测量的时间延迟不同。每个传感器的数据是以FIFO（先进先出）形式缓存的，然后被EKF取出在正确的时间使用。EKF2\_\*\_DELAY 控制了每个传感器的延时补偿。

互补滤波用于将状态从‘融合时间范围’向当前时间推进，它使用IMU的数据。互补滤波的时间常数由参数EKF2\_TAU\_VEL 和 EKF2\_TAU\_POS控制。

注意：‘融合时间范围’的延迟和缓存的长度由 EKF2\_\*\_DELAY 中的最大值决定。如果一个传感器没有被使用，推荐将其时间延迟设置为0。降低‘融合时间范围’的延迟就降低了互补滤波中用于将状态推荐到当前时间的误差（error）。

位置和速度状态在被输出至控制回路之前，需要调整以说明IMU和机体坐标系的偏移（offset）。IMU相对于机体坐标系的位置可由参数 EKF2\_IMU\_POS\_X,Y,Z 设置。

EKF使用IMU仅用于状态预测。在EKF推到中IMU没有被用作观测量。协方差预测、状态更新和协方差更新的代数方程由 Matlab symbolic toolbox 生成，可以在这里找到: [Matlab Symbolic Derivation](https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m)

## 使用了什么传感器测量量？

EKF包含了不同的操作模式，考虑到了不同的传感器测量值得组合。在启动时，滤波器检测最小可用传感器组合，在初始倾斜、偏航和高度校准完成后，进入了一个提供旋转、垂直速度、垂直位置、IMU角度增量偏差和IMU速度增量偏离估计的模式。

这个模式需要IMU数据、一个偏航数据来源\(磁力计或者外部视觉信息\)和一个高度数据来源。所有EKF操作模式都需要这个最小数据集。其他传感器数据可用于估计其他的状态。

### IMU

* 机体三轴固连的惯性测量单元（Inertial Measurement unit）以最低100Hz提供角度增量和速度增量的数据。注意：在IMU角度增量数据被EKF使用之前应进行锥进修正。

### 磁力计

三轴机体固连的磁力计数据（或者外部视觉系统姿态数据）要求至少以5Hz更新。磁力计数据可用以下两种方式使用：

* 使用倾斜估计和磁力计偏差可将磁力计测量值转换为偏航角。这个偏航角接着被EKF用作观测量。这个方法更不准确而且没有考虑到机体坐标系磁场偏移，但是它对磁场异常和启动时大的陀螺仪偏差更加健壮（robust）。这是在启动时和在地面上时使用的默认方法。
* XYZ 磁力计读数被用做独立的观测量。这个方法更加准确而且允许习得机体坐标系磁场偏移，但是需假设地球磁场环境缓慢变化而且在有明显外部磁场干扰的时候表现欠佳。飞机在空中或者爬升超过1.5m高度之后默认使用这个方法。

选择模式的逻辑由参数 EKF2\_MAG\_TYPE 设置。

### 高度

一个高度数据源-至少以5Hz更新的GPS、气压、测距仪或者是外部视觉。注意：高度数据的主要来源受到参数 EKF2\_HGT\_MODE 的控制。

如果这些测量值不存在，EKF就不会启动。当这些测量值被检测到后，EKF会初始化状态并完成倾斜和偏航的校准。当倾斜和对其完成后，启用额外传感器的数据，EKF会接着转换到其他操作模式。

### GPS

如果以下条件满足，GPS测量值将会用于位置和速度：

* 启用GPS，通过设置参数 EKF2\_AID\_MASK。
* GPS质量检测通过。这些检测被参数 EKF2\_GPS\_CHECK and EKF2\_REQ&lt;&gt; 控制。
* GPS高度可以被EKF直接使用，通过设置参数 EKF2\_HGT\_MODE。

### 测距仪

测距仪到地面的距离被一个单独的状态估计使用来估计相对于高度基准的地形的垂直位置。

如果在一个可以当做零高度基准平面上操作，通过把参数 EKF2\_HGT\_MODE 设置为2，测距仪也可以直接被EKF使用用于估计高度。

### 风速

通过把 EKF2\_ARSP\_THR 设置为一个正值，等效风速（EAS）数据可用于估计风速数据和降低GPS丢失时的偏移。当风速数据超过 EKF2\_ARSP\_THR 的正值设定的阈值，而且飞机类型不是旋翼时，将启用风速数据。

### 合成侧滑（Synthetic Sideslip）

固定翼平台可以利用一个假设的零侧滑观测量来提升风速估计并在没有风速传感器的情况下启用风速估计。这可以通过把参 EKF2\_FUSE\_BETA 设置为1来使能。

### 光流

如果以下条件满足将使用光流数据：

* 有效的测距仪数据可用。
* 参数 EKF2\_AID\_MASK 的比特位1为true。
* 光流传感器返回的公制质量大于参数 EKF2\_OF\_QMIN 设定的最小要求。

###外部视觉系统

例如 Vicon这样的外部视觉系统的位置和姿态测量在以下状况下可用：

* 外部视觉系统的水平位置数据将启用，如果参数 EKF2\_AID\_MASK 的比特位3是为true。
* 外部视觉系统的垂直位置数据将启用，如果参数 EKF2\_HGT\_MODE 设置为3。
* 外部视觉系统的姿态数据将启用，如果参数 EKF2\_AID\_MASK 的比特位4是为true。

## 如何使用 'ecl' library EKF?

将参数 SYS\_MC\_EST\_GROUP 设置为2来使用 ecl EKF。

## ecl EKF 相比于其他估计器来说的优点和缺点是什么？

与所有的估计器一样，性能的大部分来自于匹配传感器特性的调试。调试是精度与鲁棒性的折中，尽管我们尝试提供满足大部分用户需求的调试参数，还是有有一些应用场合需要调试。 

因此没有，没有关于有关之前 attitude\_estimator\_q + local\_position\_estimator 这种组合的精度的声明保证，估计器的最佳选择取决于应用和调谐。

### 劣势

* ecl EKF 是一个复杂的算法，要求对扩展卡尔曼滤波和在导航问题上的应用有很好地理解，才能成功的进行调试。对于那些没有取得较好结果的用户而言，知道修改什么参数就更困难了。
* ecl EKF 使用更多的RAM和flash空间。
* ecl EKF 使用更多的日志空间。
* ecl EKF 飞行测试时间更短。

### 优势

* ecl EKF 能够在数学上连续地融合来自传感器的不同时间延迟和不同速率的数据，一旦延迟参数正确设置之后就能提升动态操控时的精度。
* ecl EKF 能够大范围的不同的传感器类型。
* ecl EKF 检测和报告传感器数据中的统计意义上明显的不一致性，协助诊断传感器误差。
* 对于固定翼运算，ecl EKF 有没有风速传感器都能估计风速，能够联合使用估计风速与风速计测量值和侧滑假设，进而延长飞行中 GPS 丢失时的航位推测时间。
* ecl EKF 估计三轴加速度计的偏差，能提升tailsitter和其他在飞行阶段之间会经历大的姿态变化的飞行器的精度。
* 这种联合结构（混合姿态和位置/速度估计）意味着姿态估计受益于所有传感器的测量。如果调试得当，这应该有提升姿态估计的潜力。

## 如何检测 EKF 性能？

EKF 输出、状态（state）和状态（status）数据被发布到一系列在飞行中记录到SD卡的 uORB 话题中。以下指导假设数据已经被以 .ulog 的文件格式记录。要使用 .ulog 格式，将参数 SYS\_LOGGER 设置为1.

.ulog格式的数据可以通过使用 [PX4 pyulog library](https://github.com/PX4/pyulog) 在python中解析。

大部分 EKF 数据可以在记录到 .ulog 文件中的 [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg) 和 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) uORB 消息中找到。

### 输出数据

* 姿态输出数据在 [vehicle\_attitude](https://github.com/PX4/Firmware/blob/master/msg/vehicle_attitude.msg) 消息中。
* 本地位置输出在 [vehicle\_local\_position](https://github.com/PX4/Firmware/blob/master/msg/vehicle_local_position.msg) 消息中。
* 控制回路反馈数据在 [control\_state](https://github.com/PX4/Firmware/blob/master/msg/control_state.msg) 消息中。
* 全局 \(WGS-84\) 输出数据在 [vehicle\_global\_position](https://github.com/PX4/Firmware/blob/master/msg/vehicle_global_position.msg) 消息中。
* 风速输出数据在 [wind\_estimate](https://github.com/PX4/Firmware/blob/master/msg/wind_estimate.msg) mess消息中。

### 状态


参考 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) 中的ststatesate\[32\]。 states\[32\] 的索引图如下:

* \[0 ... 3\] 四元数
* \[4 ... 6\] 速度 NED \(m/s\)
* \[7 ... 9\] 位置 NED \(m\)
* \[10 ... 12\] IMU 角度增量偏差 XYZ \(rad\)
* \[13 ... 15\] IMU 速度增量偏差 XYZ \(m/s\)
* \[16 ... 18\] 地球磁场 NED \(gauss\)
* \[19 ... 21\] 机体磁场 XYZ \(gauss\)
* \[22 ... 23\] 风速 NE \(m/s\)
* \[24 ... 32\] 未使用

### 状态方差

参考 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) 中的covariances\[28\]（协方差）。 covariances\[28\]的索引图如下:

* \[0 ... 3\] 四元数
* \[4 ... 6\] 速度 NED \(m/s\)^2
* \[7 ... 9\] 位置 NED \(m^2\)
* \[10 ... 12\] IMU 角度增量偏差 XYZ \(rad^2\)
* \[13 ... 15\] IMU 速度增量偏差 XYZ \(m/s\)^2
* \[16 ... 18\] 地球磁场 NED \(gauss^2\)
* \[19 ... 21\] 机体磁场 XYZ \(gauss^2\)
* \[22 ... 23\] 风速 NE \(m/s\)^2
* \[24 ... 28\] 未使用

### 观测新息（Observation Innovations）

* 磁力计 XYZ \(gauss\) : 参考 mag\_innov\[3\] ，位于 [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg)。
* 偏航角 \(rad\) : 参考 heading\_innov ，位于 [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* 速度和位置新息 : 参考 vel\_pos\_innov\[6\] ，位于 [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg)。vel\_pos\_innov\[6\] 的索引图如下:
  * \[0 ... 2\] 速度 NED \(m/s\)
  * \[3 ... 5\] 位置 NED \(m\)
* 真实风速 \(m/s\) : 参考 airspeed\_innov ，位于 [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg)。
* 合成侧滑 \(rad\) : 参考 beta\_innov in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg)。
* 光流 XY \(rad/sec\) : 参考 flow\_innov in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg)。
* 地面上方高度 \(m\) : 参考 hagl\_innov in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg)。

### 观测信息方差

* 磁力计 XYZ \(gauss^2\) : 参考 mag\_innov\_var\[3\] ，位于 [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg)。
* 偏航角 \(rad^2\) : 参考 heading\_innov\_var，位于the ekf2\_innovations message.
* 速度和位置新息 : 参考 vel\_pos\_innov\_var\[6\]，位于[ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg)。 vel\_pos\_innov\[6\] 的索引图如下:
  * \[0 ... 2\] 速度 NED \(m/s\)^2
  * \[3 ... 5\] 位置 NED \(m^2\)
* 真实风速 \(m/s\)^2 : 参考 airspeed\_innov\_var ，位于 [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg)。
* 合成侧滑 \(rad^2\) : 参考 beta\_innov\_var ，位于 [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg)。
* 光流 XY \(rad/sec\)^2 : 参考 flow\_innov\_var ，位于 [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg)。
* 地面上方高度 \(m^2\) : 参考 hagl\_innov\_var ，位于 [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg)。

### 输出互补滤波

输出互补滤波用于将状态从融合时间范围推进到当前时间。要查看融合时间范围内测量的角度、速度和位置追踪误差量级（magnitude），参考 ekf2\_innovations 中的 output\_tracking\_error\[3\]索引图如下：
* \[0\] 角度追踪误差量级 \(rad\)
* \[1\] 速度追踪误差量级 \(m/s\)。 可以通过调整参数 EKF2\_TAU\_VEL来调整速度追踪时间常数。降低这个参数会降低稳态误差，但是会增加NED速度输出的观测噪声数量。
* \[2\] 位置追踪误差量级 \(m\)。   可以通过调整参数 EKF2\_TAU\_POS来调整位置追踪时间常数。降低这个参数会降低稳态误差，但是会增加NED位置输出的观测噪声数量。

### EKF 误差

EKF对于恶劣状态下的状态和协方差更新内置了误差检测。 参考 filter\_fault\_flags ，位于 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg)。

### 观测误差

有两种类型的观测错误:

* 数据丢失。一个例子就是测距仪不能提供返回值。
* 新息，就是状态估计和传感器观测之间的差异超出了。一个例子就是过多的震动导致大的垂直位置误差，导致气压高度测量值被拒绝。

这些都会导致观测数据被拒绝足够长时间而导致EKF尝试使用传感器观测值重置状态。所有的观测都有一个统计置信度检测应用于新息。检测的标准差的数目受到每种观测类型的参数 EKF2\_&lt;&gt;\_GATE 的控制。


测试水平（Test levels）在 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) 中，如下所示:

* mag\_test\_ratio : 最大磁力计新息分量与新息测试限制之比
* vel\_test\_ratio : 最大速度新息分量与新息测试限制之比
* pos\_test\_ratio : 最大水平位置新息分量与新息测试限制之比
* hgt\_test\_ratio : 最大垂直位置新息分量与新息测试限制之比
* tas\_test\_ratio : 最大真实风速新息分量与新息测试限制之比
* hagl\_test\_ratio : 最大地面上方高度新息分量与新息测试限制之比

对于每个传感器的二进制 pass/fail 汇总, 参考 innovation\_check\_flags ，位于 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg)。

### GPS 质量检测

开始GPS辅助之前，EKF进行了一系列的GPS质量检测。这些检测由参数 EKF2\_GPS\_CHECK 和 EKF2\_REQ&lt;&gt;控制。这些检测的 pass/fail 状态记录在 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).gps\_check\_fail\_flags 消息中。当所有要求的GPS检测通过，这个整数将为0。如果EKF没有开始GPS校准，查看 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) 中的 gps\_check\_fail\_flags 位掩码的定义。

### EKF 数值误差

为了降低对处理器的要求，对所有的运算EKF使用单精度浮点类型，对协方差预测和更新方程使用一阶近似。这意味着当重新调试EKF时有可能遇到异常情况，在其中协方差矩阵操作条件变得很恶劣以至于导致状态估计中产生发散或者明显的误差。

为了阻止这个，每一个协方差和状态更新步骤包含以下误差检测和修正步骤：

* 如果新息方差小于观测方差（这要求一个负的状态方差，这是不可能的）或者协方差更新将对任何状态产生负的方差：
  * 状态和协方差更新被跳过
  * 协方差矩阵中相应的行和列被重置
  * 失败被记录在 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) filter\_fault\_flags 消息中
* 状态方差 \(协方差矩阵中的对角线\) 被限制成非负值
* 上限被应用到状态方差中去
* 协方差矩阵强制对称

重新调试滤波器后，像降低噪声变量、estimator\_status.gps\_check\_fail\_flags 的数值这样的部分重新调试应该再次检测以确保仍然为零。

## 如果高度估计发散该怎么办？

EKF高度在飞行中远离GPS和高度计测量值，最常见原因是震动导致的IMU限幅（clipping）和/或混淆（aliasing）。如果种种情况出现，以下迹象在数据中应该很明显：

* [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).vel\_pos\_innov\[3\] and  [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).vel\_pos\_innov\[5\] 都将有同样的迹象。
* [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).hgt\_test\_ratio 将大于1.0

推荐第一步使用一个有效的隔离安装系统确保飞控与机架隔离。一个隔离底座具有6个自由度，因此有6个共振频率。作为通用规则，安装在隔离底座上的飞控的的6个共振频率应该大于25Hz以避免与飞控动力学的交叉，并且低于点击的频率。

如果共振频率与点击或螺旋桨的转动频率重合，隔离底座只会使得振动情况更加恶劣。

通过进行以下参数修改，EKF可以变得对于震动引起的高度发散更加具有抵抗力：

* 加倍主要高度传感器的新息阈值。如果使用的是气压计高度对应的就是EK2\_EKF2\_BARO\_GATE。
* 开始时提高EKF2\_ACC\_NOISE的数值到0.5。如果发散还是会出现，每次增加0.1，但是不要超过1.0。

注意这些改变会使得EKF对于GPS垂直速度和气压更加敏感。

## 如果位置估计发散该怎么办？

位置发散最常见的原因如下：

* 高振动水平。
  * 通过提升飞控的机械隔离水平。
  * 提高 EKF2\_ACC\_NOISE 和 EKF2\_GYR\_NOISE 的数值会有效，但是会使得EKF更易受小故障的干扰。
* 大的陀螺仪偏差的偏移。
  * 通过重新校准陀螺仪来修正。检查过量的温度灵敏性（在从冷启动逐渐加热的过程中存在&gt; 3 deg/sec的偏差变化），并且如果收到影响就替换传感器以降低随着温度变化的变化速率。
* 糟糕的偏航校准
  * 检查磁力计的校正和对齐。
  * 检查QGC中显示朝向与真实朝向偏差在15度以内。
* 质量很差的GPS精度
  * 检查干扰
  * 提升隔离和屏蔽
  * 检查飞行位置是否有GPS信号阻碍或者反射（接近高楼大厦）
* GPS信号丢失

确定哪一个是主要因需需要系统的方法来分析EKF日志数据:

* 画出速度新息测试比曲线 - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vel\_test\_ratio
* 画出水平位置新息比曲线 - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg)。pos\_test\_ratio
* 画出高度新息比曲线 - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).hgt\_test\_ratio
* 画出磁力计新息比曲线 - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).mag\_test\_ratio
* 画出GPS报告的速度精度曲线 - [vehicle\_gps\_position](https://github.com/PX4/Firmware/blob/master/msg/vehicle_gps_position.msg).s\_variance\_m\_s
* 画出IMU角度增量状态估计曲线 - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).states\[10\], states\[11\] and states\[12\]
* 画出EKF内置的高频振动度量：
  * 角度增量锥进振动 - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vibe\[0\]
  * 高频角度增量振动 - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vibe\[1\]
  * 高频速度增量振动 - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vibe\[2\]

常规操作期间，搜所有的测试比例应该保持在0.5以下，只是偶尔会出现尖峰，如下面一次顺利的飞行中的例子所示：
![Position, Velocity, Height and Magnetometer Test Ratios](Screen Shot 2016-12-02 at 9.20.50 pm.png)  
下图显示了一台减震良好的多轴飞行器的EKF振动度量。可以看到起飞和降落过程中的着陆冲击和升高的振动。要提供关于最大阈值的相关建议，这些数据还不足够。
![](Screen Shot 2016-12-02 at 10.24.00 pm.png)  
以上振动度量价值有限，因为振动出现在了IMU采样频率（对于大多数飞控板而言是1 kHz）附近，这将导致高频振动度量中不会出现的偏移出现在数据之中。唯一的探测混淆误差的方式是在观察到它们影响惯性导航系统精度和导致新息水平提升。

除了产生&gt; 1.0的位置和速度测试比，不同的误差机理也会以不同的方式影响其他测试比：

### 确定超量振动

高振动水平通常影响垂直位置和速度新息还有水平分量。磁力计测试水平只受到很小程度的影响。

\(此处需要插入显示恶劣振动的示意图/insert example plots showing bad vibration here\)

### 确定过量的陀螺仪偏差

大的陀螺仪偏差偏移量通常特征是飞行中角度增量偏差的数值大于5E-4（相当于~3 deg/sec），如果偏航轴受到影响也会导致磁力计测试比的大大提高。除非是极端情形，高度通常不会受到影响。如果滤波器在飞行前有时间收敛，把偏差值提高至5 deg/sec也是可以承受的。如果位置发散，commander进行的飞行前检查应当阻止解锁。

\(此处需要插入显示恶劣陀螺仪偏差的示意图/insert example plots showing bad gyro bias here\)

### 确定糟糕的偏航精度

当飞行器开始在惯导系统和GPS测量值计算出来的速度方向内不连续的移动时，恶劣的偏航校准导致速度测试比迅速增加。磁力计新息受到轻微影响。高度通常不受影响。

\(此处需要插入显示恶劣的偏航校准的示意图/insert example plots showing bad yaw alignment here\)

### 确定糟糕的GPS精度

糟糕的GPS精度通常伴随着GPS接收器的报告速度误差的上升，连同新息的上升。多路径、遮蔽、干扰导致的瞬态误差是更常见的原因。这里有一个GPS精度突然丢失的示例，此时多旋翼开始漂移远离悬停位置且必须使用摇杆进行修正。 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vel\_test\_ratio 上升到大于1暗示GPS速度与其他测量量不连续并已经被拒绝。

![](gps glitch - test ratios.png)

这里伴随着GPS接收器报告速度精度的增加，这暗示着可能是一个GPS错误。
![](gps glitch - reported receiver accuracy.png)

如果我们也看看GPS水平速度新息和新息方差，就能看到伴随着GPS '失灵（glitch）' 事件，大的尖峰出现在北向速度新息。
![](gps glitch - velocity innovations.png)

### 确定GPS数据丢失

GPS数据丢失将被速度和位置新息测试比'一蹶不振（flat-lining）'地显示出来。如果这个出现，检查 vehicle\_gps\_position 中的其他GPS状态数据找到更深入的信息。
\(此处插入显示GPS数据丢失的示例图/insert example plots showing loss of GPS data here\)

