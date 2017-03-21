# 起飞前传感器和EKF的检查
控制器通过设置COM_ARM<>相关参数，来执行起飞时传感器和EKF的一些列检查。如果这些检查失败，电机将不能解锁，并且会产生以下错误信息：

* 起飞失败： EKF HGT ERROR
 * 此错误产生的原因是IMU和高度测量的数据不一致。
 * 执行加速度计和陀螺仪的校准，并且重启设备。如果错误仍然存在，请检查高度传感器数据是否有问题。
 * 此项检查是通过COM_ARM_EKF_HGT 来设置的。
* 起飞失败：EKF VEL ERROR
 * 此错误产生的原因是IMU和GPS的速度测量值不一致。
 * 请检查GPS速度数据是否有不合实际的跳点。如果GPS质量没有问题，请校准加速度计和陀螺仪，并且重启设备。
 * 此项检查是通过COM_ARM_EKF_VEL参数来设置的。
* 起飞失败： EKF HORIZ POS ERROR
 * 此错误产生原因是IMU和位置测量值不一致（比如GPS或者外部视觉传感器）。 
 * 请检查位置传感器数据是否有不切实际的跳点，如果数据值没有问题，请校准加速度计和陀螺仪，并且重启设备。
 * 此项检查是通过COM_ARM_EKF_POS参数来设置的。
* 起飞失败： EKF YAW ERROR
 * 此错误产生原因是采用陀螺估计的航向角值和用磁罗盘或者外部视觉系统估计的航向角值不一致。
 * 请检查IMU数据是否有大的偏航率补偿值，并且对磁罗盘进行对准和标定。
 * 此项检查是通过COM_ARM_EKF_YAW参数来设置的。
* 起飞失败： EKF HIGH IMU ACCEL BIAS
 * 此错误产生原因是通过EKF估计的IMU 加速度计零偏值超了预设值。
 * 此项检查是通过COM_ARM_EKF_AB参数来设置的。
* 起飞失败： EKF HIGH IMU GYRO BIAS
 * 此错误产生原因是通过EKF估计的IMU 陀螺仪零偏值超了预设值。 
 * 此项检查是通过COM_ARM_EKF_GB参数来设置的。
* 起飞失败： ACCEL SENSORS INCONSISTENT - CHECK CALIBRATION
 * 此错误产生原因是不同IMU单元的加速度计测量值不一致。
 * 此项检查仅适用于板上有多个IMU。
 * 此项检查是通过COM_ARM_EKF_ACC参数来设置的。
* 起飞失败：GYRO SENSORS INCONSISTENT - CHECK CALIBRATION
 * 此错误产生原因是不同IMU单元的角速度测量值不一致。
 * 此项检查仅适用于板上有多个IMU。
 * 此项检查是通过COM_ARM_EKF_GYR参数来设置的。

##COM_ARM_WO_GPS
COM_ARM_WO_GPS参数值，用来控制在没有GPS信号下机体是否允许解锁。若当前没有GPS信号，该参数设置为0时机体才能允许解锁。没有GPS就解锁状况，仅在选择不需要GPS的飞行模式下才被允许。
##COM_ARM_EKF_POS
COM_ARM_EKF_POS参数值，为EKF惯性测量数据和位置参考数据（GPS或者外部视觉系统）的允许不一致性的最大值。默认值为0.5，表示允许两者不一致性不超过EKF最大能容忍值的50%，并且提供了一些飞行开始时误差增加的余量。
##COM_ARM_EKF_VEL
COM_ARM_EKF_VEL参数值，为EKF惯性测量数据和GPS 速度测量的允许不一致性的最大值。默认值为0.5，表示允许两者不一致性不超过EKF最大能容忍值的50%，并且提供了一些飞行开始时误差增加的余量。
##COM_ARM_EKF_HGT
COM_ARM_EKF_HGT参数值，为EKF惯性测量数据和GPS 高度测量（气压计、GPS、测距仪或者外部视觉系统）的允许不一致性的最大值。默认值为0.5，表示允许两者不一致性不超过EKF最大能容忍值的50%，并且提供了一些飞行开始时误差增加的余量。
##COM_ARM_EKF_YAW
COM_ARM_EKF_YAW参数值，为EKF惯性测量数据和航向测量（磁罗盘或者外部视觉系统）的允许不一致性的最大值。默认值为0.5，表示允许两者不一致性不超过EKF最大能容忍值的50%，并且提供了一些飞行开始时误差增加的余量。
##COM_ARM_EKF_AB
COM_ARM_EKF_AB参数值，为EKF估计IMU加速度计零偏的最大值。默认值为0.005，表示允许加速度计零偏的上限为0.5m/s/s.
##COM_ARM_EKF_GB
COM_ARM_EKF_GB参数值，为EKF估计IMU陀螺仪零偏的最大值。默认值为0.00087，表示允许陀螺仪零偏的上限为5deg/s.
##COM_ARM_IMU_ACC
COM_ARM_IMU_ACC参数值，为飞控默认使用的IMU加速度计测量值，和安装的其他IMU的加速度计测量值的允许不一致性的最大值。 
##COM_ARM_IMU_GYR
COM_ARM_IMU_GYR参数值，为飞控默认使用的IMU陀螺仪测量值，和安装的其他IMU的陀螺仪测量值的允许不一致性的最大值。







