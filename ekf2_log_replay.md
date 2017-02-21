# EKF2 飞行记录回放 

本页面将展示如何通过飞行记录的回放功能来调试EKF2（扩展卡尔曼滤波）估计器，该功能是通过 `sdlog2` 记录器实现的。

## 介绍 
开发者可以启用对于EKF2估计器的输入传感器数据进行板载记录的功能。然后通过飞行记录回放对EKF2估计器的参数进行线下调试。本页余下的部分将讲解哪些参数的调试能得益于记录回放功能，并说明如何正确的进行配置。

## 预备
*  **EKF2\_REC\_RPL** 设置为1。这将通知EKF2显示针对日志回放的特定消息。
* **SDLOG\_PRIO\_BOOST** 设置为 {0, 1, 2, 3} 中的的一个数。0意味着板载飞行记录 app 获得默认（低）的调度优先级。低优先级会导致飞行记录信息的丢失。如果你发现你的记录文件出现 'gaps' （信息不连续），然后你可以将 **SDLOG\_PRIO\_BOOST** 提高到最大值3。测试显示不小于2才能避免数据丢失。

## 配置

当你有了一份飞行记录，你可以通过在 PX4 根目录下运行如下命令行来进行记录回放。

```
make posix_sitl_replay replay logfile=absolute_path_to_log_file/my_log_file.px4log
```

其中， 'absolute\_path\_to\_log\_file/my\_log\_file.px4log' 是记录文件所在根目录。该命令将检查记录文件的目录和名字。

## 估计器器参数获取与调试

你可以在文件 **replay\_params.txt** 中设置回放过程中的 EKF2 估计器参数， **replay\_params.txt** 和你的回放日志文件在同一目录下，比如： **build\_posix\_sitl\_replay/src/firmware/posix/rootfs/replay\_params.txt** 。首次运行回放功能时（比如：使用 **make clean** 命令后）， **replay\_params.txt** 将根据飞行记录自动生成并写入默认的 EKF2 估计器参数。之后，你可以通过改变该txt文件中的相关量来改变对应的 EKF2 参数。设置陀螺仪漂移的噪声值将需要以下命令行：

```
EKF2_GB_NOISE 0.001
```
