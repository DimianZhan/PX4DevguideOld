# 在 Snapdragon 飞控板上使用光流

 Snapdragon 飞控板有一个垂直向下的灰度相机，该相机可以被用来进行光流位置增稳。

除了相机外，光流模块还需要包括一个垂直向下的距离传感器。因此，在这里将讨论 TeraRanger One 模块的使用。


## TeraRanger One 设置
 TeraRanger One (TROne) 使用 I2C 接口同 Snapdragon 飞控板进行连接。TROne 必须通过出厂 I2C 固件进行刷写。

 TROne 通过一个定制的 DF13 4针转6针接口和 Snapdragon 飞控板相连。

| 4 pin | <-> | 6 pin |
| -- | -- | -- |
| 1 |  | 1 |
| 2 |  | 6 |
| 3 |  | 4 |
| 4 |  | 5 |

TROne 必须通过 10 - 20V 供电。

## 光流
光流程序运行在应用处理器上然后通过 Mavlink 传到 PX4 中。
下载 [snap_cam](https://github.com/PX4/snap_cam) ，并根据其中的 readme 文件进行编译。

在根目录下运行光流程序：
````
optical_flow -n 50 -f 30
```

光流程序需要从 PX4 传来的惯性单元（IMU）的数据。因此，可能需要在 PX4 中添加一个额外的 Mavlink 实例，该操作通过在 `mainapp.config` 中添加如下代码实现：

```
mavlink start -u 14557 -r 1000000 -t 127.0.0.1 -o 14558
mavlink stream -u 14557 -s HIGHRES_IMU -r 250
```
