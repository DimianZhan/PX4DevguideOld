# 驱动开发

PX4的代码使用轻量级的, 统一的驱动抽象层: [DriverFramework](https://github.com/px4/DriverFramework). 对于 POSIX 和 [QuRT](https://en.wikipedia.org/wiki/Qualcomm_Hexagon) 的相关驱动也会写入这个框架中.

<aside class="todo">
而 NuttX 原始驱动是基于 [设备](https://github.com/PX4/Firmware/tree/master/src/drivers/device) 框架 ，以后也会移植到驱动框架.
</aside>

## 核心框架

PX4是一个[反应式系统](concept-architecture.md) (reactive system)，采用发布/订阅方式（pub/sub）传递消息. 文件句柄是不被操作系统的核心所需要或者使用. 主要使用以下两个API:

* 发布/订阅系统拥有的文件，网络或者共享内存接口并依赖于PX4系统运行
* 一个允许枚举设备和读取/修改设备配置的全局设备注册表. 可以更简单链接或者映射文件系统

## 移植新平台

### NuttX

* 启动脚本位于 [ROMFS/px4fmu_common](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common)
* 系统配置文件位于 [nuttx-configs](https://github.com/PX4/Firmware/tree/master/nuttx-configs).系统加载后作为系统一部分编译.
    * PX4 中间件配置文件位于 [src/drivers/boards](https://github.com/PX4/Firmware/tree/master/src/drivers/boards). 它包含总线，GPIO映射和硬件平台初始化代码
    * 驱动文件位于 [src/drivers](https://github.com/PX4/Firmware/tree/master/src/drivers)
    * 参考配置 : 运行 'make px4fmu-v4_default' 命令生成针对当前 NuttX 下的 FMUv4 参考配置文件

### QuRT / Hexagon

* 启动脚本位于 [posix-configs/](https://github.com/PX4/Firmware/tree/master/posix-configs)
* 系统配置作为默认linux镜像的一部分 (待办事项: 提供LINUX IMAGE的位置 和 flash 指令)
    * PX4 中间件配置文件位于 [src/drivers/boards](https://github.com/PX4/Firmware/tree/master/src/drivers/boards). 待办事项: 增加总线配置文件
    * 驱动文件位于 [DriverFramework](https://github.com/px4/DriverFramework)
    * 参考配置: 运行 'make qurt_eagle_release' 命令生成 骁龙 Flight 参考配置文件

## 设备ID

PX4在整个系统中采用设备ID方式识别传感器. 这些设备ID被存储在配置参数表中，并被用于适配传感器校准值, 以及确定哪些传感器被记录到对应的日志文件中

传感器的顺序 (例如 `/dev/mag0` 和 备份 `/dev/mag1`)并不能确定优先级 -优先级是在发布 uORB topic 时确定

### 代码例程

用三个磁力计举例, 使用飞行日志 (.px4log)  查看参数.  三个参数表定义传感器ID 并且使用 `MAG_PRIME`定义哪个一个被选作为首要磁力计. 每一个磁力计ID MAGx_ID 有24位 ，左边不够位手动填0补全


```
CAL_MAG0_ID = 73225.0
CAL_MAG1_ID = 66826.0
CAL_MAG2_ID = 263178.0
CAL_MAG_PRIME = 73225.0
```

一个外置HMC5983采用I2C连接，总线1，地址位 `0x1E`:在日志文件显示为 `IMU.MagX`.

```
# 设备ID 73225 显示为24位2进制:
00000001  00011110  00001 001

# 编码为:
HMC5883   0x1E    bus 1 I2C
```

一个内置HMC5983采用SPI连接，总线1，片选 5。在日志文件显示为 `IMU1.MagX`.

```
# 设备ID 66826 显示为24位2进制:
00000001  00000101  00001 010

# 编码为:
HMC5883   dev 5   bus 1 SPI
```

以及一个内置MUP9250采用SPi连接，总线1，片选4。在日志文件显示为 `IMU2.MagX`.

```
# 设备ID 263178 显示为24位2进制:
00000100  00000100  00001 010

# 编码为:
MPU9250   dev 4   bus 1 SPI
```

### 设备ID编码

设备ID是按照这个格式的24位数。注意上述编码示例中，第一个字段是最低有效位

```C
struct DeviceStructure {
  enum DeviceBusType bus_type : 3;
  uint8_t bus: 5;    // 总线类型实例
  uint8_t address;   // 总线地址(例如. I2C地址)
  uint8_t devtype;   // 设备类设备类型
};
```
 `bus_type`包含以下几种类型 :

```C
enum DeviceBusType {
  DeviceBusType_UNKNOWN = 0,
  DeviceBusType_I2C     = 1,
  DeviceBusType_SPI     = 2,
  DeviceBusType_UAVCAN  = 3,
};
```

 `devtype`有以下定义 :

```C
#define DRV_MAG_DEVTYPE_HMC5883  0x01
#define DRV_MAG_DEVTYPE_LSM303D  0x02
#define DRV_MAG_DEVTYPE_ACCELSIM 0x03
#define DRV_MAG_DEVTYPE_MPU9250  0x04
#define DRV_ACC_DEVTYPE_LSM303D  0x11
#define DRV_ACC_DEVTYPE_BMA180   0x12
#define DRV_ACC_DEVTYPE_MPU6000  0x13
#define DRV_ACC_DEVTYPE_ACCELSIM 0x14
#define DRV_ACC_DEVTYPE_GYROSIM  0x15
#define DRV_ACC_DEVTYPE_MPU9250  0x16
#define DRV_GYR_DEVTYPE_MPU6000  0x21
#define DRV_GYR_DEVTYPE_L3GD20   0x22
#define DRV_GYR_DEVTYPE_GYROSIM  0x23
#define DRV_GYR_DEVTYPE_MPU9250  0x24
#define DRV_RNG_DEVTYPE_MB12XX   0x31
#define DRV_RNG_DEVTYPE_LL40LS   0x32
```
