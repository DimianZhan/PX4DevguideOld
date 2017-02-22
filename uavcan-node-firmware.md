# UAVCAN 固件升级

## 矢量控制型电调的代码库 （对应设备型号 Pixhawk 电调1.6版 和 S2740VC 电调）

下载电调源码：

<div class="host-code"></div>

```sh
git clone https://github.com/thiemar/vectorcontrol
cd vectorcontrol
```

### 烧写 UAVCAN 的引导程序（bootLoader）

在通过 UAVCAN 总线更新固件之前，首先需要对Pixhawk 电调1.6版进行 UAVCAN 引导程序的烧写。引导程序可按照以下步骤进行编译：

<div class="host-code"></div>

```sh
make clean && BOARD=px4esc_1_6 make -j8
```

编译完成后，程序镜像生成在路径 `firmware/px4esc_1_6-bootloader.bin` 下，OpenOCD（一种开源的JTAG上位机程序）的配置文件在路径 `openocd_px4esc_1_6.cfg` 下。按照 [操作说明](uavcan-bootloader-installation.md) 将引导程序安装在电调之中。

### 编译主二进制文件

<div class="host-code"></div>

```sh
BOARD=s2740vc_1_0 make && BOARD=px4esc_1_6 make
```

会编译出上述两种型号电调的 UAVCAN 节点固件。镜像分别保存在 `com.thiemar.s2740vc-v1-1.0-1.0.<git hash>.bin` 和 `org.pixhawk.px4esc-v1-1.6-1.0.<git hash>.binn` 中。

## Sapog 平台电调代码库 （对应型号Pixhawk 电调1.4版本）

下载 Sapog 代码库：

<div class="host-code"></div>

```sh
git clone https://github.com/PX4/sapog
cd sapog
git submodule update --init --recursive
```

### 烧写 UAVCAN 的引导程序

在通过 UAVCAN 总线更新固件之前，首先需要对Pixhawk 电调1.4版进行 UAVCAN 引导程序的烧写。引导程序可按照以下步骤进行编译：

<div class="host-code"></div>

```sh
cd bootloader
make clean && make -j8
cd ..
```

引导程序的镜像存放于 `bootloader/firmware/bootloader.bin`，OpenOCD 的配置文件存放于 `openocd.cfg`。 按照 [操作说明](uavcan-bootloader-installation.md) 将引导程序安装在电调中。

### 编译主二进制文件

<div class="host-code"></div>

```sh
cd firmware
make sapog.image
```
固件镜像位于 `firmware/build/org.pixhawk.sapog-v1-1.0.<xxxxxxxx>.bin`，其中的 `<xxxxxxxx>` 是一串任意的数字字母组合。

## Zubax 卫星信号接收机

请参考 [项目首页](https://github.com/Zubax/zubax_gnss) 学习如何进行编译和固件的烧写。
Zubax 卫星接收机的引导程序兼容 UAVCAN 总线，因此其固件的更新方法与下面描述的其他设备的固件更新方法是一致的。

## 自动驾驶仪的固件安装

所有 UAVCAN 的节点文件命名遵循统一的规范，所以即使在同一 UAVCAN 总线中的设备可能并不来自于同一个厂商，Pixhawk 也可以对它们进行固件的更新。因此，为了能够顺利完成更新，上述编译生成的所有固件文件必须要拷贝到 SD 卡的正确的路径下或者 PX4 的 ROMFS 中。

固件镜像文件的命名规范为：

  ```<uavcan 名称>-<硬件主版本>.<硬件副版本>-<软件主版本>.<软件副版本>.<版本 hash>.bin```

 例如 ```com.thiemar.s2740vc-v1-1.0-1.0.68e34de6.bin```

但是，基于存储空间和处理能力的限制（名字不能超过28个字母），UAVCAN 固件的更新程序要求将文件名分开并保存在如下格式的路径中：

  ```/fs/microsd/fw/<节点名>/<硬件主版本>.<硬件副版本>/<硬件名>-<软件主版本>.<软件副版本>.<git hash>.bin```

例如 ```s2740vc-v1-1.0.68e34de6.bin```

基于 ROMFS 的更新程序也遵循上述规范，只是会在文件名首字母前增加下划线 ```_```，所以要将固件添加为：

  ```/etc/uavcan/fw/<设备名称>/<硬件主版本>.<硬件副版本>/_<硬件名>-<软件主版本>.<软件副版本>.<git hash>.bin```

## 将二进制文件放在 PX4 的 ROMFS 文件夹中

最终的文件位置如下：

  * S2740VC 电调: `ROMFS/px4fmu_common/uavcan/fw/com.thiemar.s2740vc-v1/1.0/_s2740vc-v1-1.0.<git hash>.bin`
  * Pixhawk 电调 1.6版: `ROMFS/px4fmu_common/uavcan/fw/org.pixhawk.px4esc-v1/1.6/_px4esc-v1-1.6.<git hash>.bin`
  * Pixhawk 电调 1.4版: `ROMFS/px4fmu_common/uavcan/fw/org.pixhawk.sapog-v1/1.4/_sapog-v1-1.4.<git hash>.bin``
  * Zubax 卫星信号接收机 v1: `ROMFS/px4fmu_common/uavcan/fw/com.zubax.gnss/1.0/gnss-1.0.<git has>.bin`
  * Zubax 卫星信号接收机 v2: `ROMFS/px4fmu_common/uavcan/fw/com.zubax.gnss/2.0/gnss-2.0.<git has>.bin`

注意文件夹 ROMFS/px4fmu_common 在 Pixhawk 中将会位于 /etc 中。

### 启动固件升级进程

<aside class="note">
当使用 [PX4 飞行套件](concept-flight-stack.md)时，在 '电源设置' 板块使能 UAVCAN，并且在 UAVCAN 固件试图升级前重启系统。
</aside>

还可以选择通过在 NSH 中输入命令手动启动 UAVCAN 的固件升级：

```sh
uavcan start
uavcan start fw
```
