# 系统启动

PX4 的启动是由存在于[ROMFS/px4fmu_common/init.d](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d) 文件夹中的 shell 脚本控制

所有的以数字和下划线（例如 `10000_airplane`）开头的文件都是封装好的机身配置文件。在构建的时候，他们会被导出到一个叫 `airframes.xml` 的文件中，这个文件是被 [QGroundControl](http://qgroundcontrol.com) 解析并为机体选择UI的。 如果你想添加新的配置文件，请点击[这里](airframes-adding-a-new-frame.md)

剩下的文件是通用启动逻辑中的一部分，第一个执行的文件是 [rcS](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d/rcS) 脚本，它会调用所有的其他脚本。

## 调试模式启动

硬件或者软件的故障会导致引导终止。

<aside class="tip">
不完全的启动通常由于地面控制站缺少参数导致的，因为未启动的引用程序没有初始化它们的参数。
</aside>

调试引导顺序的正确方法是连接 [系统控制台](advanced-system-console.md) 然后重新对 PX4 上电。在生成的启动日志中记录了详细的启动顺序信息，并应该包含启动中断的原因。

### 常见的引导失败原因

  * 所需的传感器无法启动
  * 对于自定义的引用程序：常见的错误是由于内存溢出导致的。运行 `free` 命令来检查系统可用的 RAM 
  * 软件故障或断言导致的堆栈跟踪

## 更换系统启动

在大多数情况下，自定义默认引导是更好的方法。如果想要自定义默认引导，请在 `etc` 文件夹下创建创建一个 `rc.txt` 文件，这个文件可以在 microSD 卡上的文件夹找到。当这个文件不存在的时候，系统会默认自动启动。

## 自定义系统启动

自定义系统启动的最好办法是引入一个[新机身配置文件](airframes-adding-a-new-frame.md). 如果只是想调整配置文件（如启动更多的应用或者使用不同的混控）可以使用特殊的进程钩子。

<aside class="caution">
系统启动配置文件是 UNIX 文件，它要求 UNIX 的行结尾。如果使用 Windows 系统来编辑这个文件，请选择一个合适的编辑器。 
</aside>

一下是三个主要的进程钩子的说明。注意，microSD 卡的根目录文件夹路径为 `/fs/microsd`

  * /fs/microsd/etc/config.txt
  * /fs/microsd/etc/extras.txt
  * /fs/microsd/mixers/NAME_OF_MIXER

### 自定义配置（config.txt）

该 config.txt 文件在主系统配置完成后并在引导之前加载，并允许修改 shell 变量。

### 启动其他应用程序

extras.txt 可用于启动主系统引导后额外的应用程序。通常这些将是有效载荷控制器或类似的可选自定义组件

### 启动自定义混控

默认情况下，系统从  `/etc/mixers`  如果一个具有相同文件名的文件存在于  `/fs/microsd/etc/mixers`  那么这个文件将会被载入。通过这个设定，允许用户自定义混控且无需重新编译固件。