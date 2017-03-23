# Windows 安装指南

> **警告** 虽然Windows Toolchain是可用的，但是它并没有被正式支持，我们也不鼓励使用它。 它在固件编译期间非常缓慢，并且不支持像Snapdragon Flight这样的新板。 它也不能运行许多开发人员用于计算机视觉和导航原型的标准机器人软件包。 在Windows上开发之前，请考虑使用[Ubuntu](http://ubuntu.com)安装双引导环境。

## 开发环境安装

在你的系统上下载并安装:

  * [Qt Creator IDE](http://www.qt.io/download-open-source/#section-6)
  * [PX4 Toolchain Installer v14 for Windows 下载](http://firmware.diydrones.com/Tools/PX4-tools/px4_toolchain_installer_v14_win.exe) (32/64位系统，完整的构建系统，驱动程序)
  * [PX4 USB 驱动](http://pixhawk.org/static/px4driver.msi) (32/64位系统)

现在继续运行 [首次构建](starting-building.md)！

## 新! Windows系统上的 Bash

Windows用户有了一个新的选项，在本机运行Bash shell，然后遵循Linux
构建说明。 请参阅[BashOnWindows](https://github.com/Microsoft/BashOnWindows)。 我们
验证了PX4能够在此环境中成功构建。 它还不能闪存固件，但是
您可以使用任务计划或QGroundControl在Windows上刷自定义的固件。