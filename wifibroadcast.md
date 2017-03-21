# 在 QGroundControl 中进行长距离视频传输
这一页告诉你如何在一个装有摄像头(罗技 C920)的便携式计算机(Odroid C1 或 C0)把视频传输到一台装有 QGroundControl 且能上网的电脑中。 此设置用的是WIFI的广播模式。

整个硬件由以下几个部分组成:

在发送端(飞行器):
* Odroid C1
* 罗技摄像头 C920
* WIFI 模块 ALPHA AWUS051NH v2。

接收端 (地面站):
* 任何Linux电脑.
* WIFI 模块 ALPHA AWUS051NH v2.


## 为什么常规 WIFI 在长距离视频传输中是一个不好的选择呢
 - 连接: 视频发射器和接收机需要建立连接。如果一个硬件失去连接之后(比如 信号太弱)视频传输就立刻停止了。
 - 无错误传输: WIFI 传输正确的数据或者不传输数据。这在 FPV 场景中就意味着即使你收到的数据中只有很小的错误也会丢弃这个数据包。导致即使收到了有用的数据也会失速。 
 - 双向通信: 即使你只是从源到目标发送数据，WIFI 也需要双向数据流。原因就在于 WIFI 接收机需要确认收到的数据包。 如果发射器没有收到确认，就会中断这个连接。因此，你需要在飞机和地面端同样强度的发射机和天线。在空中使用带全向天线的大功率发射机而地面上使用高增益天线的小功率接收机对于常规 WIFI 是不可能实现以上目标的。
 - 速率控制: 如果信号太弱，常规 wifi 会自动切换到较低的传输速率。由于这个原因，(自动)选择的速率太低，无法传输视频。这样会让后面的数据排队，并引入不可估计的延迟，甚至长达几秒钟。
 - 一对一传输:  除非你使用广播帧或类似技术，否则常规 WIF 的数据流是一对一的。因此常规 WIFI 很难实现 旁观者通过锁定“频道”就可以和模拟图传一样观看你的视频流。
 - 限制分流: 常规 WIFI 会限制你的 WIFI 网卡提供的视频分流数量。

## 广播模式有什么不一样
Wifi 广播会让WIF 网卡进入监视模式. 这种模式允许发射和接收任意数据包而不需要建立连接。此外还可以接收错误的帧(其中校验和不匹配)。这样，就建立了具有模拟链路特性的真正的单向连接。这些特性是:

 - 发射机发送其数据，而不管任何已连接的接收机。因此，没有因为连接断开而视频突然中断的风险。
 - 只要在发射器的范围内，接收机就能接收视频。如果它慢慢超出范围，视频质量降低，但不会停顿。即使帧错误，它们也将被显示，而不是被拒绝。
 - 传统方案“单个广播器 - 多个接收器”又可以使用了。旁观者只需要“切换到正确的频道”就可以用他们的设备观看视频流了。
 - Wifi 广播允许你并行使用多个廉价的接收机，同时融合这些数据来提高收到正确数据的概率。这种可媲美互补接收机的软件分集允许你使用相同的接收机来提高可靠性(想象一下带有一个全向天线和多个用于高度的定向天线同的接收机，并且这些天线同时工作)。 
 - Wifi 广播在低带宽下使用了前向纠错码来实现高可靠性。在接收端具有恢复丢失或损坏数据包的能力。


## 硬件修改.
Alpha WUS051NH 是一个大功率模块，在发送的时候需要很大的电流。如果是从 USB 端口供电，那么会导致 Odroid C1/C0 复位。
所以你需要直接连接一个 5V BEC 电源，你有两个方法:

 1. 制作一条自定义 USB 线。
 2. 在 Odroid PCB 板上，把 USB 端口附近的5V电源割断，然后连接到BEC电源上。
    另外，我建议在电源和地之间添加470uF低 ESR 电容（如 ESC电容），以滤除电压尖峰。

## 软件设置
下载 wifibroadcast [源码](https://github.com/svpcom/wifibroadcast).

你需要给内核打一个补丁:
 
 1. 使能发送速率锁。 使用 ``mac80211-radiotap-bitrate_mcs_rtscts.linux-4.4.patch``。 解决无法为注入radiotap数据帧指定特定数据率。
 2. 使能发送功率锁。 使用 ``ez-wifibroadcast-1.4-kernel-4.4-patches.diff``. 这可以锁定网卡所能支持的最大发送功率。
 3. 启用错误帧（校验和）接收的。 使用 ``ez-wifibroadcast-1.4-kernel-4.4-patches.diff``。 这是可选项，在当前代码中不使用。

所以你只能在发送端给内核打补丁。

### 在发送端你需要:

1. 设置相机输出RTP流:
```
gst-launch-1.0 uvch264src device=/dev/video0 initial-bitrate=6000000 average-bitrate=6000000 iframe-period=1000 name=src auto-start=true \
               src.vidsrc ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! rtph264pay ! udpsink host=localhost port=5600
```
 2. 在发送模式中设置 wifibroadcast:

```
git clone https://github.com/svpcom/wifibroadcast
cd wifibroadcast
make
ifconfig wlan1 down    # 假设发送网卡是wlan1
iw reg set BO          # 设置 CRDA 区域为能允许的最大发送功率
iw dev wlan1 set monitor otherbss fcsfail
ifconfig wlan1 up
iwconfig wlan1 channel 149
./tx -r 24 wlan1
```
这将设置wifibroadcast在第149信道上侦听UDP端口5600，并且使用24MB/s的数据率进行通信。

### 在接收端你需要:

 1. 在接收模式中设置 wifibroadcast:
```
git clone https://github.com/svpcom/wifibroadcast
cd wifibroadcast
make
ifconfig wlan1 down    # 假设接收网卡是wlan1
iw reg set BO          # 设置与发送相同的区域，以确保您可以监听发送通道
iw dev wlan1 set monitor otherbss fcsfail
ifconfig wlan1 up
iwconfig wlan1 channel 149
./rx wlan1
```
 2. 运行qgroundcontrol 或者
```
gst-launch-1.0 udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
             ! rtph264depay ! avdec_h264 ! clockoverlay valignment=bottom ! autovideosink fps-update-interval=1000 sync=false
```
来进行解码。

## 常见问题解答
问: 与原始的 wifibroadcast 有什么区别?

答: 原始版本的 wifibroadcast 使用的是字节流作为输入，并且将其分成固定大小的数据包(默认为1024)。如果无线
数据包丢失，并且没有被 FEC 纠正的话，就会在数据流的随机位置(非预期的)产生一个“洞”。如果数据通信协议不能
抵抗这种随机擦写，这非常糟糕。所以我重新组建一个无线 UDP 数据包。
无线数据包大小取决于有效数据大小，这大大减小了传输延迟。

问: 什么样的数据可以通过 wifibroadcast 传输?

答: 任何小于等于1466字节的UDP数据包。 例如，由 x264 组成的 RTP 数据流或 Mavlink数据包。

问: 什么是传输保证?

答: Wifibrodcast 使用 FEC (前身误差校正) 的默认设置可以从12个数据包当中恢复4个数据包。
   你可以调整参数 (发送端和接收端要同时修改!) 来适应你的需求.

问: 我经常丢帧，并且出现 ``XX packets lost``的消息。 这是怎么回事?

A: 原因可能是:
   1. 信号功率太小。使用更高功率的网卡或者更高增益的天线；接收端使用定向天线；使用额外的无线网卡(接收端添加 wlan2, wlan3, ...)。
   2. 信号功率太大。特别是在室内使用了 30dBm，尝试着降低发送功率(破解内核的 CRDA 数据库， 
       并且将几个区域功率限制在10dBm 和 20dBm)。
   3. 被其它 WIFI 干扰了. 尝试更改一个 WIFI 信道或增加 WIFI 带宽。注：不要使用无线接收机的频段！或者正确设置 RTL 以免你的飞行器丢失。
      你可以增加 FEC 块的大小(默认是 8/12，8个数据块， 4个 FEC 块)，缺点是增加了延迟。 使用额外的接收模块(接收程序增加 wlan2, wlan3, ...)

## 需要做的
1. 使用不同的无线网卡/天线进行飞行测试。
2. 研究如何不通过破解 CRDA 系统设置发送功率。
3. 调节 FEC 来获得最佳延迟/冗余。

## 无线网卡:

以下创锐讯芯片组可以使用:

 -  Atheros AR9271, Atheros AR9280, Atheros AR9287

以下雷凌芯片组可以使用

 -  RT2070, RT2770, RT2870, RT3070, RT3071, RT3072, RT3370, RT3572, RT5370, RT5372, RT5572

 然而，这些网卡工作前都可能遇到任何小问题，所以你想要安全的使用它，请选择一个经过不同的人测试过的，一定可以工作的网卡：

 -  CSL 300Mbit Stick (2.4/5Ghz, 分集天线, RT5572 芯片组)
 -  Alfa AWUS036NHA (2.3/2.4Ghz, 大功率, 创锐讯 AR9271 芯片组)
 -  TPLink TL-WN722N (2.3/2.4Ghz, 创锐讯 AR9271 chipset)
 -  ALFA AWUS051NH v2 (2.4Ghz/5Ghz, 大功率, 雷凌 RT3572 芯片组)
 -  ALFA AWUS052NH v2 (2.4Ghz/5Ghz, 分集天线, 大功率, Ralink 芯片组)
 -  TP-Link-TL-WDN3200 (2.4/5Ghz, 分集天线, RT5572 芯片组)
 -  雷凌 RT5572 (2.4/5Ghz, 分集天线, RT5572 芯片组)

另一方面，如果所有人都使用相同的网卡，那么我们永远不会知道其它网卡也能用。中国国内的商店还有很多小而轻便，在4美元以下的 RT5370 无线网卡。 例如 Aliexpress 就有很多便宜的无线网卡。 当你尝试了一个以上没有列出来的无线网卡，如果你向我们反馈了你的发现，这对我们很有用。

* AWUS036NHA 此适配器将提供约 280mW 的功率，大约几公里(但是是使用定向天线).

* TL-WN722N 这个适配器提供大约 60mW 的功率，使用 2.1dbi 的天线，传输范围大概在800-1000米。 重要信息：在某些情况下，PCB 上的第二个天线会导致接收不良。 请通过移除 PCB 背面的白色元件来断开天线。如下图所示(在图中，元件被焊接到焊盘上， 以便需要时可以反接)![图片](https://github.com/DimianZhan/PX4Devguide/blob/master/images/videostreaming/power-pins.png)

* CSL 300Mbit 无线网卡，这个适配器提供大约 30mW 的输出功率，范围在5Ghz不是很高，大约200-300m。 库存天线在5Ghz不可用，因为它们是简单的2.4Ghz 2.1dbi套筒偶极天线。

当用作是接收加密狗时，当接收信号强度高于 -20dbm 时，可能会发生坏块。这可以通过在不同方向/极化上使用多于一个适配器和指向天线来解决。

* AWUS051NH 这个适配器提供了大约 300mW 的输出功率。在5GHz频段下大约800-1000米 。 不推荐使用胶棒天线， 因为它们具有 5dbi 的增益，输出太平坦的辐射图案。

* AWUS052NH 这个适配器提供了大概3330mW 的输出功率。除了有两个发送链路不同之后，其它和 051NH 一样，不推荐使用胶棒天线，原因同上。

## 链接:
 - wifibroadcast 的[原始版本](https://befinitiv.wordpress.com/wifibroadcast-analog-like-transmission-of-live-video-data/) 
