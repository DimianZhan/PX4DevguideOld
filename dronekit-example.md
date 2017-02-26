# 使用 DroneKit 与 PX4 建立通信

[DroneKit](http://dronekit.io) 帮助你在无人机上创建强大的应用。这些应用运行在无人机的机载计算机上，并且通过执行计算密集型和要求低延迟链路的任务（例如计算机视觉）来增强自驾仪。

DroneKit 和 PX4 目前正在努力获取全兼容性。自 DroneKit-python 2.2.0 起，已经有对任务处理和无人机监控的基本支持。

## 根据 PX4 配置 DroneKit 

通过从最新的master安装 DroneKit-python 开始。

```sh
git clone https://github.com/dronekit/dronekit-python.git
cd ./dronekit-python
sudo python setup.py build
sudo python setup.py install
```

创建一个新的 python 文件并且导入 DroneKit，pymavlink 和其他基本的模块。

```C
# 导入 DroneKit-Python
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math

```

连接到你的无人机支持 Mavlink 的端口或模拟器

```C
# 连接到无人机
print "Connecting"
connection_string = '127.0.0.1:14540'
vehicle = connect(connection_string, wait_ready=True)

```

显示一些基本的状态信息

```C
# 显示无人机的基本情况
print " Type: %s" % vehicle._vehicle_type
print " Armed: %s" % vehicle.armed
print " System status: %s" % vehicle.system_status.state
print " GPS: %s" % vehicle.gps_0
print " Alt: %s" % vehicle.location.global_relative_frame.alt
```


## 完整的任务示例

接下来的 python 脚本展示一个利用 Dronekit 和 PX4 完整的任务示例。由于模式切换并没有得到 Dronekit 的完整支持，于是我们发送我们自定义的模式切换命令。

```C
################################################################################################
# @File DroneKitPX4.py
# Example usage of DroneKit with PX4
#
# @author Sander Smeets <sander@droneslab.com>
#
# Code partly based on DroneKit (c) Copyright 2015-2016, 3D Robotics.
################################################################################################

# 导入 DroneKit-Python
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math


################################################################################################
# 设置
################################################################################################

connection_string       = '127.0.0.1:14540'
MAV_MODE_AUTO   = 4
# https://github.com/PX4/Firmware/blob/master/Tools/mavlink_px4.py


# 解析连接参数
parser = argparse.ArgumentParser()
parser.add_argument("-c", "--connect", help="connection string")
args = parser.parse_args()

if args.connect:
    connection_string = args.connect


################################################################################################
# 初始化
################################################################################################

# 连接到无人机
print "Connecting"
vehicle = connect(connection_string, wait_ready=True)

def PX4setMode(mavMode):
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                               mavMode,
                                               0, 0, 0, 0, 0, 0)



def get_location_offset_meters(original_location, dNorth, dEast, alt):
    """
    返回一个包含距离指定的 `original_location` 位置 `dNorth` 米和 `dEast` 米的经纬度的 LocationGlobal 对象。返回的位置拥有与 `original_location` 相同的 'alt' 值。
    当你想要移动无人机到相对于当前无人机位置的某个指定位置附近时，这个函数是有帮助的。
    除了接近极点的时候，该算法在小距离范围内是相对准确的（1km内10m误差）。
    获取更多信息请访问：
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #“球形”地球的半径
    #以弧度为单位的坐标偏移量
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #以十进制度数为单位的新的位置
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt+alt)





################################################################################################
# 监听器
################################################################################################

home_position_set = False

#创建一个针对 home position 定位的消息监听器
@vehicle.on_message('HOME_POSITION')
def listener(self, name, home_position):
    global home_position_set
    home_position_set = True



################################################################################################
# 开始任务示例
################################################################################################

# 等待获取 home position lock
while not home_position_set:
    print "Waiting for home position..."
    time.sleep(1)

# 显示基本的无人机情况
print " Type: %s" % vehicle._vehicle_type
print " Armed: %s" % vehicle.armed
print " System status: %s" % vehicle.system_status.state
print " GPS: %s" % vehicle.gps_0
print " Alt: %s" % vehicle.location.global_relative_frame.alt

# 切换到 AUTO 模式
PX4setMode(MAV_MODE_AUTO)
time.sleep(1)

# 加载命令
cmds = vehicle.commands
cmds.clear()

home = vehicle.location.global_frame

# 起飞到10m的高度
wp = get_location_offset_meters(home, 0, 0, 10);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)

# 向北移动10m
wp = get_location_offset_meters(wp, 10, 0, 0);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)

# 向东移动10m
wp = get_location_offset_meters(wp, 0, 10, 0);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)

# 向南移动10m
wp = get_location_offset_meters(wp, -10, 0, 0);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)

# 向西移动10m
wp = get_location_offset_meters(wp, 0, -10, 0);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)

# 降落
wp = get_location_offset_meters(home, 0, 0, 10);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)

# 上传任务
cmds.upload()
time.sleep(2)

# 解锁无人机
vehicle.armed = True

# 监控任务的执行
nextwaypoint = vehicle.commands.next
while nextwaypoint < len(vehicle.commands):
    if vehicle.commands.next > nextwaypoint:
        display_seq = vehicle.commands.next+1
        print "Moving to waypoint %s" % display_seq
        nextwaypoint = vehicle.commands.next
    time.sleep(1)

# 等待无人机降落
while vehicle.commands.next > 0:
    time.sleep(1)


# 无人机上锁
vehicle.armed = False
time.sleep(1)

# 在退出脚本之前关闭无人机对象
vehicle.close()
time.sleep(1)

```
