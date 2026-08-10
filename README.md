# Dashgo ROS Noetic 工作空间

本工作空间是 Dashgo 移动底盘的最终精简运行版本，提供 STM32 底盘驱动、
三台 YDLidar、里程计与 IMU 融合、GMapping 建图、AMCL/TEB 导航、RViz、
键盘控制和两路激光安全门。

## 最终运行环境

| 项目 | 要求 |
|---|---|
| 计算平台 | NVIDIA Jetson Nano |
| 操作系统 | Ubuntu 20.04，ARM64/aarch64 |
| ROS | ROS 1 Noetic |
| 工作空间 | `~/dashgo_ws`，catkin |
| Python | Python 3 |
| 底盘控制器 | STM32，115200 bit/s |
| 激光雷达 | 3 台 YDLidar，均为 230400 bit/s |

当前源码也在 Ubuntu 20.04 / ROS Noetic 的 WSL 环境完成了构建和无硬件启动检查，
但最终部署与真机验收平台是 Jetson Nano。WSL 检查不能代替 Nano 上的串口、雷达、
建图和导航验证。

## 工作空间结构

```text
src/
├── dashgo_driver/          STM32、编码器、odom、IMU、急停和速度接口
├── ydlidar/                三台 YDLidar 的驱动核心与启动文件
├── dashgo_description/     URDF/Xacro 与机器人 TF
├── yocs_velocity_smoother/ /cmd_vel 速度和加速度平滑
└── dashgo_bringup/         基础、建图、导航、RViz、安全和 teleop
```

## 硬件与串口连接

### 代码中的固定逻辑映射

| 逻辑设备 | 设备角色 | 波特率 | ROS 输出 | 配置位置 |
|---|---|---:|---|---|
| `/dev/port1` | STM32 底盘控制器 | 115200 | `/odom`、`/imu`及底盘状态 | `dashgo_driver/config/base.yaml` |
| `/dev/port2` | `ydlidar1_up` | 230400 | `/scan`，`laser_frame` | `ydlidar/launch/ydlidar1_up.launch` |
| `/dev/port3` | `ydlidar2_backup` | 230400 | `/scan_3`，`laser_frame_3` | `ydlidar/launch/ydlidar2_backup.launch` |
| `/dev/port4` | `ydlidar3_down` | 230400 | `/scan_2`，`laser_frame_2` | `ydlidar/launch/ydlidar3_down.launch` |

`/dev/port1`～`/dev/port4`不是 Ubuntu 自动生成的标准设备名，而是部署时必须建立的
持久化别名。不要直接依赖`/dev/ttyUSB0`等编号；设备重新插拔或 Nano 重启后，这些
编号可能变化。

仓库没有保存实际设备的 USB VID、PID、序列号或 Nano USB 插座编号，因此物理设备
应在 Nano 上逐台接入并确认，不能仅按`ttyUSB`出现顺序判断。

### 推荐连接方式

- STM32 通过 USB 转串口连接 Nano，并固定命名为`/dev/port1`；
- 三台 YDLidar 的 USB 串口分别固定为`/dev/port2`、`/dev/port3`和`/dev/port4`；
- Nano、STM32和雷达必须可靠共地；
- 三台雷达加 USB 转串口可能超过 Nano 单个 USB 口的稳定供电能力，建议使用有独立
  电源的 USB Hub，并保证 Nano 本身供电稳定；
- 首次接线和首次运动测试时抬起驱动轮、保留急停，并确认电机方向后再落地。

### 在 Nano 上识别设备

每次只插入一个设备，记录其稳定标识：

```bash
dmesg --follow
ls -l /dev/serial/by-id/
udevadm info --query=property --name=/dev/ttyUSB0
```

如果设备带唯一序列号，可在`/etc/udev/rules.d/99-dashgo-serial.rules`中按以下模板
创建别名。`<...>`必须替换成 Nano 上读取到的真实值：

```udev
SUBSYSTEM=="tty", ATTRS{idVendor}=="<VID>", ATTRS{idProduct}=="<PID>", ATTRS{serial}=="<STM32_SERIAL>", SYMLINK+="port1", GROUP="dialout", MODE="0660"
SUBSYSTEM=="tty", ATTRS{idVendor}=="<VID>", ATTRS{idProduct}=="<PID>", ATTRS{serial}=="<LIDAR_UP_SERIAL>", SYMLINK+="port2", GROUP="dialout", MODE="0660"
SUBSYSTEM=="tty", ATTRS{idVendor}=="<VID>", ATTRS{idProduct}=="<PID>", ATTRS{serial}=="<LIDAR_BACKUP_SERIAL>", SYMLINK+="port3", GROUP="dialout", MODE="0660"
SUBSYSTEM=="tty", ATTRS{idVendor}=="<VID>", ATTRS{idProduct}=="<PID>", ATTRS{serial}=="<LIDAR_DOWN_SERIAL>", SYMLINK+="port4", GROUP="dialout", MODE="0660"
```

如果多台雷达没有唯一序列号，应按 Nano/USB Hub 的物理 USB 拓扑编写 udev 规则，
不能给相同 VID/PID 的设备使用完全相同的规则。可用下面的命令查看父级拓扑：

```bash
udevadm info --attribute-walk --name=/dev/ttyUSB0
```

应用规则并重新插拔设备：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout <nano用户名>
```

加入`dialout`组后需要注销并重新登录。随后确认四个别名存在、指向四个不同设备：

```bash
ls -l /dev/port1 /dev/port2 /dev/port3 /dev/port4
readlink -f /dev/port1
readlink -f /dev/port2
readlink -f /dev/port3
readlink -f /dev/port4
```

### STM32 直接连接 Nano J41 UART（可选）

只有确认 STM32 接口为 3.3 V TTL UART 时才可直连：Nano J41 pin 8（TX）接 STM32
RX，pin 10（RX）接 STM32 TX，并连接 GND。禁止把 5 V TTL 信号接到 Nano UART。
Jetson 端还可能需要停用占用该 UART 的串口控制台：

```bash
sudo systemctl disable --now nvgetty
```

不同 Ubuntu 镜像中的设备名可能为`/dev/ttyTHS1`等。确认实际设备后，应通过 udev
将它映射为`/dev/port1`，或修改`dashgo_driver/config/base.yaml`。仓库不能确定现车
使用的是 USB 转串口还是 J41 直连，因此不要在没有核对线束和电平时切换连接方式。

## Nano 软件准备与构建

先确认 ROS Noetic 已安装，且以下文件存在：

```bash
test -f /opt/ros/noetic/setup.bash
```

Noetic 已进入 EOL。首次配置 rosdep 时按实际情况执行`sudo rosdep init`，更新索引时
显式包含 EOL 发行版，然后安装工作空间声明的依赖：

```bash
source /opt/ros/noetic/setup.bash
rosdep update --include-eol-distros
cd ~/dashgo_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release -j2
source devel/setup.bash
```

如果 Nano 内存较小或并行构建不稳定，将`-j2`改为`-j1`。每个新终端都要 source
ROS 和本工作空间；也可以将下面两行加入 Nano 用户的`~/.bashrc`：

```bash
source /opt/ros/noetic/setup.bash
source ~/dashgo_ws/devel/setup.bash
```

## 启动前检查

```bash
source /opt/ros/noetic/setup.bash
source ~/dashgo_ws/devel/setup.bash
rosdep check --from-paths ~/dashgo_ws/src --ignore-src
ls -l /dev/port1 /dev/port2 /dev/port3 /dev/port4
```

确认没有其他程序占用串口：

```bash
sudo fuser -v /dev/port1 /dev/port2 /dev/port3 /dev/port4
```

不要使用`cat`或串口终端向 STM32/YDLidar 随意发送字符，这些设备使用二进制协议。

## 运行方法

以下命令均在 Nano 上执行，并先 source ROS 与工作空间。

### 分层硬件检查

只启动 STM32 底盘链，不启动雷达：

```bash
roslaunch dashgo_bringup base.launch use_lidars:=false
```

只启动三台雷达和模型，不启动 STM32：

```bash
roslaunch dashgo_bringup base.launch use_driver:=false
```

也可以逐台检查雷达：

```bash
roslaunch ydlidar ydlidar1_up.launch
roslaunch ydlidar ydlidar2_backup.launch
roslaunch ydlidar ydlidar3_down.launch
```

### 基础底盘与键盘控制

终端 1：

```bash
roslaunch dashgo_bringup base.launch
```

终端 2：

```bash
rosrun dashgo_bringup teleop_twist_keyboard.py
```

键盘节点发布`/cmd_vel`，经速度平滑后形成`/smoother_cmd_vel`并发送给 STM32。

### GMapping 建图

`mapping.launch`已经包含基础硬件链，不要同时重复启动`base.launch`：

```bash
roslaunch dashgo_bringup mapping.launch
```

Nano 接显示器时可附加`rviz:=true`。无桌面的 Nano 应保持默认`false`，在远程电脑上
单独启动 RViz。保存地图示例：

```bash
mkdir -p ~/dashgo_maps
rosrun map_server map_saver -f ~/dashgo_maps/site_01
```

### AMCL/TEB 导航

默认使用仓库中的`bj050101.yaml`：

```bash
roslaunch dashgo_bringup navigation.launch
```

指定其他地图和初始位姿：

```bash
roslaunch dashgo_bringup navigation.launch \
  map_file:=/home/<nano用户名>/dashgo_maps/site_01.yaml \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_a:=0.0
```

导航默认启用两路激光安全门。只有在明确的诊断场景下才使用
`safety_filters:=false`，正常运动时不应关闭。

### RViz

```bash
roslaunch dashgo_bringup rviz.launch
```

建图或导航也可通过`rviz:=true`启动 RViz。若 RViz 在另一台电脑运行，两端必须
处于同一网络，并正确设置`ROS_MASTER_URI`和各自可被对方访问的`ROS_IP`。

## 运行状态检查

```bash
rostopic hz /odom
rostopic hz /imu
rostopic hz /scan
rostopic hz /scan_2
rostopic hz /scan_3
rosrun tf tf_echo odom_combined base_footprint
```

预期控制链为：

```text
/cmd_vel -> yocs_velocity_smoother -> /smoother_cmd_vel -> dashgo_driver -> STM32
```

预期定位链为：

```text
/odom + /imu -> robot_pose_ekf -> /odom_combined
```

如果 STM32 无法连接，优先检查`/dev/port1`指向、115200 波特率、`dialout`权限和
串口占用；如果某一路雷达无数据，检查对应`/dev/port2`～`/dev/port4`映射、230400
波特率和 USB 供电，不要通过交换 topic 名掩盖物理接线错误。

## 文档与验证边界

- `docs/dependency_report.md`：当前 package、系统包和硬件依赖；
- `docs/refactor_report.md`：最终功能结构、兼容性修改和已验证/未验证状态。

当前已完成源码构建、依赖解析、launch 解析和无硬件基础启动检查。Nano 真机上的
STM32通信、三雷达数据、激光安全门、GMapping和AMCL/TEB仍需逐项验收。
