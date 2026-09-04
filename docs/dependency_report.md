# Dashgo 当前运行依赖报告

## 报告边界

本报告只描述当前`dashgo_ws`中的最终源码和实际启动入口。旧功能包、旧工作空间和
`总体目标.md`均已从工作区删除，不再作为运行时依赖或文档入口。

真机部署平台为 NVIDIA Jetson Nano、Ubuntu 20.04（ARM64/aarch64）和 ROS 1
Noetic；仓库同时包含 Gazebo Classic 仿真。开发环境中的 WSL/Gazebo 验证不能
替代 Nano 真机验证。

## 本地 package 与职责

| package | 职责 | 主要依赖 |
|---|---|---|
| `dashgo_driver` | STM32协议、编码器、`/odom`、`/imu`、急停与速度输出 | rospy、PySerial、NumPy、geometry/nav/sensor/std_msgs、tf |
| `ydlidar` | 三台 YDLidar 的 C++驱动核心与静态TF | roscpp、rosconsole、sensor_msgs、tf |
| `dashgo_description` | Xacro、RobotModel和底盘link/joint TF | xacro、robot_state_publisher、joint_state_publisher |
| `yocs_velocity_smoother` | `/cmd_vel`到`/smoother_cmd_vel`的速度/加速度平滑 | roscpp、nodelet、pluginlib、dynamic_reconfigure |
| `dashgo_bringup` | 基础启动、建图、导航、RViz、teleop及激光安全门 | 上述本地包与Noetic导航栈 |
| `dashgo_gazebo` | 仿真底盘、三激光、IMU、场景、地图与安全速度门 | gazebo_ros、gazebo_plugins、laser_filters及bringup/description |

本地依赖关系为：

```text
dashgo_bringup
├── dashgo_driver
├── ydlidar
├── dashgo_description
├── yocs_velocity_smoother
└── ROS Noetic 系统包

dashgo_gazebo
├── dashgo_bringup
├── dashgo_description
└── Gazebo/ROS 仿真系统包
```

## 启动入口与递归依赖

| 功能 | 入口 | 递归启动的关键组件 |
|---|---|---|
| 基础硬件 | `roslaunch dashgo_bringup base.launch` | STM32、三雷达、模型TF、robot_pose_ekf、速度平滑 |
| GMapping建图 | `roslaunch dashgo_bringup mapping.launch` | 基础硬件、slam_gmapping |
| 边建图边导航 | `roslaunch dashgo_bringup mapping_navigation.launch` | 基础硬件、slam_gmapping、move_base、TEB、安全门 |
| AMCL/TEB导航 | `roslaunch dashgo_bringup navigation.launch` | 基础硬件、map_server、AMCL、move_base、GlobalPlanner、TEB、安全门 |
| RViz | `roslaunch dashgo_bringup rviz.launch` | RViz及当前配置 |
| 键盘控制 | `rosrun dashgo_bringup teleop_twist_keyboard.py` | rospy、geometry_msgs、终端输入 |
| Gazebo基础链 | `roslaunch dashgo_gazebo sim_base.launch` | gzserver、仿真机器人、三激光、IMU、EKF、速度链 |
| Gazebo建图 | `roslaunch dashgo_gazebo sim_mapping.launch` | Gazebo基础链、slam_gmapping |
| Gazebo边建图边导航 | `roslaunch dashgo_gazebo sim_mapping_navigation.launch` | Gazebo基础链、slam_gmapping、move_base、TEB、安全门 |
| Gazebo定位导航 | `roslaunch dashgo_gazebo sim_navigation.launch` | Gazebo基础链、map_server、AMCL、move_base、TEB、安全门 |

`mapping.launch`和`navigation.launch`已经包含`base.launch`，运行时不能再重复启动
基础入口，否则会产生节点名、串口和TF冲突。

## 系统 ROS 与 Python 依赖

主要外部依赖为：

- GMapping、map_server、AMCL、move_base、costmap_2d、GlobalPlanner和TEB；
- robot_pose_ekf、robot_state_publisher、joint_state_publisher；
- RViz、tf、nodelet、pluginlib、dynamic_reconfigure和xacro；
- Gazebo Classic、gazebo_ros、gazebo_plugins和laser_filters；
- Python 3、`python3-numpy`和`python3-serial`；
- catkin、C++编译器及常规 ROS Noetic 消息包。

依赖以各 package 的`package.xml`为准。在 Nano 上执行：

```bash
source /opt/ros/noetic/setup.bash
cd ~/dashgo_ws
rosdep update --include-eol-distros
rosdep install --from-paths src --ignore-src -r -y
rosdep check --from-paths src --ignore-src
```

Noetic 已进入 EOL，因此刷新 rosdep 索引时需要`--include-eol-distros`。截至
2026-09-04，当前开发环境的`rosdep check`返回所有已声明系统依赖均满足；Nano 是独立
部署环境，仍需在板端重新执行依赖检查。

这里的“均满足”只检查 manifest 已声明内容。`ydlidar`的三个雷达 launch 还直接使用
`tf/static_transform_publisher`，但`ydlidar/package.xml`尚未声明`tf`运行依赖；当前
工作空间因`dashgo_bringup`同时依赖`tf`而没有暴露该缺项，单独安装`ydlidar`时可能失败。

## 硬件与串口依赖

| 逻辑设备 | 硬件 | 波特率 | topic/frame |
|---|---|---:|---|
| `/dev/port1` | STM32底盘控制器 | 115200 | `/odom`、`/imu`及底盘状态 |
| `/dev/port2` | `ydlidar1_up` | 230400 | `/scan`、`laser_frame` |
| `/dev/port3` | `ydlidar2_backup` | 230400 | `/scan_3`、`laser_frame_3` |
| `/dev/port4` | `ydlidar3_down` | 230400 | `/scan_2`、`laser_frame_2` |

四个`/dev/port*`名称是部署约定，不是 Linux 默认编号。Nano 上必须根据设备唯一
序列号或 USB 物理拓扑配置 udev 规则，并将运行用户加入`dialout`组。仓库没有实际
设备的 VID、PID、序列号和插座编号，不能从源码推断物理 USB 口。

详细接线、udev 模板、J41 UART 可选方案和逐路检查命令见根目录`README.md`。

## ROS 数据依赖

### 底盘与定位

```text
STM32 encoder -> /odom --+
STM32 IMU -----> /imu  --+-> robot_pose_ekf -> /odom_combined + TF

/cmd_vel -> yocs_velocity_smoother -> /smoother_cmd_vel
          -> dashgo_driver -> STM32
```

驱动配置`useImu: true`时，底盘驱动只发布`/odom`消息，不发布
`odom -> base_footprint` TF；该 TF 由`robot_pose_ekf`以
`odom_combined -> base_footprint`形式提供。

### 雷达、建图与导航

```text
/scan   -> GMapping / AMCL / global+local costmap / laser_safety_gate
/scan_2 -> global+local costmap / laser_safety_gate_2
/scan_3 -> global+local costmap
```

GMapping依赖`/scan`、`odom_combined`和`base_footprint`。导航依赖地图、AMCL、
三路雷达、TF和move_base。激光安全门输出`/is_passed`和`/is_passed_2`，底盘驱动
在任一计数大于2时禁止正向速度。当前 ROS 驱动代码虽然在急停状态下发布零值
`robot_cmd_vel`，但没有同步清零实际轮速计算使用的`x/th`；因此不能从仓库代码得出
“急停软件链优先停车”的结论，必须核对 STM32 固件并做抬轮测试。

### Gazebo替代链

```text
/scan_raw -> LaserScanBoxFilter -> /scan
/scan + /scan_2 + /scan_3 + /imu + /odom -> 原 bringup 定位/导航链
/cmd_vel -> velocity_smoother -> /smoother_cmd_vel
         -> sim_safety_cmd_gate -> /gazebo_cmd_vel -> gazebo_ros_diff_drive
```

Gazebo差速插件的`wheelAcceleration`为`0.0`，禁用插件内部的第二层加速度限制；
`yocs_velocity_smoother`仍然启用。仿真不会执行 STM32 串口协议、固件急停、真实雷达
噪声、USB供电或底盘摩擦标定，因此不能替代这些硬件依赖的验收。

## 明确不在当前依赖范围内

- rosbridge、网页或 APP 通信；
- 多目标、取消目标及厂商业务动作服务器；
- 相机点云和已禁用的 sonar costmap 数据源；
- 历史地图、备份launch、Python 2字节码、测试示例和IDE文件；
- ROS导航栈的本地源码副本；导航组件使用 Noetic 系统包。

## 当前验证边界

已完成 6 个 package 构建、Python/XML/YAML解析、Xacro展开、rosdep检查、17 个
bringup/Gazebo/description/YDLidar launch 文件解析，以及 Gazebo 基础链和完整定位
导航链的无界面短时启动。完整导航启动中地图、AMCL、global/local costmap、TEB、
安全节点、三激光、IMU、里程计与 EKF 均完成初始化。

尚未完成 Jetson Nano 板端部署、STM32实机通信、三台雷达数据、真实障碍安全门、
GMapping实地建图和AMCL/TEB实车导航；本轮也没有重新执行仿真目标点运动精度测试。
它们是部署/行为验收项，不应由构建或短时启动成功代替。当前问题详见
`docs/current_issues.md`。
