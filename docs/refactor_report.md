# Dashgo ROS Noetic 最终状态报告

## 当前结论

工作区已收敛为 5 个运行 package，旧功能包和`总体目标.md`已经删除。当前文档、
构建和运行均以现有源码为准，不再依赖已删除目录。

最终部署目标是 NVIDIA Jetson Nano、Ubuntu 20.04（ARM64/aarch64）、ROS 1
Noetic和Python 3。开发环境中的构建与无硬件检查已经完成，Nano真机验收尚未完成。

## 最终运行入口

| 功能 | 命令 |
|---|---|
| 基础硬件链 | `roslaunch dashgo_bringup base.launch` |
| GMapping建图 | `roslaunch dashgo_bringup mapping.launch` |
| AMCL/TEB导航 | `roslaunch dashgo_bringup navigation.launch` |
| RViz | `roslaunch dashgo_bringup rviz.launch` |
| 键盘控制 | `rosrun dashgo_bringup teleop_twist_keyboard.py` |

建图和导航入口已经递归包含基础硬件链，不应另行重复启动`base.launch`。建图和导航
可通过`rviz:=true`同时启动 RViz；无桌面的 Nano 应保持默认`false`。

导航可指定地图和初始位姿：

```bash
roslaunch dashgo_bringup navigation.launch \
  map_file:=/absolute/path/to/map.yaml \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_a:=0.0
```

## 最终 package 职责

```text
src/
├── dashgo_driver/          STM32、编码器、odom、IMU、急停和底盘速度接口
├── ydlidar/                YDLidar 1.3.1驱动核心及三雷达launch
├── dashgo_description/     导航TF和RViz RobotModel所需Xacro
├── yocs_velocity_smoother/ /cmd_vel速度/加速度平滑nodelet
└── dashgo_bringup/         基础启动、建图、导航、RViz、teleop、安全和配置
```

`yocs_velocity_smoother`来自上游`yujinrobot/yujin_ocs`提交
`17337e5a2d0a0f3711c55e272e656eb59174d657`的最小运行子集，来源见其
`UPSTREAM.md`。算法保持不变，旧ECL线程包装替换为C++11 `std::thread`。

## launch 职责

- `base.launch`：STM32、三雷达、URDF、IMU静态TF、robot_pose_ekf和速度平滑；
- `mapping.launch`：在基础链之上启动GMapping；
- `navigation.launch`：在基础链之上启动地图、AMCL、move_base、GlobalPlanner、
  TEB和两路激光安全门；
- `rviz.launch`：只启动RViz及当前配置。

`base.launch`提供`use_driver`、`use_lidars`和`use_description`开关，便于在 Nano 上
逐层诊断。默认值均为`true`。

## 串口和硬件接口

| 逻辑端口 | 设备 | 波特率 | ROS接口 |
|---|---|---:|---|
| `/dev/port1` | STM32 | 115200 | 发布`/odom`、`/imu`，订阅`/smoother_cmd_vel` |
| `/dev/port2` | `ydlidar1_up` | 230400 | `/scan`、`laser_frame` |
| `/dev/port3` | `ydlidar2_backup` | 230400 | `/scan_3`、`laser_frame_3` |
| `/dev/port4` | `ydlidar3_down` | 230400 | `/scan_2`、`laser_frame_2` |

这些是 udev 持久化逻辑名称。仓库不含现车 USB 设备的 VID、PID、序列号或物理
插座编号；Nano部署时必须逐台识别并建立规则。完整接线、权限和检查步骤见根目录
`README.md`。

## ROS 数据链

### 基础与控制

```text
STM32 encoder --> /odom --+
STM32 IMU ------> /imu  --+-> robot_pose_ekf -> /odom_combined + TF

teleop 或 move_base
        -> /cmd_vel
        -> yocs_velocity_smoother
        -> /smoother_cmd_vel
        -> dashgo_driver
        -> STM32
```

底盘仍订阅`smoother_cmd_vel`，没有改为直接订阅`cmd_vel`。驱动配置
`useImu: true`时不发布`odom -> base_footprint` TF；融合定位链使用
`odom_combined -> base_footprint`。

### 雷达、安全、建图与导航

```text
/scan   -> GMapping / AMCL / costmap / laser_safety_gate -> /is_passed
/scan_2 -> costmap / laser_safety_gate_2                 -> /is_passed_2
/scan_3 -> costmap

/scan + TF + /odom_combined -> slam_gmapping -> /map
/scan + /map + TF           -> AMCL + move_base
```

底盘驱动在`is_passed* > 2`时禁止正向速度；STM32急停状态优先输出零速度。正常导航
默认启用两路安全门，不应为日常运行关闭`safety_filters`。

## TF 发布关系

| TF | 发布者 |
|---|---|
| `map -> odom_combined` | 建图时GMapping；导航时AMCL，二者不同时启动 |
| `odom_combined -> base_footprint` | `robot_pose_ekf` |
| `base_footprint -> base_link` | `robot_state_publisher`根据Xacro发布 |
| `base_link -> wheel/front_flag` | `robot_state_publisher`根据Xacro发布 |
| `base_footprint -> imu_base` | `static_transform_publisher` |
| `base_footprint -> laser_frame*` | 三个YDLidar launch中的静态TF |

## 当前功能范围

- STM32底盘协议、编码器、IMU、里程计、急停和速度控制；
- 三台 YDLidar 的固定端口、波特率、角度、range、topic和TF；
- GMapping、AMCL、GlobalPlanner、TEB、costmap及恢复参数；
- 默认`bj050101.yaml/.pgm`地图；
- RViz配置、RobotModel和Xacro；
- 键盘控制、速度平滑和两路前向激光安全门。

当前工作区明确不包含 rosbridge、网页/APP通信、多目标/取消目标、厂商业务动作、
相机点云、已禁用sonar数据源、历史地图和ROS导航栈源码副本。

## Noetic/Python 3兼容性修改

### 底盘驱动

- 使用Python 3 shebang和语法；
- 串口接收状态机统一处理`bytes`；
- PySerial参数使用`write_timeout`；
- 修正`reset_IMU`调用名和shutdown发布；
- STM32数据包布局、校验、里程计和运动学公式保持不变。

### teleop 与模型

- teleop移除rosbuild时代的`roslib.load_manifest`，退出时恢复终端并发送零速度；
- Xacro宏调用显式使用`xacro:`命名空间；
- `robot_state_publisher`使用Noetic可执行文件名。

### C++构建

- 所有 package 使用catkin；
- YDLidar只编译当前ROS节点和Linux SDK核心；
- 速度平滑以C++11标准线程替代旧ECL包装。

## 系统依赖状态

截至2026-08-10，当前开发环境已满足所有 package 声明的系统依赖：

- `rosdep check --from-paths src --ignore-src`返回
  `All system dependencies have been satisfied`；
- `gmapping`、`move_base`、`map_server`、`amcl`、`costmap_2d`、
  `global_planner`、`teb_local_planner`和`robot_pose_ekf`均可由`rospack`定位；
- 五个本地 package 可在 Ubuntu 20.04 / ROS Noetic 下完成catkin构建。

Nano是独立目标机，复制工作空间后仍需在板端重新执行`rosdep install`、
`rosdep check`和`catkin_make`。Noetic已进入EOL，更新rosdep索引时使用：

```bash
rosdep update --include-eol-distros
```

## 验证状态

### 已完成

- 删除旧`build/devel`后的干净`catkin_make`；
- YDLidar驱动和yocs nodelet的C++编译/链接；
- Python 3脚本和动态配置的语法编译；
- STM32接收状态机的Python 3字节包静态烟雾测试；
- 13份YAML及全部XML、launch和Xacro解析；
- Xacro展开并生成`base_footprint`、`base_link`和左右轮link；
- 五个最终命令对应的 launch/可执行文件解析，其中四个launch完成
  `roslaunch --files`和`roslaunch --nodes`检查；
- `rosdep check`及八个主要标准ROS运行包的`rospack find`复验；
- `use_driver:=false use_lidars:=false`条件下短时启动`base.launch`，基础节点持续
  运行到测试主动结束。

无硬件基础启动时，URDF报告`iRobot/LightGrey`材质未定义。该警告只影响模型显示
颜色，不影响link/joint、TF或导航数据链。

### 尚未完成

- Jetson Nano Ubuntu 20.04板端构建和开机启动；
- `/dev/port1`的STM32实机通信、编码器、IMU、急停及电机方向；
- `/dev/port2`～`/dev/port4`的三台YDLidar实机数据和USB供电稳定性；
- 两路激光安全门的真实障碍和停车距离验证；
- 实际场地GMapping建图；
- AMCL定位、GlobalPlanner/TEB导航和重复运行稳定性。

构建、静态解析和无硬件启动只证明源码集成层成立，不等同于 Nano、传感器或机器人
运动行为验收。

## Nano部署验收顺序

1. 在 Nano 上安装依赖并构建，确认`rosdep check`通过；
2. 建立并核对`/dev/port1`～`/dev/port4`，确认运行用户具有`dialout`权限；
3. 抬起驱动轮，仅启动 STM32，检查`/odom`、`/imu`、急停和电机方向；
4. 逐台启动 YDLidar，核对topic、frame、安装方向和扫描范围；
5. 启动完整基础链，检查`/odom_combined`及TF树；
6. 低速验证键盘控制、速度平滑和安全停车；
7. 进行GMapping建图并保存地图；
8. 验证AMCL/TEB导航，最后再进行长时间稳定性测试。
