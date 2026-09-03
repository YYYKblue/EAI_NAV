# Dashgo Gazebo 仿真环境

本包复用现有 `dashgo_description`、`dashgo_bringup` 和导航 YAML，在 Gazebo
Classic 中替代串口底盘与三台真实激光雷达，用于验证建图、定位、全局规划、
TEB 局部规划及安全停车链路。

## 模型边界

| 项目 | 仿真取值或实现 | 说明 |
|---|---|---|
| 驱动轮直径 | `0.148 m` | 与 `dashgo_driver/config/base.yaml` 一致 |
| 驱动轮轮距 | `0.445 m` | 与 `dashgo_driver/config/base.yaml` 一致 |
| 车体碰撞盒 | `0.40 x 0.40 x 0.20 m` | 保留原 URDF 值 |
| 导航 footprint | 原 `dashgo_bringup` 多边形 | 仿真不另建一套 footprint |
| 支承轮 | 四角四个低摩擦球形接触体 | 近似真实被动全向轮，不模拟滚子细节 |
| 质量与惯量 | 车体 `20 kg`，驱动轮 `0.5 kg/个` | 理想值，只用于获得稳定动力学；不能据此验证电机电流、打滑或倾覆 |
| 传感器 | 三路 2D 激光 + IMU | 话题及 frame 名沿用真机配置 |

轮径和轮距同时出现在三个位置，是因为消费者不同：

- `dashgo_driver/config/base.yaml`：真机串口驱动计算编码器里程计和轮速；
- `dashgo_description/.../base.urdf.xacro`：描述 RViz/Gazebo 中的轮子几何与关节位置；
- `dashgo_gazebo/urdf/dashgo.gazebo.xacro`：Gazebo 差速驱动插件计算左右轮速度和仿真里程计。

它们不会因“参数名相同”自动冲突，也不会自动同步；各节点只读取自己的配置。
因此修改真机标定值时，必须同步检查以上三个位置。目前统一为直径 `0.148 m`、
轮距 `0.445 m`。

低位激光在仿真中放在车体前缘外侧。原因是 Gazebo Classic 的 ray 传感器不会
自动滤除本机器人碰撞体；完全照搬真机安装点会扫到车体或全向支承轮，并在代价
地图中生成假障碍。其 frame 名、朝向、扫描范围和高度分层用途仍与真机一致。

## 启动

```bash
cd /home/bboss/dashgo_ws
catkin_make
source devel/setup.bash
```

只启动底盘、三路激光、IMU、里程计和 EKF：

```bash
roslaunch dashgo_gazebo sim_base.launch
```

建图：

```bash
roslaunch dashgo_gazebo sim_mapping.launch
```

建图并同时启动导航：

```bash
roslaunch dashgo_gazebo sim_mapping_navigation.launch
```

使用包内测试地图定位导航：

```bash
roslaunch dashgo_gazebo sim_navigation.launch
```

无图形界面运行时追加 `gui:=false rviz:=false`。`sim_navigation.launch` 默认在
`(-2.0, -2.0, 0)` 生成机器人，并加载与测试世界匹配的 8 m x 8 m 栅格地图。

## 接口与验证点

控制链路为：

```text
move_base /cmd_vel
  -> yocs_velocity_smoother /smoother_cmd_vel
  -> sim_safety_cmd_gate /gazebo_cmd_vel
  -> gazebo_ros_diff_drive
```

`sim_safety_cmd_gate` 复现真机驱动当前逻辑：任一安全区计数 `/is_passed` 或
`/is_passed_2` 大于 `2` 时，只禁止正向 `linear.x`，仍允许后退和转向；超过
1 秒未收到速度命令时输出零速度。

主要输出：`/scan`、`/scan_2`、`/scan_3`、`/imu`、`/odom`、
`/odom_combined`、`/joint_states`。导航模式的主 TF 链应为：

```text
map -> odom_combined -> base_footprint -> base_link -> sensors/wheels
```

推荐每次修改参数后至少检查：

```bash
rostopic hz /scan /scan_2 /scan_3 /odom /imu
rostopic echo /is_passed
rostopic echo /is_passed_2
rosrun tf tf_echo map base_footprint
```

测试世界包含外墙、分隔墙和方形障碍物，可用于规划绕障，但不代表真实场地。
当前实测短距离与分隔墙绕行均能返回 `Goal reached`；长距离到达后 AMCL 仍可能
继续修正地图位姿，因此应结合 Gazebo 真值和 `/amcl_pose` 评估定位误差，不能只
以 action 成功作为定位精度结论。
