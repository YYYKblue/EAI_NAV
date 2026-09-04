# Dashgo Gazebo 仿真环境

本包复用现有 `dashgo_description`、`dashgo_bringup` 和导航 YAML，在 Gazebo
Classic 中替代串口底盘与三台真实激光雷达，用于验证建图、定位、全局规划、
TEB 局部规划及安全停车链路。

RoboCup Home 场景的家具模型、DAE 网格和贴图已经放入本包，不依赖
`wpr_simulation`。原 `robocup_home.world` 本身只包含地面和墙壁；家具由
`robocup_home_objects.launch` 在 Gazebo 启动后生成。

## 模型边界

| 项目 | 仿真取值或实现 | 说明 |
|---|---|---|
| 驱动轮直径 | `0.148 m` | 与 `dashgo_driver/config/base.yaml` 一致 |
| 驱动轮轮距 | `0.445 m` | 与 `dashgo_driver/config/base.yaml` 一致 |
| 车体碰撞包络 | `0.566 x 0.566 x 0.20 m` | X/Y 与实测最外边界及导航 footprint 一致 |
| 导航 footprint | `[-0.283, 0.283] m` 方形 | 与仿真碰撞包络共用 `base_footprint` 中心 |
| 支承轮 | 四角四个低摩擦球形接触体 | 近似真实被动全向轮，不模拟滚子细节 |
| 质量与惯量 | 车体 `20 kg`，驱动轮 `0.5 kg/个` | 理想值，只用于获得稳定动力学；不能据此验证电机电流、打滑或倾覆 |
| 传感器 | 三路 2D 激光 + IMU | 话题及 frame 名沿用真机配置 |
| 默认场景 | RoboCup Home 墙体和家具 | 家具可通过启动参数关闭 |

`base_footprint` 位于两只驱动轮的地面接触平面；`base_link` 是机械本体参考点，
相对前者为 `(0.115, 0, 0.112) m`。车体 visual、collision 和理想化质心在
`base_link` 下向后偏移 `0.115 m`，因此其 X/Y 几何中心仍与
`base_footprint` 以及导航 footprint 重合。

轮径和轮距同时出现在三个位置，是因为消费者不同：

- `dashgo_driver/config/base.yaml`：真机串口驱动计算编码器里程计和轮速；
- `dashgo_description/.../base.urdf.xacro`：描述 RViz/Gazebo 中的轮子几何与关节位置；
- `dashgo_gazebo/urdf/dashgo.gazebo.xacro`：Gazebo 差速驱动插件计算左右轮速度和仿真里程计。

它们不会因“参数名相同”自动冲突，也不会自动同步；各节点只读取自己的配置。
因此修改真机标定值时，必须同步检查以上三个位置。目前统一为直径 `0.148 m`、
轮距 `0.445 m`。

主激光 `laser_frame` 的仿真 TF 与真机一致：在 `base_footprint` 下为
`(0.174, -0.176, 0.28)`、yaw `-0.785 rad`。它位于实测车体包络内的右前区域，
此前看起来几乎在实体外，是旧 `0.40 m` URDF 外观小于实测包络造成的，不是 TF
填错。主激光原始数据发布为 `/scan_raw`；仿真专用
`LaserScanBoxFilter` 会删除机身包络内的自体回波，再以 `/scan` 提供给建图、
AMCL、Costmap 和安全区节点。

低位激光 `laser_frame_2` 则确实在仿真中放在车体前缘外侧。这是必要的理想化：
真机坐标 `(0.17, 0, 0.05)` 落在当前完整方盒碰撞体内部，而简化模型没有表达
真实底盘的安装孔和缺口；完全照搬会让 Gazebo ray 从碰撞体内部发射。其 frame
名、朝向、扫描范围和高度分层用途仍与真机一致。若以后把底盘 collision 拆成
能表达真实缺口的组合几何，再把该雷达恢复到真机坐标。

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
`(0, 0, 0)` 生成机器人，避免与位于 `(-2.1, -2.2)` 的茶几重叠。四种仿真入口默认加载 `robocup_home.world`，
并生成床、沙发、茶几、书架、餐桌、柜子和椅子。

如果临时切换到其他 world，应关闭 RoboCup Home 家具：

```bash
roslaunch dashgo_gazebo sim_base.launch \
  world:=/absolute/path/to/another.world \
  spawn_scene_objects:=false
```

`sim_navigation.launch` 当前默认静态地图是 `maps/robocup_home.yaml`。如果你用
旧版本（尚未过滤自体回波）的 `/scan` 建过图，建议重新建图并保存，避免旧地图
中残留跟随机器人产生的伪障碍痕迹；也可通过 `map_file:=...` 传入其他地图。

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

主要输出：`/scan`（过滤后）、`/scan_raw`（仅诊断）、`/scan_2`、`/scan_3`、`/imu`、`/odom`、
`/odom_combined`、`/joint_states`。导航模式的主 TF 链应为：

```text
map -> odom_combined -> base_footprint -> base_link -> sensors/wheels
```

推荐每次修改参数后至少检查：

```bash
rostopic hz /scan /scan_2 /scan_3 /odom /imu
rostopic hz /scan_raw /scan
rostopic echo /is_passed
rostopic echo /is_passed_2
rosrun tf tf_echo map base_footprint
```

包内的 `navigation_test.world` 包含外墙、分隔墙和方形障碍物，可用于规划绕障，
但不代表真实场地。
当前实测短距离与分隔墙绕行均能返回 `Goal reached`；长距离到达后 AMCL 仍可能
继续修正地图位姿，因此应结合 Gazebo 真值和 `/amcl_pose` 评估定位误差，不能只
以 action 成功作为定位精度结论。

## 第三方场景资源

RoboCup Home 家具模型和网格来自 `wpr_simulation`，原项目声明为 GPLv3。
许可证副本位于 `third_party_licenses/wpr_simulation_GPL-3.0.txt`。为解除运行时
包依赖，模型中的 mesh URI 已从 `package://wpr_simulation/...` 改为
`package://dashgo_gazebo/...`，模型几何、碰撞体和场景生成位姿保持不变。
