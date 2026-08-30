# 配置联动、风险与调参验证方法

本文不直接修改当前 YAML 数值，而是把代码中能够确认的联动关系、风险优先级和实机验证方法列清楚。标为“建议验证”的内容不是对机器人现状的断言，必须用日志和实测决定是否调整。

## 1. 当前配置中已经一致的部分

| 检查项 | 结果 | 依据 |
|---|---|---|
| Costmap 与 TEB 外形 | 一致 | 两处都是 `±0.283 m` 的四点正方形 polygon |
| 静态地图、Gmapping、Costmap 分辨率 | 一致 | 都是 `0.05 m/cell` |
| 局部 Costmap 与 TEB frame | 一致 | 都使用 `odom_combined` |
| 差速运动模型 | 一致 | AMCL `diff`、TEB `min_turning_radius=0`、`weight_kinematics_nh=1000` |
| TEB 与速度平滑器线速度上限 | 一致 | 两者都是 `0.6 m/s` |
| TEB 与速度平滑器线/角加速度 | 一致 | 两者都是 `0.3`，单位分别为 m/s²、rad/s² |
| TEB 角速度与平滑器上限 | 有余量 | TEB `0.30 rad/s`，smoother `0.5 rad/s` |
| 三台激光安装高度与 Costmap 窗口 | 匹配 | 约 0.05、0.28、0.35 m，分别落在三组高度区间内 |
| 静态层和实时层组合 | 明确 | 两套 Costmap 都是 Static + Voxel + Inflation |

这些一致性只证明参数之间没有直接矛盾，不证明机器人已经达到最优或安全性能。

## 2. 需要优先关注的代码事实

### 2.1 激光安全门不是 fail-safe

完整链路是：

```text
/scan、/scan_2
   -> laser_safety_gate.py 统计盒内点数
   -> /is_passed、/is_passed_2
   -> dashgo_driver：任一计数 > 2 且 x > 0 时，令 x = 0
```

已经确认的限制：

1. 安全节点 TF 查询失败或雷达不再发布时，本帧不产生新计数。
2. 驱动没有计数话题超时判断，保留上次状态；启动初始状态是允许前进。
3. `safety_filters:=false` 时驱动仍以允许前进初始化。
4. 单独运行 `mapping.launch` 时没有包含安全节点，驱动同样以允许前进初始化。
5. 门控只禁止正向 X，不禁止倒车和角速度；正向转弯受阻后会变成原地旋转。
6. `/scan_3` 没有进入安全门。
7. 阈值 `> 2` 写死在驱动代码，不可通过这两个 YAML 调整。
8. 盒体前缘只比 polygon 前缘多 `0.167 m`。

`base.launch` 加载的 `dashgo_driver/config/base.yaml` 将驱动内部 `accel_limit` 配为 `1.0`。以 `0.6 m/s` 为例，即便假设实际制动减速度能立即达到 `1.0 m/s²`，理想连续模型的制动距离也约为：

```text
d = v² / (2a) = 0.6² / (2 × 1.0) = 0.18 m
```

这还没有计入约 7 Hz 激光采样、ROS/TF 处理、命令周期、轮胎打滑和驱动执行延迟，已经略大于框在 polygon 前方的 0.167 m 外伸。因此当前安全框不能被解释为“在 0.6 m/s 下保证碰撞前停车”。这个计算只是风险下界提示，实际制动距离必须装载真实负载，在不同地面上实测。

建议优先实测并记录：

- 0.1、0.2、0.4、0.6 m/s 下，从第三个框内点出现到线速度为 0 的距离和时间；
- 单台雷达断线、TF 中断、安全节点退出时驱动行为；
- 障碍位于方形角点旋转扫掠区时，门控能否防止接触；
- 玻璃、黑色材料、细杆和斜面是否稳定产生至少 3 个点。

### 2.2 安全框与机器人 polygon 重叠

```text
robot: x=[-0.283,0.283], y=[-0.283,0.283]
box 1: x=[0.10,0.45],   y=[-0.30,0.30]
box 2: x=[0.17,0.45],   y=[-0.29,0.29]
```

如果雷达能看到机身、支架、线缆或载荷，静止无障碍时也可能有 3 个以上点。不要先改框；先采集空场数据，分别统计两个计数的最小值、最大值、P95/P99 和角度来源，再决定是调盒体、雷达 `ignore_array`、点数阈值还是连续帧条件。

### 2.3 动态障碍响应的时间链

当前时间尺度：

| 环节 | 当前值 | 近似周期 |
|---|---:|---:|
| 激光驱动目标频率 | 7 Hz | 0.143 s |
| 局部 Costmap 更新 | 3 Hz | 0.333 s |
| move_base 控制循环 | 4 Hz | 0.250 s |
| velocity smoother | 20 Hz | 0.050 s |
| TEB `dt_ref` | 0.45 s | 轨迹位姿参考间隔，不是循环周期 |
| 底盘控制器 | 30 Hz | 0.033 s |

局部 Costmap 比控制循环慢，所以一部分 TEB 周期会复用旧障碍地图。`dt_ref=0.45 s` 也比控制周期 0.25 s 粗。它们不是配置错误，但如果出现动态障碍响应慢、轨迹离散碰撞检查漏得较远或速度指令抖动，建议在 CPU 有余量的前提下依次实验：

1. 把局部 Costmap 提高到与控制频率相同或略高，例如 4～5 Hz。
2. 把 `dt_ref` 向 0.25～0.30 s 试验，并把 `dt_hysteresis` 调到约 `dt_ref` 的 10%～20%。
3. 再评估是否需要提高 `controller_frequency`；不要先把所有频率一起提高。

每一步都应记录控制周期超时、CPU、TEB 规划时间和轨迹成功率。计算资源不足时，提高频率反而会造成更大延迟。

### 2.4 传感器掉线不会由 Costmap 参数主动判定

三个源的 `expected_update_rate` 都是 0，表示关闭更新超时检查。VoxelLayer 可能继续保留或通过其他雷达清除已有信息，但不会因为某台激光停止更新而把导航直接置为失败。

建议验证是否需要设置为略大于真实周期的秒数，例如对稳定 7 Hz 数据先观测实际 P99 间隔，再选择有余量的超时值。不能直接写 `7.0`，因为该参数单位是秒，不是 Hz。

### 2.5 主激光高度窗口上余量较小

主激光 `/scan` 的 TF 高度约 0.28 m，而允许范围为 `[0.21,0.30] m`，离上界只有约 0.02 m。固定 TF 只有 yaw 时扫描平面高度稳定；如果以后校准加入 pitch/roll，远处点的 Z 可能随距离变化并被部分过滤。

验证方法是在 RViz 同时显示原始 LaserScan 和 VoxelGrid，或临时把扫描转换成 PointCloud2 后查看变换到 `base_footprint` 的 Z 分布。窗口应覆盖正常数据的统计分布，同时排除不希望的高度层。

## 3. 建议按现象评估的参数

### 3.1 定位 CPU 高或更新慢

当前 AMCL 最少 2000、最多 5000 粒子，每次最多使用 60 束。若 CPU 高：

1. 先确认实际粒子数是否长期顶在 5000；
2. 检查 `odom_alpha3=0.8` 是否由真实里程误差支持；
3. 在定位精度、重定位成功率不下降的前提下，逐步降低 `min_particles`；
4. 最后才考虑降低 `laser_max_beams`。

不要为了降 CPU 直接同时减少粒子和激光束，否则很难判断精度下降来自哪里。

### 3.2 机器人被搬动后无法恢复

`recovery_alpha_slow=0` 且 `recovery_alpha_fast=0`，AMCL 的随机粒子注入恢复关闭。如果存在“绑架机器人”需求，可试验启用一慢一快的平均权重滤波，但必须防止正常动态障碍导致频繁随机注入。更可控的方法是由上层在确认位姿失效后重新发布 `/initialpose` 或执行全局定位服务。

### 3.3 全局规划很快进入恢复

`planner_frequency=1 Hz`、`max_planning_retries=1`、`planner_patience=3 s`。重试次数限制可能比 3 s patience 更早满足。如果暂态 TF/Costmap 抖动就触发清图，应从日志确认触发条件，再决定增加 retry、增加 patience 或修复上游数据，而不是直接关闭恢复。

### 3.4 路径贴墙或窄门过不去

先区分是哪一层拒绝：

| 现象 | 优先检查 |
|---|---|
| 全局路径本身绕得很远/无路径 | 静态地图、`inflation_radius`、`cost_scaling_factor`、`allow_unknown` |
| 全局路径通过但 TEB 不出速度 | 两处 polygon 是否一致、`min_obstacle_dist`、局部障碍、feasibility check |
| TEB 轨迹有但控制抖动 | `dt_ref`、控制频率、加速度限制、里程计速度、优化收敛 |
| 贴墙但没有碰撞 | 先确认是否真的需要更大余量，再减小 `cost_scaling_factor` 或调整 TEB 距离/权重 |
| 窄门物理能过但规划不过 | 测量门宽、地图墙厚、定位方差；不要先缩小真实 footprint |

当前满足 TEB 两侧各 0.15 m 期望间距的粗略通道宽度约 0.866 m。方形原地旋转不加余量也需约 0.800 m 的圆形扫掠直径，窄门“直行能过”不代表“门内能转身”。

### 3.5 倒车几乎不发生

不是 `max_vel_x_backwards=0.15` 没生效，而是 `weight_kinematics_forward_drive=100` 强烈惩罚倒车。只有明确确认后方感知足够、任务需要倒车脱困时，才逐步降低此前进偏好权重。同时注意安全框不限制倒车，而 TEB 只考虑机器人后方 1.0 m 内的 Costmap 障碍。

### 3.6 到达目标位置可以但朝向偏差大

`yaw_goal_tolerance=0.5 rad`，约 28.65°，本来就允许较大朝向误差。精确停靠任务可逐步减小到任务允许值，但要同步检查：

- AMCL 航向方差；
- `max_vel_theta` 和低速角速度控制死区；
- 方形 polygon 旋转空间；
- 目标附近局部地图是否允许完成最后旋转。

### 3.7 边建图边导航不能规划到未知区

这是 `GlobalPlanner/allow_unknown=false` 的直接结果，不是 Gmapping 没有更新。若任务是自主探索，仅改成 true 也不够：还需要 frontier/探索目标生成器，并确认未知区规划和安全策略。当前 launch 更适合“在已经建出的区域内边完善地图边导航”。

## 4. 性能优化项

### 4.1 两套 VoxelLayer 都发布体素图

公共配置在全局和局部 Costmap 各加载一次，`publish_voxel_map=true` 也生效两次。调试结束后若不需要三维体素显示，可关闭发布；这不会关闭障碍层本身。

### 4.2 全局 Costmap 也融合三台实时激光

这是当前公共插件表带来的明确行为。优点是全局路径会考虑实时障碍，缺点是维护两套 VoxelLayer、三套订阅各两份。如果 CPU/内存不足，可把全局与局部 `plugins` 拆开配置，但这属于架构调整：必须明确全局是否仍需要实时障碍，不能只为了“减少重复”直接删层。

### 4.3 局部窗口与前视距离

局部窗口为 `3 m × 3 m`，机器人位于中心时正前方约只有 1.5 m 可见范围；TEB `max_global_plan_lookahead_dist=3 m` 还会受局部地图尺寸限制。`obstacle_range=2 m` 也无法把窗口外的障碍写进局部图。

若高速时前视不足，应一起评估窗口、地图更新成本、传感器可靠距离和制动距离。单独把 lookahead 调到更大不会凭空获得窗口外障碍。

## 5. 运行前静态检查

### 5.1 YAML 语法

工作区已有 ROS Python 环境时：

```bash
python3 -c 'import glob,yaml; [yaml.safe_load(open(f)) for f in glob.glob("src/dashgo_bringup/**/*.yaml", recursive=True)]; print("YAML OK")'
```

### 5.2 launch XML

```bash
for f in src/dashgo_bringup/launch/*.launch; do xmllint --noout "$f"; done
```

### 5.3 两处 polygon 人工核对

```bash
rg -n -A2 'footprint:|vertices:' src/dashgo_bringup/config/navigation
```

## 6. 启动后的参数和数据验证

### 6.1 确认实际加载值

```bash
rosparam get /move_base/global_costmap/footprint
rosparam get /move_base/local_costmap/footprint
rosparam get /move_base/TebLocalPlannerROS/footprint_model
rosparam get /move_base/TebLocalPlannerROS/max_vel_x
rosparam get /move_base/local_costmap/obstacle_layer/observation_sources
rosparam get /amcl
```

ROS 参数服务器中的值才是运行时真值。若和文件不同，检查 launch 加载顺序、命名空间、dynamic_reconfigure 和是否启动了旧节点。

### 6.2 确认 TF

```bash
rosrun tf tf_echo map odom_combined
rosrun tf tf_echo odom_combined base_footprint
rosrun tf tf_echo base_footprint laser_frame
rosrun tf tf_echo base_footprint laser_frame_2
rosrun tf tf_echo base_footprint laser_frame_3
```

检查平移、yaw、时间戳是否持续更新/固定正确，并关注 extrapolation 警告。不要用增大 `transform_tolerance` 长期掩盖 TF 断链或系统时间不同步。

### 6.3 确认话题频率和安全计数

```bash
rostopic hz /scan /scan_2 /scan_3
rostopic hz /is_passed /is_passed_2
rostopic echo /is_passed
rostopic echo /is_passed_2
rostopic hz /cmd_vel /smoother_cmd_vel
```

安全测试应架空驱动轮或使用可控低速场地。分别让 0、1、2、3 个以上稳定回波进入框，确认 `>2` 时只禁止正向线速度，并观察角速度是否仍执行。

### 6.4 RViz 必看图层

- `/map` 与主激光是否重合；
- global/local Costmap 的 obstacle 和 inflation；
- 两套 footprint 是否随机器人正确移动；
- TEB 局部轨迹是否穿过 polygon 障碍；
- 三台激光是否分别出现在正确高度和方向；
- 原地旋转时方形四角是否有足够空间。

## 7. 建议的实机场景矩阵

| 场景 | 速度 | 主要观测指标 |
|---|---:|---|
| 空场直行 5～10 m | 0.1→0.6 m/s | 里程漂移、AMCL 修正、指令跟踪 |
| 空场原地 ±180° | 低→正常 | 外接圆扫掠、角速度、地图重合 |
| 单侧贴墙 | 低速 | polygon 间距、路径是否抖动 |
| 已知宽度窄门 | 低速 | 全局是否出路、TEB 是否可行、最小实测间隙 |
| 前方纸箱突然进入 | 分级速度 | 感知延迟、Costmap 延迟、停车距离 |
| 细杆/黑物/玻璃 | 最低安全速度 | 三台雷达可探测性和点数阈值 |
| 雷达逐台断线 | 轮子架空 | Costmap、安全门、驱动默认行为 |
| AMCL 错误初始位姿 | 不驱动/低速 | 粒子收敛和恢复方式 |
| 机器人被搬动 | 不驱动起步 | recovery alpha 为 0 时的实际表现 |

## 8. 推荐调参流程

1. 固定机器人 polygon，不以“让路径通过”为理由缩小真实外形。
2. 先校准 TF、雷达方向/高度、轮径/轮距和里程计。
3. 建立基线 rosbag，记录 `/tf`、`/tf_static`、三路 scan、odom、amcl_pose、cmd_vel、smoother_cmd_vel、两路安全计数和 Costmap。
4. 按“定位 → Costmap → 全局规划 → TEB → 安全门”的顺序排查。
5. 每次只改一个参数或一组有明确公式关系的参数。
6. 每次修改记录：旧值、新值、假设、测试场景、量化结果和是否回退。
7. 最后在最大载荷、最低电量、低摩擦地面和最差可探测障碍上复测制动距离。

规划“能跑通”与“具备安全保证”是两个目标。Costmap、TEB 和当前激光门控都属于软件行为层，不能代替急停和满足目标风险等级的独立安全措施。
