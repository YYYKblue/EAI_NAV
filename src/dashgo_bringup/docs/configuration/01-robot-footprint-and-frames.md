# 机器人多边形、坐标系与安全边界

## 1. 多边形是本导航配置中的外形真值

用户已明确机器人的二维外形以 `dashgo_bringup` 中的多边形为准：

```text
                    +X（前）
                       ↑
       (-0.283,+0.283) │ (+0.283,+0.283)
                 ┌─────┼─────┐
       +Y（左）← │     ●     │ → -Y（右）
                 └─────┼─────┘
       (-0.283,-0.283) │ (+0.283,-0.283)
                       ↓
                    -X（后）
```

上图只是坐标关系示意，点 `●` 是 `base_footprint` 原点。ROS REP-103 的移动机器人惯例是：X 向前、Y 向左、Z 向上。

YAML 中点的顺序为：后右角、后左角、前左角、前右角。首点不需要在末尾重复，最后一个点会自动连接回第一个点。四点形成一个无自交闭合多边形。

## 2. 几何尺寸

多边形范围为：

```text
x ∈ [-0.283, +0.283] m
y ∈ [-0.283, +0.283] m
```

所以二维外形是以 `base_footprint` 为中心、边长 `0.566 m` 的正方形：

```text
长度 = 0.283 - (-0.283) = 0.566 m
宽度 = 0.283 - (-0.283) = 0.566 m
面积 = 0.566 × 0.566 = 0.320356 m²
内切半径 = 0.283 m
外接半径 = sqrt(0.283² + 0.283²) ≈ 0.4002 m
```

外接半径很重要：正方形原地旋转时，四个角会扫过半径约 `0.400 m` 的圆。即使正前方和侧方各有 `0.283 m` 空间，角点仍可能在旋转时碰撞。因此不加额外余量时，理论原地旋转空间直径约为 `0.800 m`。

## 3. 为什么同一外形必须写两次

### 3.1 Costmap 外形

文件：`config/navigation/costmap_common_params.yaml`

```yaml
footprint:
  [[-0.283, -0.283], [-0.283, 0.283], [0.283, 0.283], [0.283, -0.283]]
```

该文件分别加载到 `/move_base/global_costmap` 和 `/move_base/local_costmap`。Costmap 用它：

- 计算机器人的内切、外接半径；
- 清除机器人轮廓内部的实时障碍物；
- 为局部规划器提供 footprint；
- 在 footprint 碰撞检查时判断机器人占据区域。

### 3.2 TEB 外形

文件：`config/navigation/teb_local_planner_params.yaml`

```yaml
TebLocalPlannerROS:
  footprint_model:
    type: polygon
    vertices:
      [[-0.283, -0.283], [-0.283, 0.283], [0.283, 0.283], [0.283, -0.283]]
```

TEB 不会自动把 Costmap 的多边形复制成自己的优化模型。它使用 `vertices` 计算每个候选轨迹位姿上的机器人多边形到障碍物距离，并结合：

- `min_obstacle_dist: 0.15 m`；
- `inflation_dist: 0.30 m`；
- `weight_obstacle: 50`；
- `weight_inflation: 0.1`

形成避障优化代价。

如果两处不一致，会出现两类典型问题：

- Costmap 外形较小、TEB 外形较大：地图看似可通过，但 TEB 拒绝生成轨迹。
- Costmap 外形较大、TEB 外形较小：TEB 可能把轨迹贴得过近，后续 footprint 可行性检查失败或反复重规划。

## 4. 三种“距离”不能相加后简单理解

当前配置中有三组很容易混淆的参数：

| 机制 | 参数 | 当前值 | 距离从哪里量起 |
|---|---|---:|---|
| 实体几何 | polygon | 半边长 0.283 m | 从 `base_footprint` 原点到外形边 |
| Costmap 膨胀 | `inflation_radius` | 0.50 m | 从障碍栅格向外扩散代价值 |
| TEB 硬约束近似 | `min_obstacle_dist` | 0.15 m | 从 TEB 多边形外缘到障碍物 |
| TEB 软缓冲 | `inflation_dist` | 0.30 m | 从 TEB 多边形外缘到障碍物的软代价作用距离 |

`inflation_radius=0.50` 并不表示“机器人外形再向外扩 0.50 m”。Costmap 的膨胀层把障碍栅格周围一定范围赋予代价值，代价还与 footprint 推导出的内切半径有关；全局规划器主要在这个中心点代价场中找路。

TEB 的 `min_obstacle_dist` 则直接使用多边形边界。当前理想侧向通道宽度可粗略估算为：

```text
0.566 + 2 × 0.15 = 0.866 m
```

这表示两侧都满足 TEB 期望最小间距时约需 `0.866 m`，不是严格的“窄于就绝不通过”阈值，因为 TEB 通过带权惩罚近似约束，而且地图分辨率、障碍物离散方式和优化收敛都会影响结果。纯几何宽度下限是 `0.566 m`，实机绝不能按这个零余量极限运行。

## 5. Costmap 分辨率带来的离散误差

当前 `resolution: 0.05 m/cell`：

- 机器人边长约为 `0.566 / 0.05 = 11.32` 个栅格；
- 单边 `0.283 m` 约为 `5.66` 个栅格；
- TEB 最小间距 `0.15 m` 正好约 3 个栅格；
- Costmap 膨胀半径 `0.50 m` 约 10 个栅格。

障碍边缘和 footprint 最终都落到离散栅格上，因此实际可通行判定会有约一个栅格量级的变化。测量机器人外形时建议保留结构件突出、线缆摆动、定位误差和控制跟踪误差的余量，而不是只抄机械图纸的名义尺寸。

## 6. 与 URDF 的关系

`dashgo_description/urdf/dashgobase/base.urdf.xacro` 中的底盘碰撞包络已按实测最外边界改为 `0.566 m × 0.566 m`。由于 `base_link` 相对 `base_footprint` 向前偏置 `0.115 m`，其 visual、collision 和理想化质心均在 `base_link` 下使用 `x=-0.115 m` 的 origin，使三维包络中心准确落在 `base_footprint`。

因此按当前约定：

- 导航二维外形和 Gazebo X/Y 碰撞包络均为 `0.566 m × 0.566 m`；
- 两者都以 `base_footprint` 为几何中心；`base_link` 仍保留原机械参考点偏置；
- 修改实测外边界时，必须同步检查 URDF、Costmap、TEB 和仿真自体回波过滤盒。

## 7. 坐标系链路

已有地图导航时的主 TF 链为：

```text
map
└── odom_combined       AMCL 发布 map -> odom_combined
    └── base_footprint  robot_pose_ekf 发布里程计融合结果
        ├── laser_frame    /scan，安装高度约 0.28 m
        ├── laser_frame_2  /scan_2，安装高度约 0.05 m
        ├── laser_frame_3  /scan_3，安装高度约 0.35 m
        └── base_link      URDF 机器人实体模型
```

边建图边导航时，`map -> odom_combined` 改由 Gmapping 发布，其他链路不变。

各模块的固定坐标系：

| 模块 | 固定/参考坐标系 | 原因 |
|---|---|---|
| AMCL | `map` | 在静态地图中定位并校正里程计漂移 |
| 全局 Costmap | `map` | 路径要固定在地图坐标系中 |
| 局部 Costmap | `odom_combined` | 里程计连续，避免 AMCL 修正引起局部轨迹跳变 |
| TEB | `odom_combined` | 与局部 Costmap 一致 |
| footprint | `base_footprint` | 外形随机器人运动 |
| 安全框 | `base_footprint` | 安全区域固定在机器人周围 |

当前 frame ID 已统一为不带前导斜杠的 `map`、`odom_combined` 和 `base_footprint`。话题 `/map`、`/scan` 中的前导斜杠仍保留；frame ID 与 topic 名不要混淆。

## 8. 三台激光与高度窗口

三台激光在 `ydlidar` 包中配置为：

| 话题 | frame | 相对 `base_footprint` 的 Z | Costmap 高度窗口 |
|---|---|---:|---:|
| `/scan_2` | `laser_frame_2` | 约 0.05 m | `[0.01, 0.10] m` |
| `/scan` | `laser_frame` | 约 0.28 m | `[0.21, 0.30] m` |
| `/scan_3` | `laser_frame_3` | 约 0.35 m | `[0.31, 0.40] m` |

对水平二维 `LaserScan` 而言，点变换到 `base_footprint` 后的 Z 通常接近传感器安装高度。三个窗口正好把三层扫描分别纳入 VoxelLayer。若改动激光安装高度而不改高度窗口，整台激光的数据可能全部被过滤掉。

## 9. 安全框和多边形的空间关系

两个安全框的 XY 投影分别是：

```text
box_filter.yaml:   x=[0.10, 0.45], y=[-0.30, 0.30]
box_filter_2.yaml: x=[0.17, 0.45], y=[-0.29, 0.29]
robot polygon:     x=[-0.283,0.283], y=[-0.283,0.283]
```

可以确认：

- 两个框都只检查机器人前方 X 为正的区域；
- 两个框的前缘比机器人 polygon 前缘多出 `0.45 - 0.283 = 0.167 m`；
- 两个框都与机器人 polygon 的前半部分重叠，并不是从 polygon 前缘之外才开始；
- 第一个框比 polygon 每侧宽 `0.017 m`，第二个框每侧只宽 `0.007 m`；
- 第一个节点订阅 `/scan`，第二个订阅 `/scan_2`；`/scan_3` 没有安全框节点。

重叠本身不一定是错误，可能是为了适应雷达安装位置和遮挡，但如果雷达能看到机身、支架或线缆，框内点数会持续非零。必须用实机静止数据确认基线。

更关键的是，`laser_safety_gate.py` 当前只把框内有效激光点数量发布为 `Int16`：

```text
/is_passed    <- /scan
/is_passed_2  <- /scan_2
```

安全脚本本身只负责计数，但 `base.launch` 启动的 `dashgo_driver` 会订阅这两个话题。驱动代码的实际规则是：任一计数 **大于 2** 时，把后续正向线速度 `x > 0` 改成 0。它不会禁止倒车，也不会把角速度清零，所以原前进转弯命令会变成原地旋转。话题名 `is_passed` 也不是布尔值：`0～2` 被驱动视为允许前进，`3` 及以上被视为前方受阻。

这是一条软件辅助防碰链，不等于失效安全的硬件安全回路：驱动没有对计数话题做超时检查，安全节点 TF 失败时也不会发布新计数。具体风险与验证方法见 [04 文档](04-tuning-and-validation.md)。

## 10. 修改外形时的同步清单

如果以后更新 polygon，至少执行以下步骤：

1. 修改 `costmap_common_params.yaml/footprint`。
2. 用完全相同的点修改 `teb_local_planner_params.yaml/footprint_model/vertices`。
3. 重新计算最大角点半径，确认原地旋转空间。
4. 根据新前缘和侧缘检查两个安全框是否仍合理。
5. 重新检查窄通道宽度与 `min_obstacle_dist`。
6. 在 RViz 同时显示 `local_costmap/footprint`、局部代价地图和 TEB 轨迹。
7. 低速完成直行、贴墙、窄门、原地旋转和倒车测试，再恢复正常速度。
