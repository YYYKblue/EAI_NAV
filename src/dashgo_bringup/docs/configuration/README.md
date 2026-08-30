# Dashgo Bringup YAML 配置总览

本文档集说明 `dashgo_bringup` 包内全部 YAML。重点是当前机器人采用的多边形外形，以及该外形如何进入 ROS1 导航、建图、定位和激光安全检测链路。

## 1. 文档范围

包内共有 11 个 YAML 文件：

| 分类 | 文件 | 参数使用者 |
|---|---|---|
| 定位 | `config/navigation/amcl.yaml` | `amcl` |
| 代价地图公共配置 | `config/navigation/costmap_common_params.yaml` | `move_base/global_costmap`、`move_base/local_costmap` |
| 全局代价地图 | `config/navigation/global_costmap_params.yaml` | `move_base/global_costmap` |
| 局部代价地图 | `config/navigation/local_costmap_params.yaml` | `move_base/local_costmap` |
| 全局规划 | `config/navigation/global_planner_params.yaml` | `global_planner/GlobalPlanner` |
| 局部规划 | `config/navigation/teb_local_planner_params.yaml` | `teb_local_planner/TebLocalPlannerROS` |
| 导航调度与恢复 | `config/navigation/move_base_params.yaml` | `move_base` 和恢复插件 |
| 建图 | `config/mapping/gmapping.yaml` | `slam_gmapping` |
| 激光安全框 1 | `config/safety/box_filter.yaml` | `/laser_safety_gate` |
| 激光安全框 2 | `config/safety/box_filter_2.yaml` | `/laser_safety_gate_2` |
| 地图元数据 | `maps/bj050101.yaml` | `map_server` |

逐参数说明分为以下文档：

- [机器人多边形、坐标系与安全边界](01-robot-footprint-and-frames.md)
- [导航 YAML 逐参数说明](02-navigation-yaml-reference.md)
- [建图、安全框和地图 YAML 逐参数说明](03-mapping-safety-map-yaml-reference.md)
- [配置联动、风险与调参验证方法](04-tuning-and-validation.md)

## 2. 参数加载关系

### 2.1 已有地图定位导航

```text
navigation.launch
├── base.launch
│   ├── dashgo_driver：消费安全计数，计数 > 2 时门控正向线速度
│   ├── 三台激光雷达
│   ├── robot_pose_ekf：发布 odom_combined -> base_footprint
│   └── velocity_smoother
├── map_server：读取 maps/bj050101.yaml，发布 /map
├── amcl
│   ├── 读取 config/navigation/amcl.yaml
│   └── 发布 map -> odom_combined
└── move_base.launch
    ├── move_base：读取 6 个 costmap/planner YAML
    ├── laser_safety_gate：读取 box_filter.yaml，订阅 /scan
    └── laser_safety_gate_2：读取 box_filter_2.yaml，订阅 /scan_2
```

启动命令：

```bash
roslaunch dashgo_bringup navigation.launch rviz:=true
```

### 2.2 Gmapping 建图

```text
mapping.launch
├── base.launch
└── slam_gmapping
    ├── 读取 config/mapping/gmapping.yaml
    ├── 默认订阅 /scan
    └── 发布 /map 和 map -> odom_combined
```

启动命令：

```bash
roslaunch dashgo_bringup mapping.launch rviz:=true
```

### 2.3 边建图边导航

`mapping_navigation.launch` 同时启动 `mapping.launch` 和 `move_base.launch`，但不启动 `map_server`、`amcl`。地图和 `map -> odom_combined` 均由 Gmapping 提供。

```bash
roslaunch dashgo_bringup mapping_navigation.launch rviz:=true
```

当前 `GlobalPlanner/allow_unknown: false`，因此只能规划到 Gmapping 已经观测为自由空间的区域，不能直接把目标设到尚未探索的未知区后让机器人自动探索。

## 3. Costmap 公共文件为什么加载两次

`move_base.launch` 明确把同一个公共文件分别装入两个命名空间：

```text
costmap_common_params.yaml
├── /move_base/global_costmap/*
└── /move_base/local_costmap/*
```

因此两次加载不是重复操作。最终有效参数可以理解为：

```text
global_costmap = costmap_common_params + global_costmap_params
local_costmap  = costmap_common_params + local_costmap_params
```

公共文件中的 `plugins` 也会复制到两套地图，所以当前全局和局部代价地图都由以下三层组成：

```text
StaticLayer -> VoxelLayer -> InflationLayer
```

这意味着局部滚动地图也包含 `/map` 的静态墙体，而不只是三台激光的实时障碍物。

## 4. 当前机器人外形的核心结论

导航采用以下方形多边形，单位为米，坐标相对于 `base_footprint`：

```yaml
[[-0.283, -0.283],
 [-0.283,  0.283],
 [ 0.283,  0.283],
 [ 0.283, -0.283]]
```

由此得到：

| 派生量 | 数值 | 含义 |
|---|---:|---|
| X 方向总长 | 0.566 m | 后缘 `-0.283` 到前缘 `+0.283` |
| Y 方向总宽 | 0.566 m | 右缘 `-0.283` 到左缘 `+0.283` |
| 内切半径 | 0.283 m | 中心到任一边的距离 |
| 外接半径 | 约 0.400 m | 中心到角点的距离，也是原地转动扫过圆的半径 |
| 原地转动扫掠直径 | 约 0.800 m | 不计任何额外安全余量 |

同一多边形在两处分别配置：

1. `costmap_common_params.yaml/footprint`：用于两套代价地图的机器人轮廓和碰撞相关处理。
2. `teb_local_planner_params.yaml/TebLocalPlannerROS/footprint_model/vertices`：用于 TEB 轨迹优化时计算多边形到障碍物的距离。

两处是不同节点读取的不同 ROS 参数，必须保持一致。详细解释见[机器人多边形、坐标系与安全边界](01-robot-footprint-and-frames.md)。

## 5. 阅读参数表的方法

后续参数表使用以下约定：

- “当前效果”只说明此仓库代码和当前数值可以确认的行为。
- “增大/减小”说明主要趋势，不代表任何场景都单调成立；规划器参数经常相互耦合。
- “默认值”未写入 YAML 的参数仍可能由节点自身默认值生效。本文逐项解释的是 YAML 中显式出现的参数，不等同于节点全部可配置参数。
- 角度统一注明 `rad`；速度、加速度和距离使用 SI 单位。
- 配置依据是本工作区代码，以及本机 ROS Noetic 中安装的 `navigation 1.17.3`、`gmapping 1.4.2`、`teb_local_planner 0.9.1` 接口。若迁移到其他发行版或插件版本，需要重新核对参数接口。

## 6. 最重要的四个联动关系

1. **外形联动**：Costmap `footprint` 必须等于 TEB `footprint_model/vertices`。
2. **速度联动**：TEB 的速度、加速度上限不能高于底盘驱动和 `velocity_smoother` 的真实能力，否则规划速度无法被底盘正确执行。
3. **时间联动**：`controller_frequency`、TEB `dt_ref`、局部地图 `update_frequency` 和激光实际频率共同决定动态响应速度。
4. **障碍距离联动**：Costmap `inflation_radius` 与 TEB `min_obstacle_dist`、`inflation_dist` 是两套不同代价机制，不能把三个值当成同一个“安全距离”。
