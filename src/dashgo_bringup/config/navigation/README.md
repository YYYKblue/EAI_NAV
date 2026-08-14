# Dashgo navigation 参数说明

`navigation.launch` 启动 `map_server`、`amcl` 和 `move_base`，并按需启动激光安全过滤节点与 RViz。各 YAML 已在参数旁添加简短中文说明，本页用于说明文件职责和参数加载关系。

## 文件职责

- `amcl.yaml`：粒子滤波定位、里程计噪声和激光观测模型。
- `costmap_common_params.yaml`：全局、局部代价地图共同使用的机器人轮廓、分辨率和图层参数。
- `global_costmap_params.yaml`：全局代价地图独有的坐标系、频率和固定窗口设置。
- `local_costmap_params.yaml`：局部代价地图独有的坐标系、频率和滚动窗口尺寸。
- `global_planner_params.yaml`：`global_planner/GlobalPlanner` 的搜索与路径提取参数。
- `teb_local_planner_params.yaml`：TEB 轨迹、运动约束、避障和优化权重。
- `move_base_params.yaml`：规划器选择、线程频率、振荡检测和恢复行为。

## Costmap 参数加载关系

公共配置会分别加载到下面两个独立命名空间：

```text
costmap_common_params.yaml
├── /move_base/global_costmap
└── /move_base/local_costmap
```

因此公共文件加载两次是必要的，并不是重复覆盖。随后加载的 `global_costmap_params.yaml` 和 `local_costmap_params.yaml` 只补充两套地图之间的差异。

当前全局和局部代价地图都显式加载三层：

```text
StaticLayer -> VoxelLayer -> InflationLayer
```

所以局部滚动地图也会使用 `/map` 中的静态障碍物。若希望局部地图只依赖实时激光，需要把插件列表拆回全局、局部文件，并从局部列表中删除 `static_layer`。

## 需要同步维护的参数

Costmap 和 TEB 是两个独立模块，机器人轮廓不能合并成一个 ROS 参数：

- `costmap_common_params.yaml/footprint`
- `teb_local_planner_params.yaml/TebLocalPlannerROS/footprint_model/vertices`

修改机器人外形时必须同步修改两处，否则代价地图碰撞判断和 TEB 轨迹优化会使用不同的机器人尺寸。

## 调参注意点

- `inflation_radius: 0.50` 是全局、局部 costmap 的软代价影响范围，不是全部不可通行的硬障碍半径。
- `min_obstacle_dist: 0.15` 是从机器人 footprint 外缘到障碍物的期望间距，不是从机器人中心计算。
- `inflation_dist: 0.30` 是 TEB 在最小间距之外施加较弱避障代价的软缓冲范围。
- `allow_unknown: false` 控制全局规划器能否穿过未知区域；`track_unknown_space: true` 控制代价地图是否保存未知区域，两者作用不同。
- `transform_tolerance` 在 AMCL 和 costmap 中分别服务不同节点，虽然数值相同，也不能合并为同一个参数。
