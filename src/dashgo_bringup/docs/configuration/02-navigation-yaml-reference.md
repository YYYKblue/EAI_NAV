# 导航 YAML 逐参数说明

本文覆盖 `config/navigation` 下 7 个 YAML 文件中显式出现的每一个参数。参数路径按它们加载到 `move_base` 后的逻辑路径书写。

## 1. `amcl.yaml`

### 1.1 文件作用和加载位置

`navigation.launch` 把该文件加载到 `/amcl` 节点私有参数空间。AMCL 使用静态地图、`/scan` 和里程计运动增量估计机器人在 `map` 中的位姿，并发布 `map -> odom_combined`。

launch 另外设置了三个不在 YAML 中的 frame 参数：

```text
odom_frame_id   = odom_combined
base_frame_id   = base_footprint
global_frame_id = map
```

### 1.2 地图和里程计模型

| 参数 | 当前值 | 类型/单位 | 详细含义与当前效果 |
|---|---:|---|---|
| `use_map_topic` | `true` | bool | 订阅 `/map` 获取地图，而不是只通过静态地图服务获取。适合当前 `map_server` 的锁存话题，也方便地图重新发布。它不表示 AMCL 会自己建图。 |
| `odom_model_type` | `diff` | string | 使用差速运动模型，允许前后移动与绕 Z 轴转动，不建模横向平移。与 TEB 的 `min_turning_radius: 0` 和高非完整约束相匹配。可选值还包括 `omni`、`diff-corrected`、`omni-corrected`；不要仅为“改善漂移”盲目换模型。 |
| `odom_alpha1` | `0.2` | 非负系数 | 旋转运动对“旋转估计噪声”的贡献。增大会在转弯后把粒子航向扩得更散。 |
| `odom_alpha2` | `0.2` | 非负系数 | 平移运动对“旋转估计噪声”的贡献。直行时航向漂移明显可考虑增大。 |
| `odom_alpha3` | `0.8` | 非负系数 | 平移运动对“平移估计噪声”的贡献。当前明显大于其他 alpha，表示 AMCL 对直行里程距离较不信任，会依赖激光把位置拉回地图。粒子会更分散，计算和观测匹配压力也更大。 |
| `odom_alpha4` | `0.2` | 非负系数 | 旋转运动对“平移估计噪声”的贡献。原地转动时底盘打滑造成位置偏移，可通过实测估计此项。 |

四个 alpha 不是米或弧度的直接标准差，而是 AMCL 运动模型中随本次运动量缩放噪声的系数。应根据轮速里程计与外部真值的残差标定；数值越大表示越不信任里程计，不是“定位一定越准”。

### 1.3 KLD 自适应粒子滤波

| 参数 | 当前值 | 类型/单位 | 详细含义与当前效果 |
|---|---:|---|---|
| `min_particles` | `2000` | int，粒子数 | 自适应采样的下限。即使定位已经集中，仍至少保留 2000 个粒子，鲁棒性较高但 CPU 开销较大。 |
| `max_particles` | `5000` | int，粒子数 | 自适应采样的上限。全局重定位或分布很散时最多 5000 个。必须不小于 `min_particles`。 |
| `kld_err` | `0.05` | 正数 | KLD 采样允许的分布估计误差。越小通常需要越多粒子，越接近上限。它不是位置误差 0.05 m。 |
| `kld_z` | `0.99` | 正数 | KLD 公式使用的标准正态上分位参数。这里的 `0.99` 是传入算法的数值，不能简单解释成“定位置信度 99%”。增大会倾向要求更多样本。 |
| `resample_interval` | `2` | int，滤波更新次数 | 每 2 次滤波更新才重采样一次。这里的“更新”受 `update_min_d/a` 触发，不是每收到两帧激光。值大可减少重采样开销和粒子贫化，但响应变慢。 |

当前 2000～5000 粒子和每帧最多 60 束激光的组合偏重。是否需要这么多粒子，应以 `/amcl/particlecloud` 收敛情况、CPU 占用、激光环境重复性和绑架恢复需求判断。

### 1.4 激光观测模型

| 参数 | 当前值 | 类型/单位 | 详细含义与当前效果 |
|---|---:|---|---|
| `laser_model_type` | `likelihood_field` | string | 概率场模型：比较激光端点到地图最近障碍物的距离。通常比逐束射线的 `beam` 模型快且对少量动态障碍更稳健。 |
| `laser_max_beams` | `60` | int，束/帧 | 从一帧扫描中均匀抽取最多 60 束参与权重更新，不是只取前 60 束。增大可利用更多几何信息，同时近似按粒子数成比例增加计算量。 |
| `laser_max_range` | `6.0` | m | AMCL 参与定位计算的最大量程。激光 launch 的上报最大量程为 16 m，但 AMCL 只用到 6 m。设为 `-1` 才表示采用 LaserScan 自带的最大量程。 |
| `laser_z_hit` | `0.5` | 混合权重 | 激光端点与地图障碍物吻合的命中模型权重。 |
| `laser_z_rand` | `0.5` | 混合权重 | 无法由地图解释的随机测量权重。当前两项各占一半，对动态物体和异常回波较宽容，但地图匹配的区分度相对降低。当前 `likelihood_field` 主要使用这两项，二者通常应归一化或至少保持相对比例明确。 |
| `laser_sigma_hit` | `0.2` | m | 命中模型高斯宽度。地图墙线、激光噪声或位姿偏差在约此尺度内仍能取得较好似然。增大更宽容但定位峰值会变钝；减小更精确但更容易因地图误差失配。 |
| `laser_likelihood_max_dist` | `2.0` | m | 地图概率场中“到最近障碍物距离”的截断范围。超过该距离后命中似然不再继续按距离细分。它不是激光最大量程，也不是导航膨胀半径。 |

### 1.5 更新、TF 与恢复

| 参数 | 当前值 | 类型/单位 | 详细含义与当前效果 |
|---|---:|---|---|
| `update_min_d` | `0.25` | m | 自上次滤波更新后至少平移 0.25 m 才以新激光更新粒子；旋转条件可独立触发。减小可提高低速定位刷新率，也会增加 CPU。 |
| `update_min_a` | `0.2` | rad，约 11.46° | 自上次滤波更新后至少旋转该角度才更新。当前 TEB 最大角速度 0.30 rad/s，连续旋转时理论约 0.67 s 才由角度阈值触发一次。 |
| `gui_publish_rate` | `10.0` | Hz | AMCL 可视化相关输出的最大发布频率。它不等于滤波更新频率；实际更新仍受运动阈值和激光数据限制。设为负值可关闭相关发布。 |
| `transform_tolerance` | `1.0` | s | AMCL 发布 `map -> odom_combined` 时将变换时间戳向未来延展的容忍量，用于吸收处理和通信延迟。过大可能掩盖时钟或延迟问题，过小会导致消费者报 TF extrapolation。 |
| `recovery_alpha_slow` | `0.0` | [0,1] 系数 | 粒子平均权重的慢速指数滤波系数，用于检测定位质量突降并注入随机粒子。0 表示关闭这一路自适应恢复。 |
| `recovery_alpha_fast` | `0.0` | [0,1] 系数 | 快速平均权重滤波系数。两个 recovery alpha 都为 0 时，随机粒子注入恢复被关闭。机器人“被搬走”后主要依靠手动 `/initialpose` 或现有粒子偶然扩散恢复。通常 fast 应明显大于 slow。 |

## 2. `costmap_common_params.yaml`

### 2.1 加载作用域

该文件分别加载到：

```text
/move_base/global_costmap
/move_base/local_costmap
```

因此下述所有参数在两套 Costmap 中各有一份，包括三台激光的订阅和体素地图发布。

### 2.2 机器人模型和基础网格

| 参数 | 当前值 | 类型/单位 | 详细含义与当前效果 |
|---|---:|---|---|
| `robot_base_frame` | `/base_footprint` | frame ID | Costmap 查询机器人位姿、解释 footprint 的机器人坐标系。必须能从各自 `global_frame` 通过 TF 到达。建议后续统一去掉前导 `/`。 |
| `resolution` | `0.05` | m/cell | 两套代价地图每个栅格边长 5 cm。值更小轮廓精细但内存和更新计算按面积快速增加；值更大会损失窄通道和小障碍细节。静态地图本身也是 0.05 m/cell。 |
| `transform_tolerance` | `1.0` | s | Costmap 可容忍的 TF 数据延迟。它和 AMCL 的同名参数属于不同节点，不能互相替代。持续需要 1 s 才能工作通常说明时间戳或处理延迟需要排查。 |
| `footprint` | 四点 polygon | m | 相对 `base_footprint` 的二维闭合外轮廓，当前为 `0.566 m × 0.566 m` 正方形。详见 [01 文档](01-robot-footprint-and-frames.md)。配置 footprint 后不再使用圆形 `robot_radius`。 |

### 2.3 `plugins` 列表

`plugins` 是有顺序的插件实例列表，每一项都有 `name` 和 `type`：

| 列表项参数 | 当前值 | 含义 |
|---|---|---|
| `plugins[0].name` | `static_layer` | 插件实例名，也是后面 `static_layer:` 参数块的命名空间。 |
| `plugins[0].type` | `costmap_2d::StaticLayer` | 读取二维静态占据地图。 |
| `plugins[1].name` | `obstacle_layer` | 实时障碍物层实例名。 |
| `plugins[1].type` | `costmap_2d::VoxelLayer` | 使用垂直体素列融合三层 LaserScan，再投影到二维 Costmap。 |
| `plugins[2].name` | `inflation_layer` | 膨胀层实例名。 |
| `plugins[2].type` | `costmap_2d::InflationLayer` | 在已融合的障碍物周围生成递减代价。 |

执行次序是静态层、体素障碍层、膨胀层。膨胀层必须位于障碍来源之后，才能对最终障碍做膨胀。

### 2.4 `static_layer`

| 参数 | 当前值 | 类型/单位 | 详细含义与当前效果 |
|---|---:|---|---|
| `static_layer/enabled` | `true` | bool | 两套 Costmap 都启用静态地图层。局部地图也会把滚动窗口覆盖范围内的静态墙体复制进来。 |
| `static_layer/map_topic` | `/map` | topic | 静态 OccupancyGrid 话题。纯导航由 `map_server` 发布，边建图边导航由 Gmapping 发布。绝对话题名避免被 `/move_base/...` 命名空间重写。 |

### 2.5 `obstacle_layer` 公共参数

| 参数 | 当前值 | 类型/单位 | 详细含义与当前效果 |
|---|---:|---|---|
| `obstacle_layer/enabled` | `true` | bool | 启用 VoxelLayer。关闭后，三台激光都不会进入 Costmap，但静态层仍在。 |
| `obstacle_layer/max_obstacle_height` | `1.2` | m | 该层接受障碍点的全局最高 Z。每个传感器还受自己的 `max_obstacle_height` 约束，实际采用更严格的范围。当前体素网格只覆盖约 0～1.0 m。 |
| `obstacle_layer/obstacle_range` | `2.0` | m | 从传感器原点起，最多用 2 m 内的有效端点“标记”障碍。超过 2 m 的返回不会新增障碍。该值可被各传感器源的同名参数覆盖；当前三个源均继承公共值。 |
| `obstacle_layer/raytrace_range` | `5.0` | m | 从传感器原点沿激光射线最多清除 5 m 自由空间。当前大于标记距离，可用远处或无障碍射线清掉近处旧障碍。不能超过传感器可靠量程随意增大。 |
| `obstacle_layer/combination_method` | `1` | enum int | VoxelLayer 与主地图叠加时取最大代价值。`0` 为覆盖，`1` 为最大值，`99` 为不更新主地图。取最大值可避免实时自由观测覆盖静态墙体。 |
| `obstacle_layer/observation_sources` | 三个源名组成的字符串 | string | 声明要实例化的观测源：`laser_scan_sensor laser_scan_sensor_2 laser_scan_sensor_3`。YAML 的 `>-` 只是把多行折叠成空格分隔字符串，不是数组。名称必须与下方配置块完全一致。 |
| `obstacle_layer/track_unknown_space` | `true` | bool | 让 VoxelLayer 区分未知和已通过射线确认的自由空间。它控制实时体素层的空间状态，不等于 GlobalPlanner 是否允许穿越未知区。 |
| `obstacle_layer/footprint_clearing_enabled` | `true` | bool | 每次更新将机器人当前 footprint 内的实时障碍物清除，减少自身结构或短暂噪声把机器人“困死”。它只清理机器人当前轮廓内，不会清掉前方安全距离内的障碍。 |

`obstacle_range` 和 `raytrace_range` 都以各激光的传感器原点为起点，不是从机器人 polygon 外缘起算。

### 2.6 垂直体素网格

| 参数 | 当前值 | 类型/单位 | 详细含义与当前效果 |
|---|---:|---|---|
| `obstacle_layer/origin_z` | `0.0` | m | 体素网格的最低 Z，相对 Costmap 使用的全局坐标表达，但点会经 TF 正确转换。 |
| `obstacle_layer/z_resolution` | `0.1` | m/voxel | 垂直单层高度 10 cm。 |
| `obstacle_layer/z_voxels` | `10` | int | 每个 XY 栅格柱有 10 个有效 Z 体素，因此有效高度跨度约为 `0.0～1.0 m`。底层实现最多支持 16 层。 |
| `obstacle_layer/unknown_threshold` | `15` | int | VoxelLayer 判定一列是否仍属未知时使用的未知体素阈值。其实现包含对固定 16 位体素存储和实际 `z_voxels` 的补偿，不能简单理解成“10 层里要有 15 层未知”。值越大越倾向把列视为已知。 |
| `obstacle_layer/mark_threshold` | `0` | int | 一列允许保持非障碍的已标记体素上限；超过阈值就投影为二维障碍。0 表示只要有至少一个合格体素被标记，该 XY 列即可成为障碍，灵敏但也更容易受单点噪声影响。 |
| `obstacle_layer/publish_voxel_map` | `true` | bool | 发布三维体素调试消息。由于公共文件加载两次，全局和局部层都会发布各自体素图，方便 RViz 调试但增加 CPU、内存和通信负担。 |

### 2.7 `/scan_2` 低位激光源

| 参数 | 当前值 | 类型/单位 | 详细含义与当前效果 |
|---|---:|---|---|
| `obstacle_layer/laser_scan_sensor_2/data_type` | `LaserScan` | string | 输入类型为 `sensor_msgs/LaserScan`。拼写和大小写必须正确。 |
| `.../topic` | `/scan_2` | topic | 低位激光话题，对应 `laser_frame_2`，安装 Z 约 0.05 m。 |
| `.../marking` | `true` | bool | 有效回波端点用于标记障碍。 |
| `.../clearing` | `true` | bool | 端点之前的射线路径用于清除自由空间。 |
| `.../expected_update_rate` | `0.0` | s | 期望消息间隔；名称里虽有 rate，ROS Costmap 这里使用的是秒。0 关闭“传感器长时间未更新”检查，所以雷达掉线不会仅由此参数触发过期警告。 |
| `.../min_obstacle_height` | `0.01` | m | 只接收变换到目标坐标系后 Z 不低于 0.01 m 的点。 |
| `.../max_obstacle_height` | `0.10` | m | 只接收 Z 不高于 0.10 m 的点。0.05 m 的扫描平面落在窗口内。 |

### 2.8 `/scan` 中位激光源

| 参数 | 当前值 | 类型/单位 | 详细含义与当前效果 |
|---|---:|---|---|
| `obstacle_layer/laser_scan_sensor/data_type` | `LaserScan` | string | 输入为二维激光扫描。 |
| `.../topic` | `/scan` | topic | 主激光话题，对应 `laser_frame`，安装 Z 约 0.28 m；AMCL 和 Gmapping 默认也只使用这一台。 |
| `.../marking` | `true` | bool | 用回波标记障碍。 |
| `.../clearing` | `true` | bool | 用射线清除自由空间。 |
| `.../expected_update_rate` | `0.0` | s | 关闭更新超时检查。实际雷达 launch 频率目标为约 7 Hz。 |
| `.../min_obstacle_height` | `0.21` | m | 接受点的最低 Z。 |
| `.../max_obstacle_height` | `0.30` | m | 接受点的最高 Z；0.28 m 扫描平面位于区间内且距离上界仅 0.02 m，安装误差或 TF 倾斜需关注。 |

### 2.9 `/scan_3` 高位激光源

| 参数 | 当前值 | 类型/单位 | 详细含义与当前效果 |
|---|---:|---|---|
| `obstacle_layer/laser_scan_sensor_3/data_type` | `LaserScan` | string | 输入为二维激光扫描。 |
| `.../topic` | `/scan_3` | topic | 高位激光话题，对应 `laser_frame_3`，安装 Z 约 0.35 m。 |
| `.../marking` | `true` | bool | 用回波标记障碍。 |
| `.../clearing` | `true` | bool | 用射线清除自由空间。 |
| `.../expected_update_rate` | `0.0` | s | 关闭更新超时检查。 |
| `.../min_obstacle_height` | `0.31` | m | 接受点的最低 Z。 |
| `.../max_obstacle_height` | `0.40` | m | 接受点的最高 Z；0.35 m 扫描面位于窗口中央附近。 |

### 2.10 `inflation_layer`

| 参数 | 当前值 | 类型/单位 | 详细含义与当前效果 |
|---|---:|---|---|
| `inflation_layer/enabled` | `true` | bool | 启用障碍物周围的递减代价场。关闭后全局路径可能紧贴墙面，局部规划余量也变小。 |
| `inflation_layer/inflation_radius` | `0.50` | m | 从障碍栅格向外计算非零膨胀代价的最大半径。它不是从机器人 polygon 外缘再加 0.50 m，也不意味着整个 0.50 m 都是致命障碍。 |
| `inflation_layer/cost_scaling_factor` | `3.0` | 1/m 尺度系数 | 控制内切半径之外代价随距离指数衰减的速度。**值越大衰减越快、路径越可能靠墙；值越小衰减越慢、路径越倾向通道中央。**该趋势容易被误解。 |

## 3. `global_costmap_params.yaml`

文件顶层 `global_costmap:` 是参数命名空间容器，不是一个独立布尔参数。

| 参数 | 当前值 | 类型/单位 | 详细含义与当前效果 |
|---|---:|---|---|
| `global_costmap/global_frame` | `/map` | frame ID | 全局代价地图固定坐标系。AMCL 或 Gmapping 必须提供从 `map` 到机器人链路的 TF。 |
| `global_costmap/update_frequency` | `1.5` | Hz | 图层融合和地图内部更新每秒约 1.5 次，即周期约 0.667 s。动态障碍进入全局重规划的速度受此限制。 |
| `global_costmap/publish_frequency` | `1.0` | Hz | 对外发布完整/显示用 Costmap 的频率。发布频率低于更新频率不会阻止内部规划使用最新地图。 |
| `global_costmap/rolling_window` | `false` | bool | 使用固定地图范围，通常从 `/map` 获得尺寸和原点；不会始终以机器人为中心滚动。 |

## 4. `local_costmap_params.yaml`

顶层 `local_costmap:` 是命名空间容器。

| 参数 | 当前值 | 类型/单位 | 详细含义与当前效果 |
|---|---:|---|---|
| `local_costmap/global_frame` | `/odom_combined` | frame ID | 局部地图的固定坐标系。里程计连续，适合短时间轨迹控制；AMCL 校正由更上层 `map -> odom_combined` 吸收。必须与 TEB `map_frame` 保持一致。 |
| `local_costmap/update_frequency` | `3.0` | Hz | 局部障碍地图每约 0.333 s 更新一次。当前略低于 4 Hz 控制循环，部分控制周期会复用上一版地图。 |
| `local_costmap/publish_frequency` | `1.0` | Hz | RViz/话题发布频率，不代表内部只每秒更新一次。 |
| `local_costmap/rolling_window` | `true` | bool | 局部窗口随机器人移动，避免建立无限大的高频局部地图。 |
| `local_costmap/width` | `3.0` | m | 滚动窗口 X 尺寸，约 60 个栅格。机器人位于中心附近时，前后各约 1.5 m。 |
| `local_costmap/height` | `3.0` | m | 滚动窗口 Y 尺寸，同样约 60 个栅格。二维主栅格约 3600 cells，各图层和体素存储另计。 |

## 5. `global_planner_params.yaml`

顶层 `GlobalPlanner:` 必须与 `base_global_planner: global_planner/GlobalPlanner` 的插件实例名对应。下面的参数属于该插件私有空间。

| 参数 | 当前值 | 类型/单位 | 详细含义与当前效果 |
|---|---:|---|---|
| `GlobalPlanner/old_navfn_behavior` | `false` | bool | 使用 GlobalPlanner 的修正行为，而不是尽量复刻旧 `navfn` 的坐标和边界处理。没有兼容旧路径的明确需求时保持 false。 |
| `GlobalPlanner/use_quadratic` | `true` | bool | 使用二次近似计算势场，而不是简单线性势值，通常使势场梯度和提取路径更平滑。 |
| `GlobalPlanner/use_dijkstra` | `true` | bool | 使用 Dijkstra 扩展整个相关代价区域；false 则使用带启发式的 A*。Dijkstra 行为稳定但大地图上可能扩展更多节点。 |
| `GlobalPlanner/use_grid_path` | `false` | bool | false 使用梯度下降从势场提取路径；true 使用沿栅格单元的路径提取。当前路径通常不局限于明显的栅格折线。 |
| `GlobalPlanner/allow_unknown` | `false` | bool | 不允许全局路径经过未知栅格。纯导航中较保守；边建图边导航时无法把路径规划进未探索区域。它和 VoxelLayer 的 `track_unknown_space` 是不同层次的开关。 |
| `GlobalPlanner/planner_window_x` | `0.0` | m | 局部规划窗口 X 尺寸。0 表示禁用窗口限制，使用完整全局 Costmap。 |
| `GlobalPlanner/planner_window_y` | `0.0` | m | 局部规划窗口 Y 尺寸。0 同样表示使用完整地图。仅设置一个非零窗口通常没有意义。 |
| `GlobalPlanner/default_tolerance` | `0.0` | m | 目标点本身不可达时，可在目标附近寻找替代终点的最大半径。0 表示目标栅格必须可达，不自动接受邻近位置。注意这和 TEB 到达目标的 `xy_goal_tolerance` 不同。 |

## 6. `move_base_params.yaml`

### 6.1 规划器插件

| 参数 | 当前值 | 类型 | 详细含义与当前效果 |
|---|---|---|---|
| `base_global_planner` | `global_planner/GlobalPlanner` | plugin class | 选择全局路径插件。对应参数块名为 `GlobalPlanner`。 |
| `base_local_planner` | `teb_local_planner/TebLocalPlannerROS` | plugin class | 选择 TEB 作为局部轨迹规划器和速度控制器。对应参数块名为 `TebLocalPlannerROS`。 |

### 6.2 全局规划和局部控制线程

| 参数 | 当前值 | 类型/单位 | 详细含义与当前效果 |
|---|---:|---|---|
| `planner_frequency` | `1.0` | Hz | 有活动目标时每秒周期性重算全局路径。0 表示只在新目标、路径阻塞等事件发生时按需规划。 |
| `planner_patience` | `3.0` | s | 全局规划持续无法给出有效路径的容忍时间，之后进入恢复流程。 |
| `max_planning_retries` | `1` | int | 进入恢复前允许的规划重试次数，`-1` 为无限重试。该限制会与 `planner_patience` 共同作用；以 1 Hz 规划时，一次失败就可能很快到达重试限制，而不一定完整等待 3 s。 |
| `controller_frequency` | `4.0` | Hz | 每 0.25 s 调用一次 TEB 并尝试发布速度命令。真实输出还经过 velocity smoother，底盘响应也受驱动频率限制。 |
| `controller_patience` | `3.0` | s | TEB 连续无法产生有效控制约 3 s 后进入恢复流程；按 4 Hz 粗略相当于 12 个控制周期。 |
| `shutdown_costmaps` | `false` | bool | 没有活动导航目标时 Costmap 仍保持订阅和更新。再次导航启动快，但持续占用资源。 |

### 6.3 振荡与恢复

| 参数 | 当前值 | 类型/单位 | 详细含义与当前效果 |
|---|---:|---|---|
| `oscillation_timeout` | `5.0` | s | 机器人在振荡重置半径内停留超过 5 s，就判定缺乏有效进展并触发恢复。0 可关闭超时振荡检测。 |
| `oscillation_distance` | `0.2` | m | 相对上次振荡重置位置移动超过 0.2 m，才算取得足够进展并重置计时。小幅前后抖动不一定能重置。 |
| `recovery_behavior_enabled` | `true` | bool | 允许执行下方显式列出的恢复插件。false 时规划/控制失败会直接导致目标失败，不运行清图或旋转。 |

`recovery_behaviors` 是有顺序的插件列表：

| 列表项参数 | 当前值 | 详细含义 |
|---|---|---|
| `recovery_behaviors[0].name` | `aggressive_reset` | 第一个恢复行为的实例名，也是参数块命名空间。 |
| `recovery_behaviors[0].type` | `clear_costmap_recovery/ClearCostmapRecovery` | 清除所选 Costmap 图层在保留窗口外的数据。 |
| `recovery_behaviors[1].name` | `clearing_rotation` | 第二个恢复行为实例名。 |
| `recovery_behaviors[1].type` | `rotate_recovery/RotateRecovery` | 尝试原地旋转，让激光从多方向观察并清除旧障碍。原地旋转仍会做 footprint 可行性检查。 |

自定义清图参数：

| 参数 | 当前值 | 类型/单位 | 详细含义与当前效果 |
|---|---:|---|---|
| `aggressive_reset/reset_distance` | `1.3` | m | 以机器人为中心保留边长约 1.3 m 的方形区域，清除该区域外所选层的数据。它不是半径 1.3 m。当前局部窗口为 3 m，因此会清掉局部窗口较外侧的大量实时历史障碍。 |
| `aggressive_reset/layer_names` | `[obstacle_layer]` | string list | 只清除实例名为 `obstacle_layer` 的实时 VoxelLayer，不清 `static_layer`，因此不会删除地图墙体。名称必须与 `plugins[].name` 一致。 |

## 7. `teb_local_planner_params.yaml`

顶层 `TebLocalPlannerROS:` 是 TEB 插件实例命名空间。TEB 把一条局部轨迹表示为一系列二维位姿和相邻位姿间时间差，并同时优化几何、速度、加速度、避障和时间代价。

### 7.1 里程计与参考坐标系

| 参数 | 当前值 | 类型 | 详细含义与当前效果 |
|---|---|---|---|
| `TebLocalPlannerROS/odom_topic` | `odom` | topic | TEB 获取当前机器人速度的里程计话题。相对话题名在当前启动结构中预期解析到 `/odom`；可用 `rosnode info /move_base` 核对实际 resolved topic。Costmap 位姿则来自 `odom_combined` TF，两者角色不同。 |
| `TebLocalPlannerROS/map_frame` | `/odom_combined` | frame ID | TEB 内部轨迹、全局计划和障碍物变换使用的参考 frame，与局部 Costmap `global_frame` 一致。 |

### 7.2 轨迹离散和全局路径截取

| 参数 | 当前值 | 类型/单位 | 详细含义与当前效果 |
|---|---:|---|---|
| `teb_autosize` | `true` | bool | 每次外层优化时自动增减轨迹位姿，让相邻时间差靠近 `dt_ref`，官方接口建议开启。 |
| `dt_ref` | `0.45` | s | 期望相邻轨迹位姿的时间分辨率。当前控制周期是 0.25 s，而轨迹参考步长为 0.45 s，轨迹离散相对较粗、计算量较低。它不是控制命令发布周期。 |
| `dt_hysteresis` | `0.1` | s | 自动增删位姿的滞回带，避免时间差在阈值附近导致位姿数量反复变化。通常约为 `dt_ref` 的 10% 左右；当前为约 22%。 |
| `global_plan_overwrite_orientation` | `true` | bool | TEB 根据局部全局路径几何重新计算中间路径点朝向，避免 GlobalPlanner 中间点朝向信息不可靠。最终目标朝向仍受 goal tolerance 约束。 |
| `max_global_plan_lookahead_dist` | `3.0` | m | 按路径累计长度，从全局路径中最多截取 3 m 交给 TEB；还会受局部 Costmap 尺寸限制。值更大看得更远但障碍和轨迹变量增多。 |
| `feasibility_check_no_poses` | `5` | int，位姿数 | 每个采样周期从轨迹起点向前检查最多 5 个预测位姿的 footprint 可行性。值小计算快，但更远的不可行段可能较晚被发现。0 的特殊行为应按插件版本核对，不应当作“检查全部”随意使用。 |

### 7.3 差速底盘运动约束和外形

| 参数 | 当前值 | 类型/单位 | 详细含义与当前效果 |
|---|---:|---|---|
| `max_vel_x` | `0.6` | m/s | 最大向前线速度。最终可达速度还受驱动器和 velocity smoother 限制。 |
| `max_vel_x_backwards` | `0.15` | m/s | 最大后退速度的绝对值。虽然允许倒车，但 `weight_kinematics_forward_drive: 100` 强烈偏好前进。 |
| `max_vel_theta` | `0.30` | rad/s，约 17.2°/s | 最大角速度。按该上限原地转 180° 理论至少约 10.47 s，尚未计加减速。 |
| `acc_lim_x` | `0.30` | m/s² | 线加速度绝对上限。若从静止按上限加速到 0.6 m/s，理论需约 2 s。 |
| `acc_lim_theta` | `0.30` | rad/s² | 角加速度绝对上限，从静止到最大角速度理论约 1 s。 |
| `min_turning_radius` | `0.0` | m | 差速底盘可原地旋转。若底盘实际不能原地转，必须按机械能力改成非零并重新考虑运动模型。 |
| `footprint_model/type` | `polygon` | enum string | TEB 用闭合多边形模型计算机器人到障碍物的距离，而不是点、圆或线模型。 |
| `footprint_model/vertices` | 四个 `[x,y]` 点 | m | `0.566 m × 0.566 m` 方形，坐标相对 `base_footprint`。首点不用重复；必须与 Costmap `footprint` 相同。 |

### 7.4 到达目标判定

| 参数 | 当前值 | 类型/单位 | 详细含义与当前效果 |
|---|---:|---|---|
| `xy_goal_tolerance` | `0.2` | m | 机器人参考点到最终目标位置的允许欧氏距离。它不是 polygon 外缘到目标的距离。 |
| `yaw_goal_tolerance` | `0.5` | rad，约 28.65° | 最终朝向允许误差，比较宽松。对需要精准朝向对接的任务应显著缩小，并同步验证定位和底盘低速控制能力。 |
| `free_goal_vel` | `false` | bool | 到达目标时要求规划速度降为 0。true 允许以非零速度穿过目标，更适合连续路径而非停靠。 |

位置和朝向两个条件都要满足，move_base 才会报告成功。`GlobalPlanner/default_tolerance` 决定全局路径终点能否换到目标附近，作用在更前一级，不能代替这里的到达判定。

### 7.5 障碍物关联和距离

| 参数 | 当前值 | 类型/单位 | 详细含义与当前效果 |
|---|---:|---|---|
| `min_obstacle_dist` | `0.15` | m | TEB 期望 polygon 外缘与障碍物至少相距 0.15 m。它以优化惩罚实现，数值求解中不是物理安全认证的刚性保证。 |
| `inflation_dist` | `0.30` | m | polygon 到障碍物距离小于 0.30 m 时施加额外较弱膨胀惩罚。必须大于 `min_obstacle_dist` 才有独立软缓冲区；当前软区宽约 0.15 m。 |
| `include_costmap_obstacles` | `true` | bool | 从局部 Costmap 提取障碍加入 TEB。当前没有独立 costmap converter，因此这是 TEB 感知三台激光和静态墙体的关键开关。 |
| `costmap_obstacles_behind_robot_dist` | `1.0` | m | 从 Costmap 加入 TEB 时，最多保留机器人后方 1 m 的障碍。值过小会让倒车不安全，过大增加障碍数量。 |
| `obstacle_poses_affected` | `7` | int，邻近位姿数 | 在旧式障碍关联策略中，一个障碍关联到最近轨迹位姿并影响其邻近位姿数量。当前插件版本默认使用新式关联策略时，该参数的重要性会下降，不能只靠增大它解决漏避障。 |

注意：`inflation_dist` 与 Costmap `inflation_radius` 属于不同模块。Costmap 先生成栅格代价，TEB 又以自身 polygon 和距离参数做连续空间近似优化。

### 7.6 数值优化开关与迭代次数

| 参数 | 当前值 | 类型/单位 | 详细含义与当前效果 |
|---|---:|---|---|
| `no_inner_iterations` | `5` | int | 每次外层循环调用求解器的迭代次数。 |
| `no_outer_iterations` | `4` | int | 每个控制周期执行 4 次外层循环；每轮会调整轨迹尺寸并再次进行 5 次内层优化。粗略上每周期最多 20 次求解迭代，但每次成本并不相等。 |
| `optimization_activate` | `true` | bool | 启用轨迹图优化。关闭主要用于调试，正常导航不可依赖未优化轨迹。 |
| `optimization_verbose` | `false` | bool | 不打印求解器详细信息。调试收敛问题时可临时打开，但控制频率下日志量很大。 |
| `penalty_epsilon` | `0.1` | 约束软化量 | 给速度、加速度等硬约束的惩罚函数留出小安全边界，使优化在达到名义上限前开始受罚。它用于多类约束，不能统一当成纯粹的米或无量纲参数；过大可能显著压低可用速度。 |

### 7.7 优化权重

权重都是数值优化目标的相对系数。不同误差项量纲和数值范围不同，因此不能仅凭 `1000 > 50` 就认为前者“重要 20 倍”，也不建议一次同时改多个权重。

| 参数 | 当前值 | 详细含义与当前效果 |
|---|---:|---|
| `weight_max_vel_x` | `1` | 线速度上限惩罚权重。较低，但 `penalty_epsilon` 和速度边界仍参与优化。 |
| `weight_max_vel_theta` | `1` | 角速度上限惩罚权重。 |
| `weight_acc_lim_x` | `1` | 线加速度上限惩罚权重。 |
| `weight_acc_lim_theta` | `1` | 角加速度上限惩罚权重。 |
| `weight_kinematics_nh` | `1000` | 非完整运动学约束权重，强烈抑制差速机器人横向滑移的轨迹。当前值符合差速模型常见设置。 |
| `weight_kinematics_forward_drive` | `100` | 惩罚负向线速度，强烈偏好前进。因此即使 `max_vel_x_backwards=0.15`，TEB 也只会在前进代价很差时考虑倒车。 |
| `weight_kinematics_turning_radius` | `1` | 最小转弯半径约束权重。当前 `min_turning_radius=0`，该约束通常不会成为限制因素。 |
| `weight_optimaltime` | `1` | 压缩轨迹时间、偏向更快到达的权重。提高后可能更积极，但会与避障、速度和加速度约束竞争。 |
| `weight_obstacle` | `50` | 满足 `min_obstacle_dist` 的主要障碍惩罚权重。插件默认还会在外层迭代中用未显式配置的 `weight_adapt_factor` 逐轮放大此权重，所以 50 不是每一轮固定不变的唯一有效数值。 |
| `weight_inflation` | `0.1` | `inflation_dist` 软缓冲惩罚权重，设计上应明显小于主障碍权重；当前比 50 小很多。 |

### 7.8 多拓扑轨迹

| 参数 | 当前值 | 类型 | 详细含义与当前效果 |
|---|---:|---|---|
| `enable_homotopy_class_planning` | `false` | bool | 关闭同伦类并行规划，只优化一条局部拓扑轨迹。CPU 和行为更简单，但在障碍物两侧存在多条绕行选择时，探索替代路径的能力较弱。其他未写入 YAML 的同伦参数即使使用默认值，也不会在此开关关闭时启动多类搜索。 |

## 8. 导航参数的当前数据流

```text
/map --------------------------> StaticLayer ------┐
/scan, /scan_2, /scan_3 ------> VoxelLayer -------+--> InflationLayer
                                                   │
                                                   ├--> GlobalPlanner --> 全局路径
/odom + TF + polygon ------------------------------└--> TEB -----------> /cmd_vel
```

规划输出随后由 `base.launch` 中的 `yocs_velocity_smoother` 处理，再发给底盘。TEB YAML 只是“希望发送的运动上限”，不自动证明底盘能达到或准确跟踪这些数值。

