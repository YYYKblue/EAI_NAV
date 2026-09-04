# Dashgo 当前问题清单

## 范围与判定

本清单基于 2026-09-04、提交`901fd24`的源码、launch、配置和短时运行结果。严重度
表示对真机运动安全、迁移可用性或维护性的影响，不表示问题已经通过硬件复现。
构建成功只能证明编译和依赖解析成立，不能证明底盘协议、急停或传感器失效行为安全。

## 高风险

### 1. ROS急停分支没有清零实际轮速输入

证据位于`src/dashgo_driver/scripts/dashgo_driver.py`的`cmdVelCallback()`：

1. 收到急停状态`emergencybt_val == 1`时，代码只把新建的`robot_cmd_vel`消息清零；
2. 该消息发布后，局部变量`x = req.linear.x`和`th = req.angular.z`保持原值；
3. 后续仍用`x/th`计算`v_des_left/v_des_right`，轮速循环继续调用`Stm32.drive()`；
4. 急停状态更新处也没有直接把期望轮速清零。

因此，当前代码不能证明ROS软件急停会阻止电机命令。STM32固件可能有独立硬件互锁，
但固件不在仓库内，不能据此降低风险。

建议先核对固件协议，再让急停状态直接清零运动输入和左右期望轮速，并在驱动轮抬起、
实体急停可用的条件下验证“运动中按下、保持按下、松开后不自动恢复”三个场景。

### 2. STM32接收校验和被禁用

`Stm32.receiveFiniteStates()`已经累计`receive_check_sum_`，但到
`RECEIVE_CHECK`状态后使用固定真值`if 1`，原来的校验字节比较被注释。结果是长度和
payload满足状态机时，无论尾部校验字节是否正确都会被接受。

这会降低串口噪声、丢字节或错位数据的检错能力。修复前必须先用固件协议或抓包确认
校验覆盖范围以及模255/模256规则，避免仅凭注释恢复一条可能不匹配固件的公式。

### 3. 激光安全门不是fail-safe

真机驱动中的`isPassed`和`isPassed_2`初始值都是`True`；`laser_safety_gate.py`在
scan到box frame的TF不可用时只记录警告并返回，不发布故障状态。驱动也不记录两路
计数消息的时间戳，没有超时闭锁。因此以下情况会保持初始“可前进”或最后一次状态：

- 安全节点未启动或崩溃；
- 对应LaserScan停止；
- TF断链或时间外推持续失败；
- ROS通信中断但底盘节点仍在运行。

现有逻辑只在收到计数且值大于2时禁止正向`linear.x`，仍允许倒车和转向。应根据任务
风险决定是否增加启动握手、消息新鲜度超时和故障置停；改动后需要同时验证真机驱动与
`sim_safety_cmd_gate.py`，避免仿真/真机语义分叉。

## 中风险

### 4. Gazebo默认地图不可移植

`src/dashgo_gazebo/maps/robocup_home.yaml`中的`image`写为：

```yaml
image: /home/bboss/dashgo_ws/src/dashgo_gazebo/maps/robocup_home.pgm
```

它在当前开发机可用，但工作空间移动到其他用户、目录或Nano后，map_server会找不到
图片。该字段可改为同目录相对路径`robocup_home.pgm`；修改后应从不同工作目录启动
`sim_navigation.launch`确认map_server仍能加载地图。

### 5. `ydlidar`缺少直接运行依赖

三份`ydlidar*.launch`都启动`tf/static_transform_publisher`，但
`src/ydlidar/package.xml`没有`tf`依赖。整仓`rosdep check`目前通过，是因为其他包
声明并安装了`tf`；单独安装或发布`ydlidar`包时不保证可运行。应增加
`<exec_depend>tf</exec_depend>`后重新执行rosdep检查。

## 低风险与维护问题

### 6. package元数据仍有占位项

五个自研package的`package.xml`仍使用`bboss@todo.todo`；其中`dashgo_bringup`、
`dashgo_description`、`dashgo_driver`和`ydlidar`的许可证还是`TODO`。这不阻止当前
构建，但会影响包发布、许可证审计和第三方复用。许可证不能仅凭代码用途猜测，应由
仓库权利人确认；`dashgo_gazebo`及其引入的场景资源已经按GPLv3记录并附许可证副本。

### 7. 安全节点关闭时有一次发布竞态日志

完整仿真导航链被SIGINT主动结束时，`laser_safety_gate.py`的一次scan回调可能在
publisher已关闭后执行，产生`publish() to a closed topic`异常。该现象发生在退出阶段，
本轮稳态运行未见节点崩溃；可在发布前判断`rospy.is_shutdown()`或捕获关闭异常来清理
日志，但优先级低于运动安全问题。

## 本轮未发现的阻断项

以下检查在当前环境通过：

- 6个package的Release配置catkin构建；
- 15份YAML与全部package/launch XML解析、Python 3脚本语法检查；
- 17份bringup/Gazebo/description/YDLidar launch递归解析；
- `rosdep check --from-paths src --ignore-src`（仅覆盖已声明依赖）；
- Gazebo基础链30秒无界面启动；
- Gazebo定位导航链35秒无界面启动，地图、AMCL、global/local Costmap、TEB、三激光、
  IMU、里程计、EKF和安全节点均完成初始化。

未验证Nano构建、STM32固件行为、真实串口/雷达、急停、障碍停车距离、实地建图、
实车导航，以及仿真中的目标点到达精度。
