# ArUco 对接修订：有效反馈、限速接近、15 cm 保持与航向保持

更新日期：2026-11-15。本说明对应当前 `aruco_docking.sh`，替代旧教学文档中“到达15cm立即切IDLE”“对接使用单环”“X型不控制航向”的描述。

## 1. 这次实际改变了什么

现在 dock 的控制流程是：

```text
输入 dock
  → 关推进器，把四腿水平转到 (40,-40,40,-40)°，留1秒转向时间
  → WAIT_VISION：等待有效视觉和独立机体反馈，不把零值当作到位
  → XY_ALIGN：横向对准，同时保持此时捕获的标记相对Z间距
  → APPROACH：XY持续对齐，位置外环给目标垂直速度，速度内环给控制力
  → HOLD：误差、速度、航向持续稳定后，保持15cm目标，不关掉控制器
```

任一活动阶段视觉或反馈超时、数值无效、时钟回退，就进入 `LOST_STOP`，关闭推进器并锁存停止。重新看到标记也不自动恢复，需再次输入 `dock`。没有增加无目标自动搜索。

`st` 是人工停止，不是 HOLD；脚本默认随后暂停物理。`quit` 退出并清理进程。

## 2. 首次零目标误停是如何修掉的

旧实现收到 `dock` 后立即使用 `cam_e_z` 判断是否达到 -0.15，而普通 mc 消息的该字段可能还为默认0。现在不再用“Z是否为0”推测有效性。

`camera_data.msg` 增加：

```text
bool aruco_valid
float64 detection_stamp
```

`aruco_posture` 保留源图像的 ROS 时间戳。无首帧或超过有效期时，会发 `aruco_valid=false`；此时位置字段不能当作新测量。

`decision::downCamCallback()` 缓存标志、源时间和接收墙钟；发布 `mc` 前统一检查有效性。mc 增加：

```text
bool aruco_valid
float64 aruco_stamp
bool feedback_valid
float64 feedback_stamp
float64 body_vx
float64 body_vy
float64 body_vz
float64 body_yaw_deg
float64 body_yaw_rate
```

普通反馈消息也附带最新视觉值与有效性；即使 motion 先收到 dock，也会等待有效消息，不会因0值退出。

对接时 decision 每轮只发一条 mc，关闭原来叠加的 `SINGLE_ROBOT_DEBUG` 发布，避免重复更新控制积分。`st` 会清除 `tracking_aruco_flag`。

## 3. 速度和航向反馈来自哪里

当前是 Gazebo 示例：decision 从 `/gazebo/model_states` 读取机器人的四元数、世界线速度和角速度：

```text
body_velocity = R_world_body.transpose() × world_velocity
body_yaw_deg = atan2(R(1,0),R(0,0)) × 180/pi
body_yaw_rate = (R.transpose() × world_angular_velocity).z
```

因此 XY/Z 视觉误差与速度反馈都使用机体系；航向目标与当前航向使用世界系 yaw，角速度为 rad/s。

这不是仅靠 ArUco 的完整机器人定位：速度和航向使用了 Gazebo 真值。真实机器人需换成经验证的速度、IMU/导航反馈；不能直接把测试成功等同于真机闭环可用。

## 4. 对接控制放在哪里

新增 [aruco_docking_control.h](../src/asrobot/include/asrobot/aruco_docking_control.h) 中的 `aruco_docking::Controller`，只处理数学和状态，不依赖 ROS。

`motion.cpp::handle_docking_aruco()` 从 mc 构造 Input，调用 `Controller::update()`，再把输出交给 `C_VectoredThrustControl::dockingControl()`。旧的手动XYZ、模糊PID和通用 `space3DControl()` 路径保留原行为，没有全局改为另一套控制。

`RobotMode` 仍是 `DOCKING_ARUCO`，WAIT/ALIGN/APPROACH/HOLD/LOST 是该模式内部的状态，由 `[DOCK STATE]` 日志说明。HOLD 不要求用户再发命令。

## 5. Z双环、限速与保持

机体原点位于标记上方15cm对应视觉测得 `marker_body_z=-0.15 m`。接近/保持误差：

```text
z_error = measured_marker_z - target_z
```

远处测得 -0.50：误差 -0.35，应向下；过近测得 -0.14：误差 +0.01，应向上。

当前专用控制器外环：

```text
desired_vz = clamp(position_kp × z_error, -speed_limit, +speed_limit)
```

接着限制目标速度变化率，再由带显式反馈ROS采样 dt 和抗积分饱和的 PI 速度内环产生 Fz：

```text
velocity_error = target_vz - body_vz
Fz = clamp(velocity_kp × velocity_error + integral, -force_limit, +force_limit)
```

默认远处目标速度上限2cm/s，距目标10cm内使用0.5cm/s，目标加速度上限2cm/s²。由远处进入末段时，目标速度按加速度约束渐变，不会瞬间切换。

这些是**控制器期望速度的限制，不是物理实际速度的硬保证**。惯性、推力饱和、动力学误差等仍可能使实际速度暂时超出，需要用日志检查。

XY未对齐时，Z目标保持本轮进入ALIGN时捕获的相对间距，不再简单地将两个Z都填0。对齐后才恢复接近目标。接近中XY超出允许带，会重新进入ALIGN并停止继续下降。

HOLD判定同时要求：

- XY误差在随距离收紧的阈值内。
- Z误差小于1cm。
- 垂直速度和水平速度都小于0.5cm/s。
- 航向误差小于3°。
- 连续稳定至少1秒，并有新的视觉帧确认。

HOLD继续输出控制力抵消漂移。偏离两倍Z允许误差或XY退出带时返回接近/对齐，目标仍是15cm，不靠手动重新dock。没有机械接触、吸附或锁定动作。

“15cm”仍以机体原点、机体Z轴定义；相机光心在下方12cm，因此最终光心离标记约3cm。相机位置或标记尺寸变化后必须重新核对。

## 6. 为什么XY阈值近处更小

原来的5cm阈值比近处整个画面还宽，允许机体带着大偏差继续下潜。现在：

```text
camera_depth = max(0, -marker_body_z - 0.12)
enter_xy = min(xy_tolerance, max(0.004, camera_depth × 0.12))
```

默认远处最多3cm，近处收紧到约4mm；另有1cm退出回差。HOLD也使用近处阈值。这个值是控制调试起点，可能还需要结合实际图像噪声和惯性调整，不能承诺所有初始姿态都能捕获小码。

## 7. 航向保持的目标是什么

首次dock有有效反馈时捕获当前机器人世界yaw，否则在首个有效控制样本捕获。随后保持该航向：

```text
yaw_error_deg = remainder(yaw_target - yaw_current, 360)
Mz = clamp(yaw_kp × yaw_error_deg - yaw_kd × body_yaw_rate,
           -yaw_limit, +yaw_limit)
```

这次实现的是**保持dock时的航向**，不是强制机器人转到0°，也不是对准ArUco图案yaw。这样大小码姿态噪声不直接改变航向目标。未来若需要与标记方向一致，应另加明确的目标航向模式。

## 8. 为什么不能只给旧X型分配加一个yaw数值

实际CAD在原X形态水平角为0时，推进器近似径向指向机体中心，偏航力臂只有约0.5mm，分配矩阵条件数约1824，几乎不能独立产生yaw。

对接专用形态使用H关节角 `(40,-40,40,-40)°`，将偏航力臂提高到约4.3cm，分配矩阵条件数约23。改变腿角同时改变XY方向，必须一起重算，不能继续使用旧 `sqrt(2)/4` 系数。

新增 [aruco_docking_allocation.h](../src/asrobot/include/asrobot/aruco_docking_allocation.h) 保留从实际URDF和关节校准计算得到的方向与力臂：

```text
[推进器水平X分量]
[推进器水平Y分量] × 四个水平推力 = [Fx, Fy, Mz, 总基础水平推力]
[推进器偏航力臂]
[1,1,1,1]
```

求解后把Z分量分到四腿，输出转向角；实际推力按UUV的 Basic 模型逆解速度：

```text
omega = sqrt(force / 0.00031)
```

不再把旧的线性“力→转速”映射当成精确物理力。四电机饱和时统一缩放水平增量，尽量保留Fx/Fy/Mz比例；竖直分量受腿角和电机能力限制，实际分配缩放写入日志。

**该几何专用于 `new_gazebo_model` 与当前 `joint_command_calibration.yaml`。** 修改模型、关节零位或推力常数后须重新计算，并重新运行几何检查。没有把这套常量推广到其他机器人模型。

离线几何检查使用真实URDF、校准后的关节变换、`r×F` 力矩及UUV推力模型，验证正/负yaw和XYZ组合。它验证的是稳态几何关系；实际关节跟随误差、角度命令取整和水动力仍需仿真验证。

## 9. 超时与暂停

视觉消息保留源时间，decision同时检查ROS年龄和接收墙钟；默认超过0.5秒无效。ModelStates没有header，feedback_stamp是decision接收时的ROS时间；这些数据没有严格硬件同步。

运动节点还有50ms墙钟定时看门狗：即使整个mc消息流断掉，或ROS时间因暂停而不继续，超过0.5秒仍关推进器并进入LOST_STOP。不会等到下次视觉回调才执行安全停止。

首帧等待最多3秒；首条无效消息只是等待，不是到位。活动阶段丢失则锁存停止，恢复画面后需重新dock。

脚本默认等待时暂停；dock前恢复物理并检查定位。若活动对接中手动pause，超时会锁存LOST_STOP；resume后需再次dock。st直接进入IDLE，不会继续输出HOLD控制。

## 10. 参数与日志

参数在 [docking_control.yaml](../src/asrobot/config/docking_control.yaml)，`asrobot.launch` 加载到 `/robot1/motion/docking/`。程序直接rosrun时使用同样默认值。

| 参数 | 默认值 | 用途 |
| --- | ---: | --- |
| target_z | -0.15 | 标记相对机体Z目标，m |
| max_speed / near_speed | 0.02 / 0.005 | 目标垂直速度限幅，m/s |
| near_distance | 0.10 | 末段误差范围，m |
| acceleration | 0.02 | 目标速度变化率，m/s² |
| position_kp | 0.3 | 高度误差到目标速度 |
| velocity_kp / velocity_ki | 16 / 0.5 | 垂直速度PI |
| z_force_limit | 0.6 | 竖直控制力幅值，N |
| xy_kp / xy_kd | 0.6 / 0.8 | 位置修正和速度阻尼 |
| yaw_kp / yaw_kd | 0.003 / 0.04 | 航向修正和角速度阻尼 |
| yaw_limit | 0.03 | 偏航力矩幅值，Nm |
| timeout / wait_timeout | 0.5 / 3 | 数据超时和初始等待，s |
| settle_time | 1 | HOLD稳定确认时间，s |

`robot_control.log` 和 `events.log` 包含：

- `[DOCK STATE]`：WAIT、ALIGN、APPROACH、HOLD、LOST状态切换。
- `[DOCK CONTROL]`：源图时间、年龄、测量xyz、动态XY阈值、Z误差、目标和实际vz、航向目标/误差、Fz和Mz。
- `[DOCK ALLOC]`：请求力、分配饱和系数和四电机指令。

日志config.json增加motion的对接参数快照。无效camera_data不会再以全零位置写成有效发布结果。

## 11. 如何验证

消息定义已变化，旧进程使用旧MD5不能继续混跑；先退出旧仿真，再加载重新编译后的工作空间：

```bash
cd /home/gjx/robot/gazebo_catkin_ws
source devel/setup.bash
IMAGE_VIEW=true bash src/gazebo_launch/aruco_docking.sh
```

第一次dock应等待有效反馈后进入ALIGN/APPROACH，不再出现 `DOCKING STOP: ... Z=0`。小码切换照旧。最后应看到HOLD而非IDLE，并观察数秒的间距、实际速度和yaw是否稳定。

故意让视觉丢失时，最多经过超时应看到LOST_STOP、电机为0；重获画面不自动冲刺。不要在首次验证中把无反馈自动搜索也加进来。

离线检查：

```bash
source devel/setup.bash
/usr/bin/python3 src/asrobot/test/test_docking_stop_height.py
/usr/bin/python3 src/asrobot/test/test_docking_geometry.py
/usr/bin/python3 src/asrobot/test/test_aruco_pose_logger.py
```

消息集成检查使用独立端口的本机ROS Master及实际decision/motion进程，只发布测试消息，不启动Gazebo：

```bash
/usr/bin/python3 src/asrobot/test/test_docking_ros_transport.py
```

代码和检查通过不代表已经在真实动力学中调好。实际HOLD误差、yaw方向、限幅和收敛需要下一次Gazebo日志确认；已有手动控制的参数不应直接当作本次专用控制器调好的证据。
