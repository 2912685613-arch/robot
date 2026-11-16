# 仿真启动脚本

> 2026-11-15 控制更新：dock现已使用显式有效视觉与反馈，首次零值只等待；限速接近到15cm后进入HOLD，并保持dock时航向。当前实现以 [修订说明](../../learn/aruco_docking_hold_update.md) 为准。下方历史“单环停机/未接航向”的描述不再适用于当前dock。所有旧节点需退出后重新启动，新消息定义不能与旧进程混用。

## ArUco 定位和对准/接近：aruco_docking.sh

```bash
cd /home/gjx/robot/gazebo_catkin_ws
bash src/gazebo_launch/aruco_docking.sh
```

先关闭之前的 Gazebo/ArUco launch。脚本会读取工作空间 `devel/setup.bash`，依次在后台启动：

1. 水下世界。
2. 带底部相机的动态机器人，初始位置 (0,0,-0.1)，`static_model=false`。
3. 水下关节控制器。
4. 位于 (0.10,0,-1.0) 的水平同心双码，大码 ID 5 黑边 0.20 m，中心小码 ID 18 黑边 0.02 m。
5. 决策、运动、关节及推进器控制，关闭另开的键盘输入节点。
6. ArUco 检测、坐标转换和位姿日志；实际标记边长设置为 0.20 m。
7. 默认重置机器人到配置的初始位置、零姿态、零线/角速度，取得定位后暂停物理，等待用户输入。这个准备动作不属于视觉定位或自主悬停算法。

每个阶段检查服务/模型/控制器/反馈是否就绪，再继续后续步骤。检测就绪检查要求模型、CameraInfo、视觉位姿和机体输出都有效。准备阶段未取得定位时，暂停世界并保留命令界面，可输入 reset 重试；无法执行暂停或其他核心启动检查失败时则退出并清理。

只显示 Gazebo，后台节点不弹出终端；当前终端出现 `aruco>` 后输入：

```text
dock       恢复物理，确认有效反馈后限速接近，随后保持15cm与当前航向
st         停止运动，并在默认配置下暂停物理
reset      停止并重置初始位姿/速度，重新定位后暂停
pause      先停止对接再暂停物理，resume后不会自动继续对接
resume     仅恢复物理，不自动发送 dock
status     查看节点和相机/位姿话题的注册状态
help       查看脚本帮助
quit       退出并清理本次启动的进程
```

也支持 `xyz 0 0 -0.3`、`dep -0.3` 等现有控制命令，发送前自动恢复物理。脚本不会自动发送 dock。默认等待时物理暂停，相机画面和 ROS 仿真时间也会冻结，位姿日志可能记录数据源过期；这不是相机故障。`Ctrl+C` 或关闭标准输入也会清理本次后台进程，包括 roslaunch 创建的新 session 中的节点；按运行标记和精确 ROS 日志路径匹配，并核对 PID 启动时间，不使用 killall/pkill。已存在的外部 ROS Master 不会被标成本次运行。启动检测发现已有 Gazebo 或同名控制节点时会直接拒绝启动。

### 可选参数

```bash
# 同时打开图像查看窗口，选择 /aruco_det_image 看识别结果。
IMAGE_VIEW=true bash src/gazebo_launch/aruco_docking.sh

# 不打开 Gazebo 客户端界面。相机传感器仍需要可用的渲染环境。
GUI=false bash src/gazebo_launch/aruco_docking.sh

# 修改标记位置、日志采样频率和每阶段启动超时。
MARKER_X=0.05 MARKER_Y=0 MARKER_Z=-1.2 LOG_RATE=5 STARTUP_TIMEOUT=90 \
  bash src/gazebo_launch/aruco_docking.sh

# 恢复旧的持续运行模式，等待输入时机器人可能在动力学作用下漂移。
WAIT_PAUSED=false bash src/gazebo_launch/aruco_docking.sh

# 关闭旧脚本残留的 Gazebo 窗口及本次节点，不需要 ROS Master 正常工作。
bash src/gazebo_launch/aruco_docking.sh --stop

# 明确指定要清理的历史运行目录。
bash src/gazebo_launch/aruco_docking.sh --stop /你的工作空间/logs/aruco_docking_...

bash src/gazebo_launch/aruco_docking.sh --help
```

机器人初始位置可用 ROBOT_X、ROBOT_Y、ROBOT_Z 调整。标记需处于底部相机视野内。

### 日志

所有日志保存在同一次运行目录，终端会打印完整路径：

```text
logs/aruco_docking_<本地时间>_<随机后缀>/
  startup.txt           本次位置、图形选项、日志频率等配置
  world.log             世界启动及 Gazebo 输出
  model.log             机器人生成输出
  controllers.log       控制器输出
  marker.log            标记生成输出
  robot_control.log     决策与运动控制输出
  aruco.log             检测、转换和日志节点输出
  ready_*.log           各阶段就绪检查（ready_prepare.log 含重置/暂停动作）
  commands.log          命令发布输出
  cleanup.log           本次进程清理输出
  ros/                  ROS 原生节点日志
  aruco_gazebo_robot1_*/ 位姿关系日志 events.log、poses.csv、config.json
```

### 当前控制能力的边界

此脚本复用现有 dock 控制代码，只减少启动终端和手动配置，不修正其算法：当前每轮先判断 XY 阈值并选择一次 XY 或 XYZ 控制，没有阶段回差；目标 yaw 未真正接入、没有自动对接成功判定或机械锁定，决策节点也没有视觉丢失超时保护。脚本只在发送 dock 前检查定位，不能替代运行时的丢失保护。标记是视觉平面，没有对接碰撞体。

输入 dock 后恢复动态物理；本次修复没有新增自动悬停或对接姿态控制。出现异常运动或标记丢失时输入 st，默认会暂停世界；输入 reset 可以回到初始位置重新试验。Gazebo 窗口的关闭按钮通常只关闭客户端；请优先使用当前终端的 quit 或 Ctrl+C 结束整套仿真。先前的静态 `roslaunch asrobot aruco_sim.launch` 仍适合只验证定位，但机器人被固定，不能演示主动接近。

### 离线验证

以下检查不启动 ROS Master 或 Gazebo：

```bash
bash -n src/gazebo_launch/aruco_docking.sh
/usr/bin/python3 src/gazebo_launch/tests/test_aruco_docking_script.py
/usr/bin/python3 src/gazebo_launch/tests/test_cleanup_aruco_docking.py
source devel/setup.bash
/usr/bin/python3 src/gazebo_launch/tests/test_docking_prepare.py
```

脚本测试用替身命令检查启动顺序、暂停/恢复、reset、命令转发、无有效检测时拒绝 dock、失败清理、EOF 和中断退出。另有真实替身进程测试跨 session 清理及运行隔离，以及模拟服务调用测试准备失败时仍执行暂停。

### 2026-11-15：对接高度目标

`motion.cpp` 的 `DOCKING_MARKER_Z_TARGET=-0.15f` 表示标记相对机体原点的 Z 目标。XY 对齐后，Z 误差改为 `测得标记 Z + 0.15`；测得标记 Z 达到或超过 -0.15 m 时，先退出对接到 IDLE、重置 PID 并关闭推进器。该停机判定优先于 XY 对齐，防止横向未对齐时继续越过高度阈值。它不是接触/锁定判定，也不是闭环悬停或世界暂停，停止后惯性和浮力仍可能造成位移。

相机光心在机体下方 12 cm，因此机体 15 cm 间距对应光心约 3 cm 距离。当前 20 cm 大码可能在这之前就超出画面；已有丢码缓存问题尚未修复，不能保证总能以视觉触发该阈值。后续应先加持续有效性/超时保护，再讨论小码或独立高度反馈的末段接近。


当前演示目标已包含中心 ID18 小码，原脚本会显式传入 `aruco_id_inner=18`、`aruco_length_inner=0.02`。两个码中心和朝向一致；默认大码相机 Z 小于 0.5 m 且小码可见时切换。小码在远处像素少，未必每帧可见，重叠区和近距离识别已做离线检查，实际效果需在 Gazebo 中确认。此前关于“大码近处无法完整入镜”的限制仍适用于大码，小码用于接续近距离测距。


2026-11-15最新故障修订：小码改为0.30m内且最短边≥24px、连续3帧再切换；丢小码时优先回退当前可见的大码。动态控制在机体反馈有效时给视觉丢失1.5s制动/重获窗口（目标vz=0），连续3张新图像确认后平滑恢复。长时丢失或反馈故障仍进入LOST_STOP，需再次dock。详细证据与日志字段见learn/aruco_docking_hold_update.md第12节。


最新通信修订（2026-11-15）：MC沉默0.5s先零输出进入WAIT_TRANSPORT；仿真/clock冻结时等待恢复，不立即永久锁死，恢复后必须连续确认3张停顿后生成的新图像与新反馈。/clock仍前进但MC长期中断，按transport_timeout=2.0s锁存LOST_STOP。数据有效期仍为0.5s。脚本pause先发st；GUI暂停则按模拟时钟停顿处理，恢复后可能继续此前对接。
