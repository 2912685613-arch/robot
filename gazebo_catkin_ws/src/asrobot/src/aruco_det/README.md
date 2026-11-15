# ArUco 相对定位代码与仿真示例

> 2026-11-15：camera_data现增加aruco_valid和detection_stamp；无效/过期时显式发布false。mc同时传递源时间和独立机体反馈。动态dock的有效性保护、限速/HOLD/航向保持详见 [当前修订说明](../../../../learn/aruco_docking_hold_update.md)。历史“消息无有效标志”的说明已被本次更新替代。

本目录检测标记相对相机的位姿，并转换为相对机器人机体的位姿。这里的“定位”是标记相对机器人的定位；当前没有发布机器人在世界坐标系中的视觉定位结果。

## 启动独立示例

在工作空间根目录执行；此命令会创建 Gazebo 世界，不要同时运行其他启动 Gazebo 的脚本。

```bash
cd /home/gjx/robot/gazebo_catkin_ws
source devel/setup.bash
roslaunch asrobot aruco_sim.launch
```

`aruco_sim.launch` 默认配置：

- 固定机器人 gazebo_robot1，机体原点位于世界坐标 (0, 0, 0)，航向为零。
- 底部相机的光心位于机体系 (0, 0, -0.12)，向下观察；图像为 640 × 480，水平视场角为 90°。
- 水平朝上的同心双码位于世界坐标 (0, 0, -1)，字典 DICT_6X6_250（编号 10），大码 ID 5、小码 ID 18。
- 大码黑色外边框边长 0.20 m，中心小码黑色外边框边长 0.02 m，含白边的纹理平面边长 0.28 m。两个码同心同向，大小码尺寸均已传入检测节点。
- 机器人被固定，便于先验证视觉位置；静态模式只保留相机，不加载水动力、推进器、ros_control、IMU 和压力插件，也不启动运动控制和决策节点。
- 默认不弹出 OpenCV 窗口，仍然发布标注图像。需要窗口时加 `show_debug_image:=true`。

只关闭 Gazebo 客户端界面、保留相机传感器时：

```bash
roslaunch asrobot aruco_sim.launch gui:=false
```

相机渲染仍需本机图形环境；`gui:=false` 仅表示不启动 Gazebo GUI。

## 查看结果

新终端先 source 同一个工作空间，再执行：

```bash
# 查看原始相机图像或 /aruco_det_image 标注图。
rosrun rqt_image_view rqt_image_view

# 实际相机内参和畸变参数，检测节点默认直接订阅此话题。
rostopic echo -n 1 /gazebo_robot1/camera_bottom/camera_info

# 标记在相机光学坐标系下的位姿。
rostopic echo /aruco/pose

# 标记在机体坐标系下的位置和姿态角。
rostopic echo /camera_aruco_data
```

完整接口：

| 话题 | 内容 |
| --- | --- |
| /gazebo_robot1/camera_bottom/image_raw | 原始图像，sensor_msgs/Image |
| /gazebo_robot1/camera_bottom/camera_info | 实际相机标定，sensor_msgs/CameraInfo |
| /aruco_det_image | 带检测框和坐标轴的图像 |
| /aruco/pose | 标记相对相机光学系的 PoseStamped |
| /camera_aruco_data | 标记相对机体系的 asrobot/camera_data |

话题路径中只有一个 camera_bottom。图像及位姿的 frame_id 为 `gazebo_robot1/camera_bottom_optical_frame`。

默认几何配置下，位置输出应接近以下数值，允许像素检测和位姿估计误差：

```text
/aruco/pose:
  position.x ≈ 0
  position.y ≈ 0
  position.z ≈ 0.88       # 标记到相机光心的距离

/camera_aruco_data:
  position_x ≈ 0
  position_y ≈ 0
  position_z ≈ -1.00      # 标记在机体原点下方 1 米
```

关闭示例后，重新启动以验证横向或距离变化：

```bash
# 标记沿世界 X 轴移到 0.10 m，零航向下机体 position_x 应接近 0.10。
roslaunch asrobot aruco_sim.launch marker_x:=0.10

# 标记下移到 -1.30 m：相机 Z 应接近 1.18，机体 position_z 应接近 -1.30。
roslaunch asrobot aruco_sim.launch marker_z:=-1.30
```

还可以通过 robot_x、robot_y、robot_z、robot_yaw、marker_y 和 namespace 调整初始条件。robot_yaw 使用弧度。参数移动过大可能使标记离开视野或小到无法识别；先用小幅位移验证。

## 接入已有机器人仿真

只生成带相机的动态机器人模型时，为模型启动文件传入：

```bash
roslaunch new_gazebo_model new_gazebo_model.launch enable_bottom_camera:=true
```

这条命令需要已有 Gazebo 世界；控制器和标记需按原仿真配置启动。`enable_bottom_camera` 默认 false，`static_model` 默认 false。独立示例显式启用相机并把 static_model 设为 true。

`single_water.sh` 在 ENABLE_ARUCO=true 时会同时启用相机和检测。但其默认世界没有标记，首次验证请优先使用上面的独立示例，不要同时启动两套检测节点。

已有相机和标记时，可只启动检测与坐标转换：

```bash
roslaunch asrobot aruco_det.launch aruco_length_outer:=0.2
```

默认从 camera_info 读取标定；如果使用 YAML，需显式关闭 CameraInfo 模式，并确保文件参数匹配当前相机：

```bash
roslaunch asrobot aruco_det.launch use_camera_info:=false \
  camera_param_path:=/你的路径/camera.yaml
```

YAML 字段为 fx、fy、x0、y0、k1、k2、p1、p2、k3。默认路径通过 `$(find asrobot)` 解析，不再绑定旧用户名。更改相机安装时，除 camera_x/y/z 平移参数外，还需同步调整模型安装方向与 aruco_posture 中 Tbc 的旋转矩阵。

## 运行链路和每个函数

```text
底部相机 image_raw + camera_info
  -> aruco_det.cpp：检测角点、选择目标、估计位姿
  -> /aruco/pose
  -> aruco_posture.cpp：Tba = Tbc * Tca
  -> /camera_aruco_data
  -> decision.cpp 的 downCamCallback（启动决策模块时）
```

“重要／次要”表示阅读优先级，不表示可以直接删除。

| 文件 | 函数 | 功能 | 阅读优先级 |
| --- | --- | --- | --- |
| aruco_det.cpp | camera_infoCallback | 读取和检查 K、D 标定参数 | 重要：标定输入 |
| aruco_det.cpp | cam_imageCallback | 转为 BGR8，缓存图像与采集时间 | 重要：图像输入 |
| aruco_det.cpp | main | 参数与标定、检测大小码、切换目标、位姿计算和发布 | 最重要：检测主流程 |
| aruco_posture.cpp | modelStatesCallback_ | 构建机体到世界系的 Twb | 可后读：仅用于世界位姿调试 |
| aruco_posture.cpp | aruco_pos_cb | 验证参考系与数值，构建 Tca | 重要：转换输入 |
| aruco_posture.cpp | main | 安装变换、超时判断、机体系位姿发布 | 最重要：坐标转换主流程 |
| aruco_det_singlemaker.cpp | cam_imageCallback | 缓存单码版本的输入图像 | 参考：当前未编译 |
| aruco_det_singlemaker.cpp | main | 估计同尺寸标记的位姿，只发布指定 ID | 参考：学习基本检测时阅读 |
| aruco_demo.cpp | state_cb | 缓存飞控状态 | 参考：无人机示例 |
| aruco_demo.cpp | pos_cb | 构建无人机 Twb | 参考：无人机示例 |
| aruco_demo.cpp | aruco_pos_cb | 构建无人机示例的 Tca | 参考：无人机示例 |
| aruco_demo.cpp | main | 搜索、对准、下降和降落状态机 | 参考：当前链路可跳过 |

## 可以后读的部分

- ROS 日志、画框、画坐标轴、imshow 和标注图发布，主要用于观察结果。
- 被注释掉的旧内参、旧坐标计算和试验语句，当前不执行。
- aruco_posture 的 Twb、Twa 世界位姿调试计算，不参与当前机体系结果。
- aruco.txt 是矩阵试验笔记，output/ 当前为空；本模块没有读写它们。

## 坐标、有效性和范围

- Txy 表示从 y 系到 x 系；w=世界，b=机体，c=相机光学系，a=标记。
- Tbc 的旋转对应 (x_b,y_b,z_b)=(-y_c,-x_c,-z_c)，再加安装平移 (0,0,-0.12)。
- 位置单位是米，cam_roll/pitch/yaw 单位是度。旧的固定 +0.06 m 补偿已改为 position_z_offset 参数，默认 0；相机实际安装偏移由 Tbc 处理。
- 默认大小码 ID/边长为 5/0.1 m、18/0.02 m。独立示例覆盖大码边长为 0.2 m，并放置同心 ID 18 小码（0.02 m），支持现有大小码切换逻辑。
- 切换使用相机 Z 分量；默认近于 0.5 m 转小码、远于 0.7 m 尝试转大码，形成回差。
- 每次图像回调只触发一次检测；输出保留图像采集时间，而不是把旧图标成当前时间。
- 位姿未到达前不发布机体结果，超过 pose_timeout（默认 0.5 s）后停止发布；订阅者仍应自己检测数据超时。camera_data 消息本身没有 header 或有效标志，停止发布不会清除订阅者缓存。
- 双码切换仍是两个独立标记的位姿选择，没有融合或补偿两个标记中心之间的偏移。
- Twb 来自 Gazebo 真值，不能用 Twa 调试输出声称完成了机器人世界系视觉定位。
- 无人机参考程序使用不同消息包类型，不能直接接入当前接口。

代码已完成编译和离线配置检查；Gazebo 图像渲染、实际话题发布与定位效果需由你启动仿真后验证。

## 离线检查命令

以下命令不会启动 Gazebo：

```bash
source devel/setup.bash
cmake --build build --target aruco_det aruco_posture -j2
/usr/bin/python3 src/asrobot/test/test_aruco_setup.py
/usr/bin/python3 src/gazebo_launch/tests/test_single_water.py
```

新增检查覆盖相机光学轴及安装偏移、标记 ID 和物理尺寸、完整示例的 launch 参数解析。

## 位姿与诊断日志

启动 `aruco_sim.launch` 或 `aruco_det.launch` 后，默认自动启动只读日志节点。每次运行创建独立目录，不覆盖旧记录：

```text
logs/aruco_gazebo_robot1_<UTC时间>_<随机后缀>/
  config.json    本次相机安装、检测、转换和日志参数
  events.log     可读位姿、数据源状态、标定变化、检测/转换节点的诊断消息
  poses.csv      便于画图或比较的位姿数据，默认每秒 2 行
```

`logs/` 默认是本工作空间的 `/home/gjx/robot/gazebo_catkin_ws/logs/`。启动后终端会打印此次运行的完整目录。目录名称和日志墙钟时间使用 UTC，并同时记录 ROS 仿真时间与图像采集时间，避免将暂停前的数据误认为新检测。

位姿日志字段含义：

| 前缀 | 记录的关系 |
| --- | --- |
| robot_world | 机器人在世界系中的位姿 Twb，来自 Gazebo 真值 |
| camera_body | 相机光学系在机体系中的安装位姿 Tbc |
| camera_world | 相机光学系在世界系中的位姿 Twc = Twb × Tbc |
| marker_camera_est | 视觉检测的标记相对相机位姿 Tca |
| marker_body_est | 计算得到的标记相对机体位姿 Tba = Tbc × Tca |
| marker_world_est | 结合 Gazebo 机器人真值计算的标记世界位姿 Twb × Tbc × Tca |
| marker_world_truth | 指定标记模型在世界系中的 Gazebo 真值 |
| marker_camera_truth | 用 Gazebo 真值反算的标记相对相机位姿 |
| marker_body_truth | 用 Gazebo 真值反算的标记相对机体位姿 |
| published_body_* | `/camera_aruco_data` 实际发布的位置和姿态角，包含配置的 z 补偿 |
| position_error_m | 视觉推算的标记世界位置与标记模型真值之间的欧氏距离 |

每个位姿包含 xyz（米）、qx/qy/qz/qw 四元数和 roll/pitch/yaw（度）。文本日志直接显示 xyz 与姿态角。CSV 还保存 robot_valid、pose_valid、truth_valid、camera_info_fresh、图像时间及数据年龄；缺少或过期的位姿字段留空，不用零值伪造有效结果。

`events.log` 记录：

- 相机的图像尺寸、光学参考系、内参 K、畸变 D，首次收到或参数变化时输出。
- 机器人、标记真值、检测、相机标定、机体输出从等待到就绪、从就绪到过期的状态变化。
- 错误参考系、非法四元数和非有限数值。
- 来自检测与转换节点的 ROS 日志，包括目标切换、未识别标记、检测超时与配置错误。日志节点启动前的 ROS 消息可能未被接收，因此另外保存参数服务器快照和日志节点生效参数；检测/转换程序内部未写入参数服务器的默认值仍以源码为准。

调整日志参数：

```bash
# 每秒记录 5 次，并指定目录。
roslaunch asrobot aruco_sim.launch log_rate:=5 log_dir:=/你的路径/logs

# 关闭日志节点。
roslaunch asrobot aruco_sim.launch enable_logging:=false

# 已有仿真：只有明确当前检测目标对应哪个模型时，才启用真值比较。
roslaunch asrobot aruco_det.launch aruco_length_outer:=0.2 \
  marker_model_name:=aruco_demo_marker
```

普通 `aruco_det.launch` 默认不指定标记真值模型；独立示例自动使用 `aruco_demo_marker`。模型原点必须对应被检测标记中心。`/aruco/pose` 没有标记 ID 字段，日志不做自动 ID 关联；切换到另一个标记后，原模型的真值比较不再成立。真值模型坐标轴也未必与图案定义的坐标轴一致，因此只计算位置误差，不直接计算姿态误差。

`/gazebo/model_states` 没有采样时间戳，日志使用最近收到的真值并记录其接收时间和墙钟年龄，不代表精确时间同步。`published_body_*` 所用消息也没有 header，单独保留最近收到的值；动态运动时不可把这些异步样本当作严格同步的误差评估。

日志节点即使在仿真暂停时也按墙钟周期记录；超过有效期的检测、真值和机体输出会置为不可用。这些日志不修改定位或控制结果。

离线验证日志数学和文件输出，不启动仿真：

```bash
source devel/setup.bash
/usr/bin/python3 src/asrobot/test/test_aruco_pose_logger.py
```

## 排查“窗口打开但机器人和标记不见了”

先查看终端是否已经出现两次 `SpawnModel: Successfully spawned entity`。若随后看到
`UnderwaterObjectPlugin.cc` 的 `Linear or angular accelerations are invalid` 断言，
以及 `gazebo ... exit code 134`，说明模型已生成，但 Gazebo 服务端崩溃，GUI 仍可能留在屏幕上。

静态模型的动力学加速度可能无效，静态视觉示例不能加载水动力插件。模型已按 static_model
条件跳过这些动力学插件；动态模式仍加载原来的控制与动力学配置。修改 xacro 后无需重新编译，
关闭原来的 launch，再运行 `roslaunch asrobot aruco_sim.launch` 即可。

`test_aruco_setup.py` 会分别展开静态/动态 URDF 并转换为 SDF，检查静态模型只有相机插件、
动态模型仍保留水动力/推进器/控制插件；这个检查不启动 Gazebo。

## 单终端启动动态对准/接近试验

已有动态控制流程已打包到后台启动脚本，无需逐个打开终端：

```bash
bash src/gazebo_launch/aruco_docking.sh
```

出现 `aruco>` 后输入 `dock`，停止用 `st`，退出用 `quit` 或 Ctrl+C。需要图像窗口可加 `IMAGE_VIEW=true`。详见 `src/gazebo_launch/README.md`；该脚本复用现有接近控制，没有新增对接成功/锁定机制。


## 同心小码目标（2026-11-15）

当前 marker5.png 已改为同心 ID5/18 复合纹理。1120×1120 像素对应 0.28 m 平面：大码黑边 800 px 对应 0.20 m；小码黑边 80 px 对应 0.02 m，四周有 20 px 白边。大小码中心和朝向相同，切换无需位置偏移补偿。

生成脚本是 `src/gazebo_world/scripts/generate_aruco_demo_marker.py`，使用本机 OpenCV 字典生成，可重复运行。该纹理局部覆盖了大码中心，是否仍可解码必须以测试为准，不能任意扩大覆盖范围。

离线投影测试覆盖：相机距离 0.78 m 的大码，0.50/0.40/0.35/0.30 m 的双码重叠区，以及 0.10/0.05/0.03 m 的小码识别与米制测距。0.5 m 处小码只有约 13 像素宽，默认角点下深度量化误差可能约 16%；更近时精度明显改善。这是正对平面的理想投影检查，不能替代实际 Gazebo 图像验证。

重启原有脚本即可加载新纹理，无需编译。观察 aruco.log 或 events.log 中的 `Switched` / `Switching to Inner` 和 `Targeting ArUco ID 18`。若 Gazebo 仍缓存旧纹理，应完整退出 gzclient/gzserver 再启动。

15 cm 是机体原点与标记的间距；相机光心距标记约 3 cm。2 cm 小码可以在理想居中视野中完整显示，但近距离横向容差很小，现有 5 cm XY 对齐阈值过宽。倾斜、横向偏差、渲染采样和丢码超时问题仍需后续优化。
