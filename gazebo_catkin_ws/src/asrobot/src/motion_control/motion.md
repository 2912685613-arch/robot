# `motion.cpp` 运动控制主文件学习说明

本文面向第一次阅读这个节点的开发者，介绍 [`motion.cpp`](motion.cpp) 的作用、每个函数的职责，以及一条控制命令是怎样变成机器人推进器输出的。说明以当前代码为准；文件中有几处函数暂时只有空实现，因此本文会明确区分“当前已经做了什么”和“设计上预留做什么”。

## 1. 一句话概括

[`motion.cpp`](motion.cpp) 是一个 ROS 运动控制节点。它负责：

1. 接收传感器/状态反馈（`mc_message`）；
2. 接收用户或上层决策发送的文字命令（`cmd_message`）；
3. 根据当前运动模式选择步态控制、PID、ADRC 或调试控制器；
4. 通过控制器和矢量推力分配，把位置误差转换为推进器速度/PWM和腿部动作；
5. 发布调试信息，并定时保存数据。

它更像“控制总调度器”，而不是所有算法的实现文件。具体的 PID、ADRC、步态、推进器分配等代码主要由 [`f_single_robot_control.h`](../../include/asrobot/f_single_robot_control.h) 引入的其他类和源文件实现。

---

## 2. 需要先理解的输入、处理和输出

可以把节点看成下面这条流水线：

```text
mc_message  ──> mccallback()
                 │ 更新当前位置、速度、姿态和相机数据
                 ▼
             当前状态机 g_current_state
                 │ 选择一个 handle_*() 函数
                 ▼
       PID / ADRC / 步态控制器计算控制量
                 ▼
       vec_ctrl 或 combine3DPidCtrl 做推力分配
                 ▼
       pwm_message / status_message（由底层控制类发布）

cmd_message ──> cmdcallback()
                 │ 解析例如 "xyz 1 2 -0.5"
                 ▼
       切换状态并写入目标值 exp_*
```

### 2.1 主要 ROS 接口

在 [`main()`](motion.cpp:454) 中可以看到当前实际启用的接口：

| 类型 | 名称 | 消息类型 | 用途 |
|---|---|---|---|
| 发布 | `pwm_message` | `asrobot::pwm` | 推进器 PWM/输出信息，通常由底层控制类使用或转发 |
| 发布 | `status_message` | `asrobot::status` | 舵机或机器人状态 |
| 发布 | `motion_debug_topic` | `asrobot::mc` | 调参和监视用的调试信息 |
| 订阅 | `mc_message` | `asrobot::mc` | 当前位姿、速度、姿态和视觉数据 |
| 订阅 | `cmd_message` | `std_msgs::String` | 文本控制命令 |
| 定时器 | 每 0.1 秒 | `ros::TimerEvent` | 保存调试数据 |

轨迹和多机器人位置订阅目前被注释掉了，虽然对应的回调函数仍保留。

---

## 3. 全局变量和状态机

### 3.1 ROS 发布器和消息

文件开头的 [`pub_pwm`](motion.cpp:8)、[`pub_status`](motion.cpp:9)、[`mc_pub_debug`](motion.cpp:14) 是发布器。`extern_n`、`extern_pub_pwm` 和 `extern_pub_status` 是指针形式的兼容接口，方便旧代码继续访问当前节点的 ROS 对象。

全局消息 [`pwm_msg`](motion.cpp:19)、[`key`](motion.cpp:20) 和 [`motion_debug`](motion.cpp:21) 用于保存待发布的消息或调试消息。

### 3.2 订阅器

[`sub_depth`](motion.cpp:25) 实际订阅的是 `mc_message`，名称虽然叫 depth，但消息中不只有深度；它包含完整的运动反馈。`sub_cmd` 订阅文字命令。`sub_trajectory` 和 `sub_multiRobotPosition` 是预留的轨迹/编队接口。

### 3.3 `RobotMode` 状态机

枚举 [`RobotMode`](motion.cpp:33) 表示“机器人现在要做什么”：

| 状态 | 含义 | 使用的处理函数 |
|---|---|---|
| `IDLE` | 空闲/停止 | [`handle_idle()`](motion.cpp:79) |
| `GAIT_CONTROL` | 步态控制 | [`handle_gait_control()`](motion.cpp:86) |
| `MANUAL_YAW` | 只控制航向角 | [`handle_manual_yaw()`](motion.cpp:91) |
| `MANUAL_DEPTH` | 只控制深度/`Z`位置 | [`handle_manual_depth()`](motion.cpp:101) |
| `MANUAL_DEPTH_YAW` | 同时控制深度和航向 | [`handle_manual_depthyaw()`](motion.cpp:111) |
| `MANUAL_XYZ_TARGET` | 控制三维位置 | [`handle_xyz_target()`](motion.cpp:124) |
| `DOCKING_ARUCO` | 根据 ArUco 视觉目标对接 | [`handle_docking_aruco()`](motion.cpp:140) |
| `GRASP` | 移动到抓取目标 | [`handle_grasp_target()`](motion.cpp:135) |
| `DEBUGVZ` | 调试深度速度内环 | [`handle_debug_vz()`](motion.cpp:193) |
| `DEBUGPZ` | 调试深度位置外环 | [`handle_debug_pz()`](motion.cpp:202) |

初始状态是 [`g_current_state`](motion.cpp:73) 的 `IDLE`。状态机的好处是：收到同一份传感器数据时，只执行当前模式对应的控制逻辑，避免多个控制器同时抢着控制推进器。

---

## 4. 日志函数

### [`print_header()`](motion.cpp:50)

这个函数只负责打印格式化的日志标题，不参与运动计算。

工作步骤：

1. 创建 60 个 `=` 组成的边框；
2. 如果 `is_state_change` 为 `true`，在标题前添加 `STATE CHANGE:`；
3. 计算左右空格，让标题大致居中；
4. 用 [`ROS_INFO_STREAM()`](motion.cpp:69) 一次性输出。

例如切换到 XYZ 模式时，终端会看到醒目的状态切换信息，便于判断命令是否生效。

---

## 5. 各个运动处理函数

这些函数由 [`mccallback()`](motion.cpp:212) 调用。它们通常不自己读取 ROS 消息，而是读取已经被回调写入的全局目标值 `exp_*` 和当前值 `cur_*`。

### [`handle_idle()`](motion.cpp:79)

调用 [`vec_ctrl.thrusterforce2speed()`](motion.cpp:83) 把四个推进器的输入设为零，确保机器人停止。急停命令 `st` 还会额外清零 ADRC 内部变量、停止步态并复位腿角度，见 [`cmdcallback()`](motion.cpp:336)。

### [`handle_gait_control()`](motion.cpp:86)

调用 [`gaitControl()`](motion.cpp:88)。步态控制通常用于协调机器人腿部的周期运动；这里的主文件只负责调度，具体腿部时序和角度计算在步态控制类中。

### [`handle_manual_yaw()`](motion.cpp:91)

只控制航向角：

1. 每秒打印一次目标航向 `exp_yaw` 和实际航向 `cur_yaw`；
2. 把组合控制器切换到 `DEBUG_YAW_ONLY`；
3. 调用 [`PidYawControl_H()`](motion.cpp:98) 计算航向控制输出。

这里的 `H` 表示采用 H 型推进/运动配置，最终输出如何分配到推进器由底层控制器完成。

### [`handle_manual_depth()`](motion.cpp:101)

只控制 `Z` 方向位置。它把控制模式设置为 `NAV_NORMAL`，然后调用 [`PidDepthControl_X()`](motion.cpp:108)。函数名中的 `X`/`H` 是项目历史命名，实际含义应结合对应控制器实现理解，不一定表示世界坐标的 X 轴。

### [`handle_manual_depthyaw()`](motion.cpp:111)

同时控制深度和航向：

```text
深度目标 exp_p_z ─┐
深度速度 exp_v_z ─┼─> PidDepthYawControl_H() ─> 推力输出
航向目标 exp_yaw ─┘
```

它使用 `DEBUG_DEPYAW_ONLY` 调试模式，并把深度位置、深度速度、航向位置以及三者的实际值传给 [`PidDepthYawControl_H()`](motion.cpp:121)。

### [`handle_xyz_target()`](motion.cpp:124)

输出目标是 `(X,Y,Z)` 三维位置。控制器会根据当前位置误差计算水平推力、深度推力和所需航向。代码注释中的“Computed Target Yaw”说明目标航向可能由目标方向自动推导，而不完全由命令直接给出。核心调用是 [`PidXYZControl_X()`](motion.cpp:132)。

### [`handle_grasp_target()`](motion.cpp:135)

调用 [`ADRCPosition3DControl_G()`](motion.cpp:137)，使用 ADRC 位置控制器移动到抓取目标。目标包含 `X`、`Y`、`Z` 和航向，当前值来自 `mc_message`。

### [`handle_docking_aruco()`](motion.cpp:140)

这是视觉对接控制入口，读取相机/ArUco 坐标 `cam_*`。当前代码意图分两阶段：

1. **XY 对齐**：调用 [`PidXYZControl_X()`](motion.cpp:162)，让视觉测得的 `X/Y` 误差小于 `0.05`；
2. **Z/Yaw 对齐**：调用带 `cur_v_z` 参数的 [`PidXYZControl_X()`](motion.cpp:186)，进一步控制深度和航向。

达到第一阶段阈值后会重置速度 PID，避免上一阶段的积分项影响第二阶段。

**当前实现需要注意：** `flag_xy` 和 `flag_z` 定义在函数内部，每次收到消息都会重新变成 `true` 和 `false`。因此，阶段切换不会跨回调保存，下一次回调又会回到阶段 1。若要实现真正的两阶段流程，应将阶段变量改成全局变量、类成员，或使用持久化的静态变量，并在停止/重新对接时复位。

### [`handle_debug_vz()`](motion.cpp:193)

用于调试深度双环控制的速度内环。它连续调用两次 [`combine3DPidCtrl.updateControl()`](motion.cpp:195)：第一次类似清除/更新控制状态，第二次给定 `exp_p_z` 作为深度目标。具体参数含义取决于 `C_Combine3DPidControl` 的定义。

### [`handle_debug_pz()`](motion.cpp:202)

用于调试深度位置外环。它给出深度目标 `exp_p_z` 和实际 `cur_p_z`，同时保留当前航向和速度反馈，调用 [`updateControl()`](motion.cpp:204) 观察外环调节效果。

---

## 6. 两个核心回调

### [`mccallback()`](motion.cpp:212)：反馈驱动控制

这是实际执行运动控制的核心入口。收到 `mc_message` 后，它先把消息字段复制到全局反馈变量：

- `c_x/c_y/c_z` → `cur_p_x/cur_p_y/cur_p_z`：当前位置；
- `c_speedx/c_speedy/c_speedz` → `cur_v_x/cur_v_y/cur_v_z`：速度；
- `c_navyaw/c_navroll/c_navpitch` → 当前姿态；
- `cam_e_*`、`cam_c_*` → 相机期望值和相机当前值。

然后通过 [`switch (g_current_state)`](motion.cpp:238) 分派到一个处理函数。也就是说，控制器是**事件驱动**的：每收到一条 `mc_message` 就计算一次控制输出，而不是由 `while` 循环直接调用这些控制函数。

### [`cmdcallback()`](motion.cpp:277)：命令驱动状态切换

它把字符串按空格拆成 `std::vector<std::string>`，第一个单词是命令名，后面是参数。例如 `xyz 1.0 2.0 -0.5` 会变成四个字符串。之后通过一长串 `if / else if` 判断命令，并修改状态和目标变量。

#### 常用命令表

| 命令 | 示例 | 作用 |
|---|---|---|
| `gait` | `gait 0.5` | 进入步态控制，设置半径/参数 `gait_R` |
| `yaw` | `yaw 90` | 目标航向 90 度 |
| `dep` | `dep -0.8` | 目标深度/Z 位置 |
| `depyaw` | `depyaw -0.8 90` | 同时设置深度和航向 |
| `xyz` | `xyz 1 2 -0.8` | 设置三维位置目标 |
| `dock` | `dock` | 进入 ArUco 对接模式 |
| `grasp` | `grasp 1 2 -0.5` | 进入抓取目标模式，并自动计算目标航向 |
| `st` | `st` | 急停并回到 `IDLE` |
| `a` | `a 10 20 30` | 设置腿部角度 |
| `vz` | `vz` | 调试深度速度内环 |
| `pz` | `pz -0.8` | 调试深度位置外环 |

PID 参数命令 `pidspeed`、`pidyaw`、`piddep`、`pidx`、`pidy` 和双环调参命令 `pidvz`、`pidpz` 都要求三个参数 `P I D`，再调用相应的参数设置函数。

**输入安全提示：** `grasp` 和 `a` 当前没有检查参数数量，参数不足时可能访问不存在的数组元素；`std::stod()` 遇到非数字也可能抛出异常。实际使用时应先检查 `cmd.size()`，并捕获转换异常。

---

## 7. 预留函数

以下函数目前是空函数体，只有接口占位：

- [`Spine_pathCallback()`](motion.cpp:445)：原计划接收多脊柱/参考轨迹；
- [`multiRobotPosition_Callback()`](motion.cpp:446)：原计划接收多机器人位置；
- [`reference_trajectory_to_file()`](motion.cpp:448)：原计划把参考轨迹写入文件；
- [`saveDataTimerCallback()`](motion.cpp:449)：原计划定时保存运行数据。

它们目前不会产生实际效果。尤其是定时器虽然在 [`main()`](motion.cpp:479) 中创建了，但回调为空，所以当前不会真正保存数据。

---

## 8. [`main()`](motion.cpp:454) 启动过程

主函数按以下顺序初始化：

1. [`ros::init()`](motion.cpp:457) 注册 ROS 节点，节点名为 `motion`；
2. 创建 `NodeHandle` 和控制循环频率；
3. 创建三个发布器；
4. 创建 `mc_message` 和 `cmd_message` 订阅器；
5. 创建 0.1 秒一次的数据保存定时器；
6. 调用 [`simMotorForce()`](motion.cpp:482) 做初始输出；
7. 设置航向、深度、俯仰、速度、X/Y 等 PID 参数；
8. 设置 X/Y/Z/Yaw 的 ADRC 参数；
9. 调用 [`LegShape_X()`](motion.cpp:497) 选择 X 型腿/推进配置；
10. 进入 ROS 主循环。

循环体只有三件事：

- [`combine3DPidCtrl.debug_msg()`](motion.cpp:504)：发布调试数据；
- [`ros::spinOnce()`](motion.cpp:506)：处理一次已经到达的订阅消息和定时器事件；
- [`loop_rate.sleep()`](motion.cpp:507)：按 `iMotionLoopRate` 控制循环频率休眠。

需要区分两种频率：主循环由 `iMotionLoopRate` 决定，而真正的 PID 处理发生在 `mccallback()` 中，因此控制计算频率还受 `mc_message` 发布频率影响。

---

## 9. PID 和 ADRC 的直观工作原理

### PID

以深度控制为例：

```text
误差 e = 目标深度 - 当前深度
控制输出 = Kp * e + Ki * 误差累积 + Kd * 误差变化速度
```

- `P`：当前偏差越大，修正越强；
- `I`：消除长期的小偏差，但过大可能造成积分饱和；
- `D`：抑制快速变化和超调，但对噪声敏感。

代码先通过 [`setDepthPIDPmt()`](motion.cpp:486) 等函数设置参数，再由 `Pid*Control*()` 函数在每次反馈到来时计算输出。

### ADRC

ADRC（自抗扰控制）不只看位置误差，还会估计系统受到的未知扰动，例如水流、阻力和模型误差，然后主动补偿。这里的 `setPxADRCPmt()`、`setPyADRCPmt()`、`setPzADRCPmt()` 和 `setYawADRCPmt()` 在 [`main()`](motion.cpp:492) 中完成参数初始化。抓取模式使用的 [`ADRCPosition3DControl_G()`](motion.cpp:137) 就属于这一类。

### 双环控制

深度双环通常是：

```text
位置误差 ─> 外环：生成期望速度 ─> 内环：生成推力 ─> 机器人
    ▲                                      │
    └────────────── 深度/速度反馈 ──────────┘
```

`DEBUGVZ` 主要观察速度内环，`DEBUGPZ` 主要观察位置外环；`pidvz` 和 `pidpz` 分别修改两层 PID 参数。

---

## 10. 推荐的阅读和调试顺序

1. 先看 [`RobotMode`](motion.cpp:33)，理解所有工作模式；
2. 再看 [`cmdcallback()`](motion.cpp:277)，掌握命令如何改变目标和状态；
3. 查看 [`mccallback()`](motion.cpp:212)，理解反馈如何进入控制器；
4. 选择一个简单模式，例如 [`handle_manual_yaw()`](motion.cpp:91)；
5. 沿着 `PidYawControl_H()` 继续进入控制器实现；
6. 最后阅读推进器分配、PWM发布和 Gazebo 接口。

调试时可以先发送 `st` 确保停止，再发送一个小幅度的 `yaw`、`dep` 或 `xyz` 命令，并观察 `motion_debug_topic`。修改 PID 前应记录原参数，逐步调整，避免机器人突然产生过大的推力。

## 11. 总结

[`motion.cpp`](motion.cpp) 的核心不是“实现某一个算法”，而是把 ROS 消息、命令、状态机和多个控制器连接起来：命令决定目标和模式，反馈提供实际状态，状态机选择控制算法，控制算法产生推力/腿部动作，底层模块再把这些结果发送给仿真或真实机器人。掌握 [`cmdcallback()`](motion.cpp:277) 和 [`mccallback()`](motion.cpp:212) 两条主线，就掌握了这个文件的大部分工作原理。
