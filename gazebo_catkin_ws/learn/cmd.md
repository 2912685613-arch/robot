# ASRobot 控制命令与完整控制链路

本文说明启动 `asrobot.launch` 后，输入下面四条命令时，命令会被谁接收、经过哪些 ROS 话题和 C++ 函数、最后如何作用到 Gazebo 中的机器人：

```text
xyz 1 1 -1
yaw 90
dep -1
st
```

本文以当前默认机器人为例：

```text
Gazebo 模型名：gazebo_robot1
Gazebo/ros_control 命名空间：/gazebo_robot1
ASRobot 应用命名空间：/robot1
```

如果启动时使用了其他参数，例如：

```bash
roslaunch asrobot asrobot.launch \
    namespace:=gazebo_robot2 \
    robot:=robot2 \
    model_name:=gazebo_robot2
```

那么文中的 `/gazebo_robot1` 和 `/robot1` 分别替换为 `/gazebo_robot2` 和 `/robot2`。

---

## 1. 启动顺序

完整仿真通常需要以下几层：

```text
1. Gazebo 世界
2. Gazebo 机器人模型
3. gazebo_ros_control 和关节/推进器控制器
4. asrobot.launch 上层控制程序
```

例如：

```bash
# 运行当前项目的世界、模型和底层控制器脚本
bash src/gazebo_launch/five_robots.sh

# 另开终端，启动 ASRobot 上层控制节点
source ~/robot/gazebo_catkin_ws/devel/setup.bash
roslaunch asrobot asrobot.launch
```

当前 `five_robots.sh` 实际启用的是 `gazebo_robot1`，因此 `asrobot.launch` 的默认参数正好对应它。

### 1.1 `asrobot.launch` 启动的主要节点

默认情况下，`asrobot.launch` 启动：

```text
/decision_robot1
/joint_control_robot1
/thruster_control_robot1
/robot1/motion
/robot1/mancontrol
/aruco_det_robot1
/aruco_posture_robot1
```

其中与本文四条命令直接相关的节点是：

| 节点 | 作用 |
|---|---|
| `/robot1/mancontrol` | 从终端读取字符串并发布控制命令 |
| `/decision_robot1` | 读取 Gazebo 状态和传感器数据，发布 `mc_message` |
| `/robot1/motion` | 解析命令、保存目标，依据当前状态计算控制输出 |
| `/joint_control_robot1` | 将关节控制量发送给 12 个关节位置控制器 |
| `/thruster_control_robot1` | 将推进器控制量发送给 Gazebo 推进器插件 |

`aruco_det` 和 `aruco_posture` 主要用于视觉检测/对接，不是 `xyz`、`yaw`、`dep`、`st` 这四条基础命令的必经节点。

---

## 2. 总体控制链路

先看整个系统的主链路：

```text
终端输入
    │
    │ 例如：xyz 1 1 -1
    ▼
/robot1/mancontrol
    │
    │ 发布 std_msgs/String
    ▼
/robot1/cmd_message
    │
    ├──────────────────────────────┐
    │                              │
    ▼                              ▼
/robot1/motion                 /decision_robot1
    │                              │
    │ motion::cmdcallback()       │ decision::cmdcallback()
    │ 保存目标并切换状态           │ 处理多机器人/编队专用命令
    │                              │
    │                              │ decision 主循环获取当前状态
    │                              │ 并调用 motion_control_pub()
    │                              ▼
    │                         /robot1/mc_message
    │                              │
    └──────────────┬───────────────┘
                   ▼
        motion::mccallback()
                   │
                   │ 根据 g_current_state 选择控制函数
                   ▼
          PID/ADRC/控制分配算法
                   │
                   │ 发布 asrobot/pwm
                   ▼
          /robot1/pwm_message
                   │
          ┌────────┴────────┐
          ▼                 ▼
  joint_control       thruster_control
          │                 │
          │ Float64         │ Float64
          ▼                 ▼
/gazebo_robot1/       /gazebo_robot1/
jointN_position_      thrusters/N/input
controller/command           │
          │                 │
          ▼                 ▼
   ros_control PID       UUV 推进器插件
          │                 │
          └────────┬────────┘
                   ▼
             Gazebo 物理引擎
                   ▼
             机器人运动
```

这里有一个非常重要的区别：

```text
cmd_message  = 目标/模式命令
mc_message   = 当前状态和期望状态的控制反馈消息
pwm_message  = 运动控制算法计算出的执行器输出
```

输入 `xyz 1 1 -1` 并不会直接把机器人瞬移到 `(1, 1, -1)`。它只是设置目标；之后控制器反复读取当前状态，计算误差，再输出推进器力和关节命令。

---

## 3. 命令是如何进入 ROS 的

### 3.1 `mancontrol.cpp` 读取终端输入

源码文件：

```text
src/asrobot/src/man_control/mancontrol.cpp
```

关键代码：

```cpp
ros::Publisher chatter_pub =
    n.advertise<std_msgs::String>("cmd_message", 1000);

while (ros::ok())
{
    std_msgs::String msg;
    cout << "please input cmd!" << endl;
    getline(cin, msg.data);
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
}
```

`mancontrol` 位于 `asrobot.launch` 的：

```xml
<group ns="$(arg robot)">
```

默认：

```text
robot = robot1
```

因此源码中的相对话题：

```text
cmd_message
```

最终解析为：

```text
/robot1/cmd_message
```

输入：

```text
xyz 1 1 -1
```

发布的消息类型为：

```text
std_msgs/String
```

消息内容为：

```yaml
data: "xyz 1 1 -1"
```

### 3.2 也可以不用 mancontrol，直接发布

如果不想在 `mancontrol` 终端输入，可以直接执行：

```bash
rostopic pub -1 \
  /robot1/cmd_message \
  std_msgs/String \
  "data: 'xyz 1 1 -1'"
```

其它命令：

```bash
rostopic pub -1 /robot1/cmd_message std_msgs/String "data: 'yaw 90'"
rostopic pub -1 /robot1/cmd_message std_msgs/String "data: 'dep -1'"
rostopic pub -1 /robot1/cmd_message std_msgs/String "data: 'st'"
```

`-1` 表示只发布一次。

验证命令是否进入 ROS：

```bash
rostopic echo /robot1/cmd_message
```

然后在 `mancontrol` 终端输入：

```text
xyz 1 1 -1
```

应看到：

```yaml
data: "xyz 1 1 -1"
```

---

## 4. 谁接收 `/robot1/cmd_message`

当前有两个节点订阅这个话题：

```text
/robot1/motion
/decision_robot1
```

### 4.1 `motion` 订阅

源码文件：

```text
src/asrobot/src/motion_control/motion.cpp
```

`main()` 中：

```cpp
sub_cmd = n.subscribe("cmd_message", 10, cmdcallback);
```

因为 `motion` 位于 `/robot1` 命名空间，所以最终订阅：

```text
/robot1/cmd_message
```

回调函数是：

```cpp
void cmdcallback(const std_msgs::String::ConstPtr &msg)
```

这是四条基础命令真正进入运动控制状态机的入口。

### 4.2 `decision` 订阅

源码文件：

```text
src/asrobot/src/decision_center/decision.cpp
```

`main()` 中：

```cpp
sub_cmd = n.subscribe("cmd_message", 10, cmdcallback);
```

Launch 中显式重映射为：

```xml
<remap from="cmd_message" to="$(arg robot)/cmd_message"/>
```

所以 `decision` 也订阅：

```text
/robot1/cmd_message
```

但是要注意：当前 `decision::cmdcallback()` 主要处理：

```text
cxyz
pnt
position
fm
docking
tracking
```

它并不负责 `xyz`、`yaw`、`dep` 和 `st` 这四条基础运动命令的状态切换。四条基础命令主要由 `motion::cmdcallback()` 处理。

可以查看当前连接关系：

```bash
rostopic info /robot1/cmd_message
```

应能看到两个订阅者，通常类似：

```text
Subscribers:
 * /robot1/motion
 * /decision_robot1
```

---

## 5. `motion` 如何处理字符串

`motion.cpp` 中先把字符串按空格拆分：

```cpp
std::stringstream ss(msg->data);
std::string buf;
std::vector<std::string> cmd;

while (ss >> buf)
{
    cmd.push_back(buf);
}
```

例如：

```text
xyz 1 1 -1
```

会变成：

```text
cmd[0] = "xyz"
cmd[1] = "1"
cmd[2] = "1"
cmd[3] = "-1"
```

然后通过一系列 `if / else if` 判断命令类型。

重要的是：

```text
cmdcallback() 不直接计算推进器力，也不直接发布 pwm_message。
```

它主要完成两件事：

1. 修改 `g_current_state`；
2. 把用户输入的目标保存到全局变量，例如 `exp_p_x`、`exp_yaw`、`exp_p_z`。

真正的控制计算发生在之后收到 `mc_message` 时的 `mccallback()` 中。

---

# 6. 命令一：`xyz 1 1 -1`

## 6.1 命令含义

```text
xyz x y z
```

当前命令：

```text
xyz 1 1 -1
```

表示设置目标位置：

```text
目标 x = 1
目标 y = 1
目标 z = -1
```

单位通常是米，具体坐标意义由 Gazebo 世界坐标系决定。

在水下仿真中，`z=-1` 通常表示世界坐标系中低于 `z=0` 的位置约 1 米。

## 6.2 调用链

```text
mancontrol.cpp
  getline()
  chatter_pub.publish()
      ↓
/robot1/cmd_message
      ↓
motion.cpp::cmdcallback()
      ↓
cmd[0] == "xyz"
      ↓
g_current_state = RobotMode::MANUAL_XYZ_TARGET
exp_p_x = 1
exp_p_y = 1
exp_p_z = -1
```

对应源码：

```cpp
else if (cmd[0] == "xyz" && cmd.size() == 4)
{
    g_current_state = RobotMode::MANUAL_XYZ_TARGET;
    exp_p_x = std::stod(cmd[1]);
    exp_p_y = std::stod(cmd[2]);
    exp_p_z = std::stod(cmd[3]);
    print_header("-> XYZ TARGET", true);
}
```

## 6.3 设置目标后谁继续工作

`decision` 节点持续读取：

```text
/gazebo/model_states
/gazebo_robot1/imu
/gazebo_robot1/pressure
```

其中 `modelStatesCallback()` 会在：

```text
src/asrobot/src/decision_center/decision.cpp
```

中查找：

```cpp
if (msg->name[i] == robot_name)
```

当前 `robot_name` 由 Launch 设置为：

```text
/gazebo_robot1/decision_robot1/robot_name = gazebo_robot1
```

于是它从 Gazebo 的所有模型中取出 `gazebo_robot1` 的：

- 位置 `pose.position`；
- 速度 `twist.linear`；
- 当前姿态等状态。

随后 `decision` 主循环中的 `motion_control_pub()` 组装 `asrobot::mc` 消息，并执行：

```cpp
mc_pub.publish(motion);
```

最终发布：

```text
/robot1/mc_message
```

## 6.4 `motion` 收到当前状态后执行 XYZ 控制

`motion.cpp` 中：

```cpp
sub_depth = n.subscribe("mc_message", 100, mccallback);
```

由于 `motion` 在 `/robot1` 命名空间，所以订阅：

```text
/robot1/mc_message
```

收到消息后调用：

```cpp
motion.cpp::mccallback()
```

它先更新当前状态：

```cpp
cur_p_x = msg->c_x;
cur_p_y = msg->c_y;
cur_p_z = msg->c_z;
cur_v_x = msg->c_speedx;
cur_v_y = msg->c_speedy;
cur_v_z = msg->c_speedz;
cur_yaw = msg->c_navyaw;
```

然后根据：

```cpp
g_current_state == RobotMode::MANUAL_XYZ_TARGET
```

调用：

```cpp
handle_xyz_target();
```

函数内部调用：

```cpp
PidXYZControl_X(
    exp_p_x, exp_p_y, exp_p_z,
    cur_p_x, cur_p_y, cur_p_z,
    cur_yaw);
```

也就是：

```text
目标位置：(exp_p_x, exp_p_y, exp_p_z) = (1, 1, -1)
当前位置：(cur_p_x, cur_p_y, cur_p_z)
当前航向：cur_yaw
```

控制器计算位置误差，例如：

```text
误差 x = 目标 x - 当前 x
误差 y = 目标 y - 当前 y
误差 z = 目标 z - 当前 z
```

然后结合当前航向，将世界坐标系目标力转换到机器人坐标系，并通过控制分配计算推进器输出。

最终 `motion` 发布：

```text
/robot1/pwm_message
```

消息类型为：

```text
asrobot/pwm
```

## 6.5 `xyz` 最终如何让机器人运动

`/robot1/pwm_message` 同时被两个执行器转换节点使用：

### 推进器通道

```text
/robot1/pwm_message
    ↓
thruster_control_robot1
    ↓
/gazebo_robot1/thrusters/0/input
/gazebo_robot1/thrusters/1/input
/gazebo_robot1/thrusters/2/input
/gazebo_robot1/thrusters/3/input
    ↓
UUV Gazebo 推进器插件
    ↓
机器人产生力和力矩
```

### 关节通道

```text
/robot1/pwm_message
    ↓
joint_control_robot1
    ↓
/gazebo_robot1/jointN_position_controller/command
    ↓
JointPositionController PID
    ↓
Gazebo 中的 H1、M1、V1、...、V4 关节
```

只要当前误差还没有收敛，`decision` 就会继续提供状态，`motion` 就会继续执行控制计算，机器人会持续修正位置。

---

# 7. 命令二：`yaw 90`

## 7.1 命令含义

```text
yaw 90
```

表示把目标航向设置为 `90`。当前代码按角度使用这个值，后续控制函数会按项目内部约定处理角度单位。

## 7.2 调用链

```text
mancontrol.cpp
    ↓
/robot1/cmd_message
    ↓
motion.cpp::cmdcallback()
    ↓
cmd[0] == "yaw"
    ↓
g_current_state = RobotMode::MANUAL_YAW
exp_yaw = 90
```

对应源码：

```cpp
else if (cmd[0] == "yaw" && cmd.size() == 2)
{
    g_current_state = RobotMode::MANUAL_YAW;
    exp_yaw = std::stod(cmd[1]);
    print_header("-> MANUAL YAW", true);
}
```

## 7.3 `motion` 收到反馈后调用的函数

下一条 `/robot1/mc_message` 到达时：

```text
motion.cpp::mccallback()
    ↓
case RobotMode::MANUAL_YAW
    ↓
handle_manual_yaw()
    ↓
PidYawControl_H(exp_yaw, cur_yaw)
```

代码为：

```cpp
void handle_manual_yaw()
{
    combine3DPidCtrl.setDebugMode(DEBUG_YAW_ONLY, exp_yaw);
    PidYawControl_H(exp_yaw, cur_yaw);
}
```

这里：

```text
目标航向 = exp_yaw = 90
当前航向 = cur_yaw
```

控制器根据航向误差生成绕 Z 轴的控制力矩：

```text
航向误差 = 目标航向 - 当前航向
```

之后仍然通过：

```text
motion → /robot1/pwm_message
       → thruster_control
       → Gazebo 推进器
```

实现旋转。

`yaw` 命令不会调用 `PidXYZControl_X()`，而是进入专门的 `handle_manual_yaw()` 分支。

---

# 8. 命令三：`dep -1`

## 8.1 命令含义

当前 `motion.cpp` 真正实现的普通深度命令是：

```text
dep -1
```

表示设置目标 Z 坐标：

```text
exp_p_z = -1
```

项目部分旧文档曾使用：

```text
pz -1
```

但当前代码中的 `pz` 是深度 PID 调试模式命令，不建议把它当作普通深度控制命令。普通控制请使用：

```text
dep -1
```

## 8.2 调用链

```text
mancontrol.cpp
    ↓
/robot1/cmd_message
    ↓
motion.cpp::cmdcallback()
    ↓
cmd[0] == "dep"
    ↓
g_current_state = RobotMode::MANUAL_DEPTH
exp_p_z = -1
```

对应源码：

```cpp
else if (cmd[0] == "dep" && cmd.size() == 2)
{
    g_current_state = RobotMode::MANUAL_DEPTH;
    exp_p_z = std::stod(cmd[1]);
    print_header("-> MANUAL DEPTH", true);
}
```

## 8.3 `motion` 收到反馈后调用的函数

下一条 `/robot1/mc_message` 到达时：

```text
motion.cpp::mccallback()
    ↓
case RobotMode::MANUAL_DEPTH
    ↓
handle_manual_depth()
    ↓
PidDepthControl_X(exp_p_z, cur_p_z)
```

对应代码：

```cpp
void handle_manual_depth()
{
    combine3DPidCtrl.setDebugMode(NAV_NORMAL);
    PidDepthControl_X(exp_p_z, cur_p_z);
}
```

其中：

```text
目标深度 = exp_p_z = -1
当前深度 = cur_p_z
```

控制器根据深度误差计算竖直方向的力：

```text
深度误差 = 目标 z - 当前 z
```

之后通过 `pwm_message`、`thruster_control` 和 Gazebo 推进器插件产生上浮或下潜运动。

---

# 9. 命令四：`st`

## 9.1 命令含义

```text
st
```

表示停止当前运动控制，进入空闲状态。

## 9.2 调用链

```text
mancontrol.cpp
    ↓
/robot1/cmd_message
    ↓
motion.cpp::cmdcallback()
    ↓
cmd[0] == "st"
    ↓
g_current_state = RobotMode::IDLE
```

对应代码：

```cpp
else if (cmd[0] == "st")
{
    g_current_state = RobotMode::IDLE;
    vec_ctrl.thrusterforce2speed(0, 0, 0, 0);
    clearADRCSpeedControllerVariables();
    gait_stop();
    legSetAngle(0, 0, 0);
    print_header("-> IDLE (SYSTEM STOPPED)", true);
}
```

这里会立即执行一部分停止动作：

- 状态机切换到 `IDLE`；
- 推进器速度设置为 0；
- 清理 ADRC 速度控制变量；
- 停止步态控制；
- 将腿部角度设置为 0。

## 9.3 后续反馈如何保持停止

停止后，下一次收到 `/robot1/mc_message` 时，仍会进入：

```cpp
case RobotMode::IDLE:
    handle_idle();
```

`handle_idle()` 中会再次确保推进器保持停止：

```cpp
void handle_idle()
{
    vec_ctrl.thrusterforce2speed(0, 0, 0, 0);
}
```

因此 `st` 不是只停留一个瞬间，而是把运动控制状态保持在 `IDLE`，直到下一条运动命令重新切换状态。

注意：`st` 是控制层停止命令；它不删除 Gazebo 模型，也不关闭 ROS 节点，也不停止 Gazebo 世界。

---

# 10. `mc_message` 从哪里来

四条命令设置目标后，`motion` 需要知道机器人当前的位置、速度和姿态。

这些状态由 `decision` 组织成 `asrobot::mc` 消息。

## 10.1 `decision` 获取 Gazebo 当前状态

`decision.cpp` 订阅：

```cpp
model_states_sub = n.subscribe(
    "/gazebo/model_states", 10, modelStatesCallback);
```

回调函数：

```cpp
void modelStatesCallback(
    const gazebo_msgs::ModelStates::ConstPtr &msg)
```

它遍历 Gazebo 中的模型名称，并寻找：

```cpp
msg->name[i] == robot_name
```

`robot_name` 由 `asrobot.launch` 设置：

```xml
<param name="robot_name" value="$(arg model_name)"/>
```

默认值为：

```text
gazebo_robot1
```

找到后，保存：

```cpp
robot_pos(0, 0) = pose.position.x;
robot_pos(1, 0) = pose.position.y;
robot_pos(2, 0) = pose.position.z;
```

以及线速度等状态。

同时 `decision` 还订阅：

```text
/gazebo_robot1/imu
/gazebo_robot1/pressure
```

获取姿态和深度相关信息。

## 10.2 `decision` 发布 `mc_message`

`decision` 在 `motion_control_pub()` 中组装：

```text
asrobot::mc motion
```

并执行：

```cpp
mc_pub.publish(motion);
```

由于 `mc_pub` 使用相对话题名：

```cpp
mc_pub = n.advertise<asrobot::mc>("mc_message", 10);
```

而 `decision` 的 `cmd_message`、`mc_message` 等接口被 Launch 映射到 `robot1` 命名空间，所以最终控制节点使用：

```text
/robot1/mc_message
```

`mc` 消息中重要的当前状态字段包括：

```text
c_x, c_y, c_z
c_speedx, c_speedy, c_speedz
c_navyaw, c_navpitch, c_navroll
```

`motion::mccallback()` 将它们复制到：

```cpp
cur_p_x, cur_p_y, cur_p_z
cur_v_x, cur_v_y, cur_v_z
cur_yaw, cur_pitch, cur_roll
```

---

# 11. `motion` 如何把误差变成控制输出

`motion` 的主函数在：

```text
src/asrobot/src/motion_control/motion.cpp::main()
```

启动时创建：

```cpp
pub_pwm = n.advertise<asrobot::pwm>("pwm_message", 10);
pub_status = n.advertise<asrobot::status>("status_message", 10);
mc_pub_debug = n.advertise<asrobot::mc>("motion_debug_topic", 10);
```

并订阅：

```cpp
sub_depth = n.subscribe("mc_message", 100, mccallback);
sub_cmd = n.subscribe("cmd_message", 10, cmdcallback);
```

主循环中：

```cpp
while (ros::ok())
{
    combine3DPidCtrl.debug_msg(mc_pub_debug);
    ros::spinOnce();
    loop_rate.sleep();
}
```

这里的关键点是：

```text
cmdcallback() 负责“设置模式和目标”
mccallback() 负责“拿当前状态执行控制计算”
```

`mccallback()` 先更新当前反馈，再根据状态机调用：

| 状态 | 命令 | 调用函数 |
|---|---|---|
| `MANUAL_XYZ_TARGET` | `xyz x y z` | `handle_xyz_target()` → `PidXYZControl_X()` |
| `MANUAL_YAW` | `yaw yaw` | `handle_manual_yaw()` → `PidYawControl_H()` |
| `MANUAL_DEPTH` | `dep z` | `handle_manual_depth()` → `PidDepthControl_X()` |
| `MANUAL_DEPTH_YAW` | `depyaw z yaw` | `handle_manual_depthyaw()` → `PidDepthYawControl_H()` |
| `IDLE` | `st` | `handle_idle()` → 零推进器输出 |

控制参数在 `motion::main()` 中初始化，例如：

```cpp
setYawPIDPmt(0.025, 0.011, 0.06);
setDepthPIDPmt(1.25, 0.006, 0.02);
setSpeedPIDPmt(0.8, 0, 0.2);
setXPIDPmt(0.5, 0, 0.2);
setYPIDPmt(0.5, 0, 0.2);
```

具体控制器实现分散在：

```text
src/asrobot/src/motion_control/c_controller_group.cpp
src/asrobot/src/motion_control/c_vectoredthrustcontrol.cpp
src/asrobot/src/motion_control/c_gaitcontrol.cpp
src/asrobot/src/motion_control/MPCController.cpp
src/asrobot/include/asrobot/c_controller_group.h
src/asrobot/include/asrobot/c_vectoredthrustcontrol.h
```

不同控制函数最终都会更新控制输出，并通过 `pwm_message` 发给执行器转换节点。

---

# 12. `pwm_message` 如何到达 Gazebo

`motion` 发布消息：

```cpp
pub_pwm.publish(pwm_msg);
```

消息类型：

```text
asrobot/pwm
```

Launch 中：

```xml
<remap from="pwm_message" to="$(arg robot)/pwm_message"/>
```

所以最终是：

```text
/robot1/pwm_message
```

## 12.1 推进器路径

```text
/robot1/pwm_message
    ↓
thruster_control_robot1
    ↓
/gazebo_robot1/thrusters/0/input
/gazebo_robot1/thrusters/1/input
/gazebo_robot1/thrusters/2/input
/gazebo_robot1/thrusters/3/input
    ↓
UUV/Gazebo 推进器插件
    ↓
Gazebo 中的推进力和力矩
```

对应的 Launch 重映射包括：

```xml
<remap from="thrusters/0/input"
       to="$(arg namespace)/thrusters/0/input"/>
```

其它推进器编号同理。

## 12.2 关节路径

```text
/robot1/pwm_message
    ↓
joint_control_robot1
    ↓
/gazebo_robot1/joint1_position_controller/command
...
/gazebo_robot1/joint12_position_controller/command
    ↓
controller_manager 加载的 JointPositionController
    ↓
gazebo_ros_control
    ↓
Gazebo 中的 H1、M1、V1、...、V4 关节
```

`joint_control.cpp` 中创建了 12 个 `std_msgs/Float64` 发布器，例如：

```cpp
n.advertise<std_msgs::Float64>(
    "joint1_position_controller/command", 1000);
```

Launch 再将它们重映射到：

```text
/gazebo_robot1/joint1_position_controller/command
```

最终这些 Topic 由 `effort_controllers/JointPositionController` 接收，PID 计算关节 effort，Gazebo 物理引擎再更新关节运动。

---

# 13. 四条命令的对照表

| 输入命令 | `motion` 状态 | 保存的目标 | 控制函数 |
|---|---|---|---|
| `xyz 1 1 -1` | `MANUAL_XYZ_TARGET` | `exp_p_x=1, exp_p_y=1, exp_p_z=-1` | `PidXYZControl_X()` |
| `yaw 90` | `MANUAL_YAW` | `exp_yaw=90` | `PidYawControl_H()` |
| `dep -1` | `MANUAL_DEPTH` | `exp_p_z=-1` | `PidDepthControl_X()` |
| `st` | `IDLE` | 推进器目标清零 | `handle_idle()` / `thrusterforce2speed(0,0,0,0)` |

每个运动命令的共同后半段是：

```text
控制函数
    ↓
计算推进器/关节执行器输出
    ↓
asrobot::pwm
    ↓
/robot1/pwm_message
    ↓
joint_control 和 thruster_control
    ↓
Gazebo 控制器/插件
    ↓
机器人运动
```

---

# 14. 如何逐层验证控制链路

建议按下面顺序检查。

## 14.1 查看节点

```bash
rosnode list | grep -E 'decision|motion|control|mancontrol|aruco'
```

应看到类似：

```text
/decision_robot1
/joint_control_robot1
/thruster_control_robot1
/robot1/motion
/robot1/mancontrol
```

## 14.2 查看命令话题

```bash
rostopic info /robot1/cmd_message
```

检查：

```text
Publisher: /robot1/mancontrol
Subscribers: /robot1/motion, /decision_robot1
```

## 14.3 查看运动状态消息

```bash
rostopic info /robot1/mc_message
rostopic echo /robot1/mc_message
```

如果 `decision` 正常发布，可以看到 `c_x`、`c_y`、`c_z`、`c_navyaw` 等字段变化。

## 14.4 查看运动控制输出

```bash
rostopic echo /robot1/pwm_message
```

输入：

```text
xyz 0.2 0 -0.2
```

后，应看到 PWM/力/力矩等字段变化。

## 14.5 查看推进器输入

```bash
rostopic echo /gazebo_robot1/thrusters/0/input
rostopic echo /gazebo_robot1/thrusters/1/input
rostopic echo /gazebo_robot1/thrusters/2/input
rostopic echo /gazebo_robot1/thrusters/3/input
```

## 14.6 查看关节命令连接

```bash
rostopic info \
  /gazebo_robot1/joint1_position_controller/command
```

如果 `joint_control` 正常，应看到它作为发布者；Gazebo 中的关节位置控制器作为订阅者。

## 14.7 查看控制器状态

```bash
rosservice call \
  /gazebo_robot1/controller_manager/list_controllers
```

检查：

```text
joint1_position_controller ... state: running
```

---

# 15. 推荐的第一次安全测试

不要一开始就输入很远的目标。建议：

```text
xyz 0.2 0 -0.2
```

观察：

```bash
rostopic echo /robot1/pwm_message
```

确认输出正常后，再测试：

```text
yaw 20
```

再测试：

```text
dep -0.3
```

不希望继续运动时输入：

```text
st
```

如果用命令行发布，停止命令为：

```bash
rostopic pub -1 /robot1/cmd_message std_msgs/String "data: 'st'"
```

---

# 16. 常见误区

## 误区一：输入 `xyz` 后机器人会直接跳到目标点

不会。`xyz` 只保存目标，控制器必须等待 `mc_message` 提供当前位置，然后不断计算误差。

## 误区二：`mancontrol` 直接控制 Gazebo

不会。`mancontrol` 只发布字符串命令。真正的控制计算在 `motion` 中，真正的物理执行在 Gazebo 插件和控制器中。

## 误区三：`st` 会关闭 Gazebo

不会。`st` 只让运动控制器进入 `IDLE` 并把执行器输出置零。

## 误区四：只启动 `asrobot.launch` 就一定能动

不一定。还必须保证：

```text
Gazebo 已启动
机器人模型已生成
gazebo_ros_control 已加载
controller_manager 和底层控制器已运行
```

否则 `/robot1/pwm_message` 可能有数据，但 Gazebo 没有对应的推进器或关节接口。

## 误区五：`pz -1` 和 `dep -1` 完全相同

当前代码中：

```text
dep -1  = 普通手动深度控制
pz -1   = 深度位置调试模式
```

普通运动测试优先使用：

```text
dep -1
```

---

# 17. 一句话记忆整个链路

```text
输入字符串
→ /robot1/cmd_message
→ motion::cmdcallback 设置模式和目标
→ decision 发布 /robot1/mc_message 提供当前状态
→ motion::mccallback 调用 PID/ADRC 控制函数
→ 发布 /robot1/pwm_message
→ joint_control/thruster_control 转成关节和推进器命令
→ gazebo_ros_control/UUV 插件
→ Gazebo 物理引擎
→ 机器人运动
```
