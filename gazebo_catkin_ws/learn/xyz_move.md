# 输入 `xyz 1 1 -1` 后机器人为什么会移动

本文结合当前工程中的启动脚本和源码，完整追踪下面这条实际执行链路：

```text
bash src/gazebo_launch/single_water.sh
        │
        ├── Gazebo 水下世界
        ├── 机器人模型
        ├── Gazebo 控制器
        └── asrobot 控制程序
                │
                └── 在命令输入窗口输入：xyz 1 1 -1
```

最终的完整控制链是：

```text
键盘输入
  ↓
mancontrol
  ↓  /robot1/cmd_message
motion::cmdcallback()
  ↓
保存目标点 (1, 1, -1)，切换到 XYZ 控制状态
  ↓
decision 发布当前位姿 /robot1/mc_message
  ↓
motion::mccallback()
  ↓
PID 计算 X/Y/Z 控制力
  ↓
四个推进器力分配
  ↓
发布 /robot1/pwm_message
  ↓
thruster_control
  ↓
/gazebo_robot1/thrusters/*/input
  ↓
Gazebo 推进器插件
  ↓
机器人移动
```

理解这条链路时，要牢记一个核心事实：

> `xyz 1 1 -1` 本身不是“直接给推进器发送速度”，而是设置一个目标位置。后续程序不断读取机器人当前位置，根据“目标位置 - 当前位置”的误差计算推进器输出，这才形成闭环运动。

---

## 1. `single_water.sh` 启动了什么

脚本位于：

```text
src/gazebo_launch/single_water.sh
```

脚本依次启动四类程序：

```bash
# 启动 Gazebo 世界
gnome-terminal --title="world" -- bash -c \
  "source ~/robot/gazebo_catkin_ws/devel/setup.bash && \
   roslaunch gazebo_world simple_underwater.launch; exec bash" &

# 加载机器人模型
gnome-terminal --title="model" -- bash -c \
  "source ~/robot/gazebo_catkin_ws/devel/setup.bash && \
   roslaunch gazebo_model gazebo_model.launch x:=0 y:=0 z:=-0.1; exec bash" &

# 启动 Gazebo 控制器
gnome-terminal --title="control" -- bash -c \
  "source ~/robot/gazebo_catkin_ws/devel/setup.bash && \
   roslaunch gazebo_control gazebo_control.launch mode:=water; exec bash" &

# 启动机器人控制程序
gnome-terminal --title="robot_control" -- bash -c \
  "source ~/robot/gazebo_catkin_ws/devel/setup.bash && \
   roslaunch asrobot asrobot.launch; exec bash" &
```

前 3 个窗口负责仿真环境，最后一个 `robot_control` 窗口负责启动 `asrobot` 包中的控制节点。

`asrobot.launch` 主要启动：

```xml
<node pkg="asrobot" type="decision" ... />
<node pkg="asrobot" type="joint_control" ... />
<node pkg="asrobot" type="thruster_control" ... />

<group ns="$(arg robot)">
    <node pkg="asrobot" type="motion" ... />
    <node pkg="asrobot" type="mancontrol" ... />
</group>
```

默认参数是：

```xml
<arg name="namespace" default="gazebo_robot1"/>
<arg name="robot" default="robot1"/>
```

因此后面会出现两个重要的名字：

```text
/gazebo_robot1     Gazebo 中的机器人模型命名空间
/robot1            控制程序使用的 ROS 命名空间
```

这里首先要区分两件事：Gazebo 中的机器人叫 `gazebo_robot1`，而命令控制相关的 ROS 话题主要在 `robot1` 命名空间下。

**到这里，下一个问题是：命令输入窗口中的 `please input cmd!` 是哪个程序打印的？输入一行文字后，它又把文字发到哪里？**

---

## 2. `mancontrol` 接收键盘输入并发布字符串

命令输入窗口来自：

```text
src/asrobot/src/man_control/mancontrol.cpp
```

程序启动后创建一个字符串发布器：

```cpp
ros::Publisher chatter_pub =
    n.advertise<std_msgs::String>("cmd_message", 1000);
```

然后不断读取终端输入：

```cpp
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

当你输入：

```text
xyz 1 1 -1
```

执行过程是：

```cpp
getline(cin, msg.data);
```

此时：

```cpp
msg.data == "xyz 1 1 -1"
```

紧接着：

```cpp
chatter_pub.publish(msg);
```

因为 `mancontrol` 位于 `asrobot.launch` 的：

```xml
<group ns="robot1">
```

所以代码中的相对话题：

```text
cmd_message
```

实际名称是：

```text
/robot1/cmd_message
```

可以打开另一个终端验证：

```bash
rostopic echo /robot1/cmd_message
```

然后在命令窗口输入：

```text
xyz 1 1 -1
```

应该看到：

```yaml
data: "xyz 1 1 -1"
```

这说明命令已经从键盘进入 ROS 系统，但此时机器人还没有因为这条字符串而直接获得推进器速度。

**下一个问题是：谁订阅了 `/robot1/cmd_message`，又如何把字符串 `xyz 1 1 -1` 解释成目标位置？**

---

## 3. `motion` 节点解析 `xyz` 命令

真正负责执行运动控制的是：

```text
src/asrobot/src/motion_control/motion.cpp
```

在 `motion` 的 `main()` 中，代码订阅了两个核心话题：

```cpp
sub_depth = n.subscribe(
    "mc_message",
    100,
    mccallback
);

sub_cmd = n.subscribe(
    "cmd_message",
    10,
    cmdcallback
);
```

因为 `motion` 也位于 `robot1` 命名空间中，所以实际订阅的是：

```text
/robot1/mc_message
/robot1/cmd_message
```

输入的字符串会进入：

```cpp
void cmdcallback(const std_msgs::String::ConstPtr &msg)
```

函数首先使用字符串流按空格拆分命令：

```cpp
std::stringstream ss(msg->data);
std::string buf;
std::vector<std::string> cmd;

while (ss >> buf)
{
    cmd.push_back(buf);
}
```

对于：

```text
xyz 1 1 -1
```

拆分结果是：

```text
cmd[0] = "xyz"
cmd[1] = "1"
cmd[2] = "1"
cmd[3] = "-1"
```

随后匹配到 `xyz` 分支：

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

这段代码完成了两件事。

### 3.1 切换控制状态

```cpp
g_current_state = RobotMode::MANUAL_XYZ_TARGET;
```

这表示之后收到机器人状态反馈时，要执行 XYZ 目标控制。

### 3.2 保存目标位置

```cpp
exp_p_x = 1;
exp_p_y = 1;
exp_p_z = -1;
```

也就是把目标保存为：

```text
目标位置 = (1, 1, -1)
```

注意，此处没有直接调用：

```cpp
thrusterforce2speed(...)
```

也没有直接发布：

```text
pwm_message
```

`cmdcallback()` 只是改变了控制器的状态和目标变量。

**下一个问题是：既然命令回调只保存目标，控制器从哪里获得“机器人现在的位置”？要计算运动误差，为什么还需要另一个 `mc_message`？**

---

## 4. 为什么设置目标后还需要当前位姿

控制器不能只知道目标：

```text
目标 = (1, 1, -1)
```

它还必须知道机器人当前在哪里，例如：

```text
当前位置 = (0.2, 0.1, -0.3)
```

这样才能计算：

```text
X 误差 = 1 - 0.2
Y 误差 = 1 - 0.1
Z 误差 = -1 - (-0.3)
```

也就是说，机器人运动控制需要两类数据：

```text
期望状态：exp_p_x、exp_p_y、exp_p_z
当前状态：cur_p_x、cur_p_y、cur_p_z
```

在本项目中：

```text
xyz 命令负责设置期望状态
mc_message 负责提供当前状态
```

`motion` 在启动时订阅：

```cpp
sub_depth = n.subscribe("mc_message", 100, mccallback);
```

所以我们下一步要查清楚：`mc_message` 是谁发布的，以及它的当前位置数据从哪里来。

**下一个问题是：`decision` 节点如何从 Gazebo 获取 `gazebo_robot1` 的当前位姿，并封装成 `mc_message`？**

---

## 5. `decision` 从 Gazebo 获取机器人当前位姿

`asrobot.launch` 启动了：

```xml
<node pkg="asrobot" type="decision" name="decision_robot1" ... />
```

`decision` 的源码是：

```text
src/asrobot/src/decision_center/decision.cpp
```

它发布：

```cpp
mc_pub = n.advertise<asrobot::mc>("mc_message", 10);
```

通过 `asrobot.launch` 的重映射：

```xml
<remap from="mc_message" to="$(arg robot)/mc_message"/>
```

最终发布到：

```text
/robot1/mc_message
```

但 `decision` 需要先知道机器人当前的位姿。仿真模式下，它订阅了 Gazebo 的模型状态：

```cpp
model_states_sub = n.subscribe(
    "/gazebo/model_states",
    10,
    modelStatesCallback
);
```

回调函数是：

```cpp
void modelStatesCallback(
    const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name[i] == robot_name)
        {
            const geometry_msgs::Pose &pose = msg->pose[i];
            const geometry_msgs::Twist &twist = msg->twist[i];

            robot_pos(0, 0) = pose.position.x;
            robot_pos(1, 0) = pose.position.y;
            robot_pos(2, 0) = pose.position.z;

            robot_pos(0, 1) = twist.linear.x;
            robot_pos(1, 1) = twist.linear.y;
            robot_pos(2, 1) = twist.linear.z;

            break;
        }
    }
}
```

这里的 `robot_name` 默认是：

```text
gazebo_robot1
```

所以这段代码的含义是：

1. 遍历 Gazebo 中所有模型；
2. 找到名字为 `gazebo_robot1` 的模型；
3. 读取它的世界坐标位置；
4. 读取它的线速度；
5. 保存在 `robot_pos` 矩阵中。

数据来源关系是：

```text
Gazebo
  ↓ /gazebo/model_states
modelStatesCallback()
  ↓
robot_pos
```

除了 `/gazebo/model_states`，`decision` 还订阅了：

```cpp
dep_sub = n.subscribe("pressure", 10, depcallback);
imu_sub = n.subscribe("imu", 10, imucallback);
```

因此它还可以获得：

```text
pressure  → 深度
imu       → 姿态、航向角、速度等信息
```

在当前仿真调试模式下，`decision` 的主循环会调用：

```cpp
motion_control_pub(SINGLE_ROBOT_DEBUG);
```

这个函数将当前状态填入 `asrobot::mc` 消息。例如：

```cpp
motion.c_x = robot_pos(0, 0);
motion.c_y = robot_pos(1, 0);
motion.c_z = -depth;
```

然后发布：

```cpp
mc_pub.publish(motion);
```

于是形成：

```text
Gazebo 当前位姿
  ↓
robot_pos / depth / yaw
  ↓
组装 asrobot::mc
  ↓
发布 /robot1/mc_message
```

可以通过下面的命令观察：

```bash
rostopic echo /robot1/mc_message
```

重点看：

```yaml
c_x:
c_y:
c_z:
c_speedx:
c_speedy:
c_speedz:
c_navyaw:
```

**下一个问题是：`motion` 收到 `mc_message` 后，怎样把这些字段变成 `cur_p_x`、`cur_p_y`、`cur_p_z`，并决定执行 XYZ 控制？**

---

## 6. `motion::mccallback()` 接收当前状态并选择控制分支

`motion` 收到 `/robot1/mc_message` 后，进入：

```cpp
void mccallback(const asrobot::mc::ConstPtr &msg)
```

这个回调首先把消息中的当前状态复制到全局变量：

```cpp
cur_p_x = msg->c_x;
cur_v_x = msg->c_speedx;

cur_p_y = msg->c_y;
cur_v_y = msg->c_speedy;

cur_p_z = msg->c_z;
cur_v_z = msg->c_speedz;

cur_yaw = msg->c_navyaw;
cur_roll = msg->c_navroll;
cur_pitch = msg->c_navpitch;
```

现在 `motion` 里同时拥有：

```text
目标：exp_p_x = 1，exp_p_y = 1，exp_p_z = -1
当前：cur_p_x、cur_p_y、cur_p_z
状态：g_current_state = MANUAL_XYZ_TARGET
```

然后执行状态机：

```cpp
switch (g_current_state)
{
case RobotMode::IDLE:
    handle_idle();
    break;

case RobotMode::MANUAL_YAW:
    handle_manual_yaw();
    break;

case RobotMode::MANUAL_DEPTH:
    handle_manual_depth();
    break;

case RobotMode::MANUAL_XYZ_TARGET:
    handle_xyz_target();
    break;
}
```

由于刚才输入了 `xyz`，当前状态是：

```cpp
RobotMode::MANUAL_XYZ_TARGET
```

所以执行：

```cpp
handle_xyz_target();
```

这里就是“命令设置目标”与“反馈触发控制”连接起来的地方。

`xyz` 命令只需要输入一次，但只要 `decision` 持续发布新的 `mc_message`，`mccallback()` 就会不断执行，控制器也就不断重新计算推进器输出。

**下一个问题是：`handle_xyz_target()` 具体调用哪个 XYZ 控制函数？这个函数向控制器传入了哪些目标和当前状态？**

---

## 7. `handle_xyz_target()` 调用 `PidXYZControl_X()`

`motion.cpp` 中的函数是：

```cpp
void handle_xyz_target()
{
    ROS_INFO_STREAM_THROTTLE(
        1.0,
        "--- XYZ TARGET CONTROL ---\n"
        << "  [Target] X: " << exp_p_x
        << ", Y: " << exp_p_y
        << ", Z: " << exp_p_z
        << "\n"
        << "  [Actual] X: " << cur_p_x
        << ", Y: " << cur_p_y
        << ", Z: " << cur_p_z
    );

    PidXYZControl_X(
        exp_p_x,
        exp_p_y,
        exp_p_z,
        cur_p_x,
        cur_p_y,
        cur_p_z,
        cur_yaw
    );
}
```

它把两组数据传入 `PidXYZControl_X()`：

```text
期望位置：exp_p_x、exp_p_y、exp_p_z
当前状态：cur_p_x、cur_p_y、cur_p_z、cur_yaw
```

`PidXYZControl_X()` 位于：

```text
src/asrobot/src/motion_control/f_single_robot_control.cpp
```

代码如下：

```cpp
void PidXYZControl_X(
    float _exp_p_x,
    float _exp_p_y,
    float _exp_p_z,
    float _cur_p_x,
    float _cur_p_y,
    float _cur_p_z,
    float _cur_yaw)
{
    combine3DPidCtrl.setDebugMode(NAV_NORMAL);

    combine3DPidCtrl.updateControl(
        SHAPE_X,
        _exp_p_x,
        _exp_p_y,
        0,
        _exp_p_z,
        _cur_p_x,
        _cur_p_y,
        _cur_yaw,
        _cur_p_z
    );
}
```

这里有两个重要信息。

### 7.1 选择 X 型控制模式

```cpp
SHAPE_X
```

表示采用 X 型机器人推进器布局。

在 `motion.cpp` 启动时还会调用：

```cpp
LegShape_X();
```

对应：

```cpp
void LegShape_X()
{
    vec_ctrl.yawLocomMode_X();
    cout << " the leg shape is [X] !!!" << endl;
}
```

### 7.2 进入正常导航模式

```cpp
combine3DPidCtrl.setDebugMode(NAV_NORMAL);
```

表示不是单独调试深度、Yaw 或某一个轴，而是进行正常 XYZ 导航控制。

接着：

```cpp
combine3DPidCtrl.updateControl(...);
```

把目标和当前状态交给组合控制器。

**下一个问题是：`updateControl()` 内部如何从目标与当前位置计算出 X/Y/Z 控制力？**

---

## 8. 组合控制器通过 PID 计算 X/Y/Z 控制力

`updateControl()` 位于：

```text
src/asrobot/src/motion_control/c_sim_3d_ctrl.cpp
```

它先保存传入的数据：

```cpp
this->shape_ = shape;

this->exp_p_x_ = exp_p_x;
this->exp_p_y_ = exp_p_y;
this->exp_p_z_ = exp_p_z;

this->cur_p_x_ = cur_p_x;
this->cur_p_y_ = cur_p_y;
this->cur_p_z_ = cur_p_z;
this->cur_yaw_ = cur_yaw;
```

然后调用：

```cpp
this->updatePidControl();
```

`updatePidControl()` 根据机器人形态和控制模式选择具体函数：

```cpp
if (this->shape_ == SHAPE_X &&
    this->current_mode_ == SINGLE_LOOP)
{
    handleShapeX_SingleLoop_Control();
}
else if (this->shape_ == SHAPE_X &&
         this->current_mode_ == DUAL_LOOP)
{
    handleShapeX_DualLoop_Control();
}
```

当前 XYZ 控制的核心逻辑在：

```text
handleShapeX_SingleLoop_Control()
```

其中 `NAV_NORMAL` 分支首先计算位置误差：

```cpp
float x_error = exp_p_x_ - cur_p_x_;
float y_error = exp_p_y_ - cur_p_y_;
float depth_error = exp_p_z_ - cur_p_z_;
```

假设当前位姿是：

```text
当前 = (0.2, 0.1, -0.3)
```

则误差大约是：

```text
x_error     = 1 - 0.2  = 0.8
y_error     = 1 - 0.1  = 0.9
depth_error = -1 - (-0.3) = -0.7
```

随后分别使用 X、Y、深度 PID：

```cpp
float desired_world_x_force =
    vec_ctrl.mcXPID.pidNewCalc(x_error);

float desired_world_y_force =
    vec_ctrl.mcYPID.pidNewCalc(y_error);

float desired_z_force =
    vec_ctrl.mcDepthPID.pidNewCalc(depth_error);
```

结果是三个控制量：

```text
desired_world_x_force
    世界坐标系 X 方向期望力

desired_world_y_force
    世界坐标系 Y 方向期望力

desired_z_force
    Z 方向期望力
```

这些控制量会随着当前位置变化而变化：

```text
离目标远 → 误差大 → PID 输出较大
接近目标 → 误差变小 → PID 输出减小
到达目标 → 误差接近 0 → 推力接近 0
```

**下一个问题是：为什么算出的 X/Y 力还要根据当前 Yaw 旋转？旋转之后又怎样交给四个推进器？**

---

## 9. 根据当前 Yaw 将世界坐标力转换到机器人坐标系

PID 计算出来的 X/Y 力是在世界坐标系中表达的。

但是推进器安装在机器人自身坐标系中，所以需要根据当前航向角 `cur_yaw_` 进行坐标转换：

```cpp
float cur_yaw_rad = cur_yaw_ * M_PI / 180.0;

float cos_yaw = cos(cur_yaw_rad);
float sin_yaw = sin(cur_yaw_rad);
```

然后进行二维旋转：

```cpp
float desired_robot_x_force =
    desired_world_x_force * cos_yaw
    + desired_world_y_force * sin_yaw;

float desired_robot_y_force =
    -desired_world_x_force * sin_yaw
    + desired_world_y_force * cos_yaw;
```

转换后的变量含义是：

```text
desired_robot_x_force
    机器人自身 X 方向需要的力

desired_robot_y_force
    机器人自身 Y 方向需要的力

desired_z_force
    Z 方向需要的力
```

接下来调用底层三维控制分配：

```cpp
vec_ctrl.space3DControl(
    SHAPE_X,
    desired_robot_x_force,
    desired_robot_y_force,
    desired_z_force,
    0
);
```

最后一个参数是 Yaw 力矩。当前 `xyz` 路径传入的是：

```cpp
0
```

因此这条路径的主要目标是控制：

```text
X、Y、Z 位置
```

并不单独指定最终航向角。

**下一个问题是：`space3DControl()` 如何把整体的 X/Y/Z 力拆分成四个推进器各自的力？**

---

## 10. `space3DControl()` 将整体控制力分配给四个推进器

底层函数位于：

```text
src/asrobot/src/motion_control/c_vectoredthrustcontrol.cpp
```

函数入口是：

```cpp
void C_VectoredThrustControl::space3DControl(
    int _leg_shape,
    float _X_force,
    float _Y_force,
    float _Z_force,
    float _yaw_force)
```

当 `_leg_shape == SHAPE_X` 时，执行：

```cpp
calcSprayForces_Shape_X_2(
    _X_force,
    _Y_force,
    _Z_force,
    &__force1,
    &__force2,
    &__force3,
    &__force4,
    ...
);
```

对于 X 型布局，代码大致按照如下关系分配水平力：

```cpp
// X 方向
__fx2 += _X_force / 4 * sqrt(2);
__fx3 += _X_force / 4 * sqrt(2);

__fx1 -= _X_force / 4 * sqrt(2);
__fx4 -= _X_force / 4 * sqrt(2);

// Y 方向
__fx3 += _Y_force / 4 * sqrt(2);
__fx4 += _Y_force / 4 * sqrt(2);

__fx1 -= _Y_force / 4 * sqrt(2);
__fx2 -= _Y_force / 4 * sqrt(2);

// Z 方向
__fz1 = __fz2 = __fz3 = __fz4 = _Z_force / 4;
```

直观地说：

```text
推进器 1：承担 X 反向、Y 反向和 Z 分量
推进器 2：承担 X 正向、Y 反向和 Z 分量
推进器 3：承担 X 正向、Y 正向和 Z 分量
推进器 4：承担 X 反向、Y 正向和 Z 分量
```

每个推进器的水平和垂直分量组合成合力：

```text
推进器合力 = sqrt(水平分量² + 垂直分量²)
```

代码还会根据合力计算推进器或腿部的角度：

```cpp
*_leg1_angle = atan2(__fz1, __fx1 + 0.0001);
*_leg2_angle = atan2(__fz2, __fx2 + 0.0001);
*_leg3_angle = atan2(__fz3, __fx3 + 0.0001);
*_leg4_angle = atan2(__fz4, __fx4 + 0.0001);
```

并将角度限制在允许范围内，避免推进器姿态变化过大。

分配完成后，调用：

```cpp
yawLocomMode_X(...);

thrusterforce2speed_X(
    __force1,
    __force2,
    __force3,
    __force4
);
```

**下一个问题是：四个推进器的力怎样转换成 ROS 消息中的转速，并最终送给 Gazebo？**

---

## 11. 推进器力转换为转速并发布 `pwm_message`

`thrusterforce2speed_X()` 会把四个推进器的控制结果写入 `asrobot::pwm` 消息：

```cpp
pwm_msg.rotor_speed1 = __speed1;
pwm_msg.rotor_speed2 = __speed2;
pwm_msg.rotor_speed3 = __speed3;
pwm_msg.rotor_speed4 = __speed4;
```

然后发布：

```cpp
extern_pub_pwm->publish(pwm_msg);
```

`extern_pub_pwm` 在 `motion.cpp` 的 `main()` 中指向：

```cpp
pub_pwm = n.advertise<asrobot::pwm>(
    "pwm_message",
    10
);

extern_pub_pwm = &pub_pwm;
```

因为 `motion` 位于 `robot1` 命名空间，实际话题是：

```text
/robot1/pwm_message
```

可以查看：

```bash
rostopic echo /robot1/pwm_message
```

重点看：

```yaml
rotor_speed1:
rotor_speed2:
rotor_speed3:
rotor_speed4:
```

这里已经从“目标位置”变成了“四个推进器的目标转速”。

但是，`motion` 并没有直接调用 Gazebo 服务，而是通过另一个节点 `thruster_control` 转发。

**下一个问题是：`thruster_control` 如何把 `/robot1/pwm_message` 转成 Gazebo 推进器输入？**

---

## 12. `thruster_control` 将转速发布给 Gazebo

`asrobot.launch` 中的控制器节点是：

```xml
<node pkg="asrobot"
      type="thruster_control"
      name="thruster_control1"
      output="screen">

    <remap from="pwm_message"
           to="$(arg robot)/pwm_message"/>

    <remap from="thrusters/0/input"
           to="$(arg namespace)/thrusters/0/input"/>

    <remap from="thrusters/1/input"
           to="$(arg namespace)/thrusters/1/input"/>

    <remap from="thrusters/2/input"
           to="$(arg namespace)/thrusters/2/input"/>

    <remap from="thrusters/3/input"
           to="$(arg namespace)/thrusters/3/input"/>
</node>
```

因此最终连接关系是：

```text
thruster_control 订阅：
/robot1/pwm_message

thruster_control 发布：
/gazebo_robot1/thrusters/0/input
/gazebo_robot1/thrusters/1/input
/gazebo_robot1/thrusters/2/input
/gazebo_robot1/thrusters/3/input
```

源码位于：

```text
src/asrobot/src/gazebo_commu/thruster_control.cpp
```

它订阅：

```cpp
ros::Subscriber pwm_sub =
    n.subscribe("pwm_message", 1000, pwmcallback);
```

收到 `pwm` 消息后执行：

```cpp
void pwmcallback(const asrobot::pwm::ConstPtr& msg)
{
    rotor.data = msg->rotor_speed1;
    Propeller1_pub.publish(rotor);

    rotor.data = msg->rotor_speed2;
    Propeller2_pub.publish(rotor);

    rotor.data = msg->rotor_speed3;
    Propeller3_pub.publish(rotor);

    rotor.data = msg->rotor_speed4;
    Propeller4_pub.publish(rotor);
}
```

其中四个发布器分别对应：

```cpp
Propeller1_pub = n.advertise<...>("thrusters/0/input", 1000);
Propeller2_pub = n.advertise<...>("thrusters/1/input", 1000);
Propeller3_pub = n.advertise<...>("thrusters/2/input", 1000);
Propeller4_pub = n.advertise<...>("thrusters/3/input", 1000);
```

最终，Gazebo 中的 UUV 推进器插件收到这四个输入，产生实际的推进力，机器人开始运动。

可以验证：

```bash
rostopic echo /gazebo_robot1/thrusters/0/input
```

或者查看四个推进器：

```bash
rostopic echo /gazebo_robot1/thrusters/1/input
rostopic echo /gazebo_robot1/thrusters/2/input
rostopic echo /gazebo_robot1/thrusters/3/input
```

**下一个问题是：为什么只输入一次 `xyz 1 1 -1`，机器人却会持续运动，而不是只输出一次推进器速度？**

---

## 13. 这是一个持续运行的闭环控制

整个过程不是一次性的，而是循环运行的：

```text
1. `xyz` 设置目标点 (1, 1, -1)
2. Gazebo 返回当前机器人位置
3. decision 发布 mc_message
4. motion 更新当前状态
5. PID 计算控制力
6. 发布四个推进器速度
7. Gazebo 推进器改变机器人运动状态
8. Gazebo 返回新的位置
9. 回到第 3 步
```

`motion` 主循环中有：

```cpp
while (ros::ok())
{
    combine3DPidCtrl.debug_msg(mc_pub_debug);

    ros::spinOnce();
    loop_rate.sleep();
}
```

其中：

```cpp
ros::spinOnce();
```

会处理收到的 ROS 消息。当新的 `/robot1/mc_message` 到达时，就会调用：

```cpp
mccallback()
```

而 `mccallback()` 在当前状态为：

```cpp
RobotMode::MANUAL_XYZ_TARGET
```

时，又会调用：

```cpp
handle_xyz_target();
```

因此 `xyz` 命令只需设置一次目标，之后目标变量会一直保留：

```cpp
exp_p_x = 1;
exp_p_y = 1;
exp_p_z = -1;
```

机器人每次收到新的当前位置后，都会重新计算：

```text
目标 - 当前
```

这就是闭环控制。

当机器人接近目标时：

```text
位置误差变小
PID 输出变小
推进器速度降低
```

如果控制器和模型参数合适，机器人会在目标附近稳定下来。

输入：

```text
st
```

会执行：

```cpp
g_current_state = RobotMode::IDLE;
vec_ctrl.thrusterforce2speed(0, 0, 0, 0);
```

也就是切换到空闲状态并将推进器速度置零。

**到这里，完整运动链路已经闭合。下一步应该如何用 ROS 命令逐层验证每个环节确实在工作？**

---

## 14. 用 ROS 命令验证完整链路

建议启动仿真后，另外打开几个终端。

### 14.1 验证键盘命令是否发布

```bash
rostopic echo /robot1/cmd_message
```

在命令输入窗口输入：

```text
xyz 1 1 -1
```

如果看到：

```yaml
data: "xyz 1 1 -1"
```

说明：

```text
mancontrol → /robot1/cmd_message
```

正常。

---

### 14.2 验证当前位姿反馈

```bash
rostopic echo /robot1/mc_message
```

重点观察：

```yaml
c_x:
c_y:
c_z:
c_speedx:
c_speedy:
c_speedz:
c_navyaw:
```

如果机器人运动时 `c_x`、`c_y`、`c_z` 持续变化，说明：

```text
Gazebo → decision → mc_message
```

正常。

---

### 14.3 验证控制器输出

```bash
rostopic echo /robot1/pwm_message
```

重点观察：

```yaml
rotor_speed1:
rotor_speed2:
rotor_speed3:
rotor_speed4:
```

如果输入 `xyz` 后四个速度发生变化，说明：

```text
motion → PID → 推进器速度
```

正常。

---

### 14.4 验证 Gazebo 推进器输入

```bash
rostopic echo /gazebo_robot1/thrusters/0/input
```

也可以查看：

```bash
rostopic echo /gazebo_robot1/thrusters/1/input
rostopic echo /gazebo_robot1/thrusters/2/input
rostopic echo /gazebo_robot1/thrusters/3/input
```

如果这里有数据，说明：

```text
pwm_message → thruster_control → Gazebo 推进器插件
```

正常。

---

## 15. 最后总结：一次 `xyz` 命令究竟做了什么

输入：

```text
xyz 1 1 -1
```

对应的核心代码链可以压缩成：

```text
mancontrol.cpp
    getline(cin, msg.data)
    chatter_pub.publish(msg)
              │
              ▼
motion.cpp
    cmdcallback()
        g_current_state = MANUAL_XYZ_TARGET
        exp_p_x = 1
        exp_p_y = 1
        exp_p_z = -1
              │
              ▼
decision.cpp
    读取 /gazebo/model_states
    获取 gazebo_robot1 当前位姿
    发布 /robot1/mc_message
              │
              ▼
motion.cpp
    mccallback()
        更新 cur_p_x、cur_p_y、cur_p_z
        handle_xyz_target()
              │
              ▼
f_single_robot_control.cpp
    PidXYZControl_X()
        updateControl(SHAPE_X, ...)
              │
              ▼
c_sim_3d_ctrl.cpp
    计算 X/Y/Z 位置误差
    执行 X/Y/Z PID
    得到期望控制力
              │
              ▼
c_vectoredthrustcontrol.cpp
    space3DControl(SHAPE_X, ...)
    calcSprayForces_Shape_X_2()
    thrusterforce2speed_X()
              │
              ▼
thruster_control.cpp
    pwmcallback()
    发布四个 thrusters/*/input
              │
              ▼
Gazebo
    推进器产生推力
    机器人移动
```

最重要的三个入口文件是：

```text
src/asrobot/src/man_control/mancontrol.cpp
src/asrobot/src/motion_control/motion.cpp
src/asrobot/src/gazebo_commu/thruster_control.cpp
```

最重要的控制算法文件是：

```text
src/asrobot/src/motion_control/f_single_robot_control.cpp
src/asrobot/src/motion_control/c_sim_3d_ctrl.cpp
src/asrobot/src/motion_control/c_vectoredthrustcontrol.cpp
```

可以用一句话概括：

> `mancontrol` 把 `xyz 1 1 -1` 发布成命令，`motion` 把它保存为目标位置，`decision` 持续提供机器人当前位姿，PID 根据目标与当前位姿的误差计算整体控制力，推进器分配模块再把整体控制力转换为四个推进器的速度，最后由 `thruster_control` 送入 Gazebo。
