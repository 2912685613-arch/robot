# single_water：问题、修复与启动

本文按当前代码整理；所有 `src/...` 路径均相对于 `gazebo_catkin_ws`。
以下行号以当前工作区文件为准，后续增删代码时可能变化。


### 本次修改涉及文件

#  gazebo_catkin_ws/src/new_gazebo_model/urdf/gazebo_thruster.xacro
#  gazebo_catkin_ws/src/asrobot/src/gazebo_commu/joint_control.cpp
#  gazebo_catkin_ws/src/new_gazebo_model/config/joint_command_calibration.yaml
#  gazebo_catkin_ws/src/asrobot/launch/asrobot.launch
#  gazebo_catkin_ws/src/gazebo_launch/single_water.sh
#  gazebo_catkin_ws/src/gazebo_launch/wait_single_water.py
#  gazebo_catkin_ws/src/gazebo_world/launch/simple_underwater.launch
#  gazebo_catkin_ws/src/gazebo_world/worlds/single_water_fast.world
#  gazebo_catkin_ws/src/gazebo_launch/tests/test_single_water.py

## 1. 加载模型后机器人不动：推进器关节引用不一致

**现象：** 输入 `xyz 1 1 -1` 无响应；历史日志中 `gzserver` 出现
`Assertion px != 0 failed` 并退出，随后控制器找不到 `controller_manager`。
**原因：** 推进器插件 `GetJoint()` 查不到配置的关节，之后读取空指针导致崩溃。

修改前，模型与插件的名称不一致：

```xml
<!-- asrobot_model.xacro：原来的关节定义（其余字段省略） -->
<joint name="thruster_0_joint" type="continuous">
  <!-- origin、parent、child、axis、limit 省略 -->
</joint>
<!-- gazebo_thruster.xacro：原来的插件引用 -->
<jointName>${namespace}/thruster_${thruster_id}_joint</jointName>
```

**当前解决方案：四个推进器关节都已添加命名空间，与插件引用统一。**
文件：`src/new_gazebo_model/urdf/asrobot_model.xacro`、`urdf/gazebo_thruster.xacro`。

```xml
<!-- 当前模型：第 0 个关节；其余同样添加 ${namespace}/ -->
<joint name="${namespace}/thruster_0_joint" type="continuous">
  <!-- 其余字段省略 -->
</joint>
<!-- 当前插件宏 -->
<linkName>${namespace}/thruster_${thruster_id}</linkName>
<jointName>${namespace}/thruster_${thruster_id}_joint</jointName>
<gazebo reference="${namespace}/thruster_${thruster_id}">
  <selfCollide>false</selfCollide>
</gazebo>
```

例如两边均为 `gazebo_robot1/thruster_0_joint`，0～3 号推进器全部对应。
关键是“定义与引用一致”，不是“关节不能添加命名空间”；无需改回无前缀名称。
H/M/V 关节目前仍使用 `H1` 等原名，与 transmission、控制器配置对应。

**问题 1 对应代码位置：**

- `src/new_gazebo_model/urdf/asrobot_model.xacro`：第 **265、840、555、1130 行**分别定义 `thruster_0/1/2/3_joint`（文件内第 2 号先于第 1 号）。这四处由你添加 `${namespace}/`。
- `src/new_gazebo_model/urdf/gazebo_thruster.xacro`：第 **12、16 行**分别引用 link 和 joint；第 **59–61 行**引用推进器 link 的 Gazebo 属性。当前第 15 行旧注释“不带命名空间”已与代码不符，判断应以定义和引用为准。

## 2. 能水平运动但不下潜：新模型转向方向与零位不同

**原因：** 经过完整 CAD 关节坐标变换后，V2/V4 与 V1/V3 的垂直转向作用相反。
旧程序把同号角度直接发给四个关节，导致两个推进器向下、两个向上，推力抵消。
**方案：** 在最终发布关节命令时，为新模型增加方向和零位校准，不改 PID 目标。

文件：`src/asrobot/src/gazebo_commu/joint_control.cpp`。

```cpp
// 修改前：直接发布，示例为 V1 分支
servo3_pub.publish(channel);
```

```cpp
// 修改后：全局默认参数，未加载配置时保持原行为
std::vector<double> joint_command_scale(12, 1.0);
std::vector<double> joint_command_offset_deg(12, 0.0);

// main()：读取该节点的私有参数
ros::NodeHandle private_n("~");
private_n.getParam("joint_command_scale", joint_command_scale);
private_n.getParam("joint_command_offset_deg", joint_command_offset_deg);

// publish()：12 个发布分支都改用 calibrated，以下仅展示 V1
std_msgs::Float64 calibrated;
calibrated.data = joint_command_scale[num] * channel.data
                + joint_command_offset_deg[num] * PI / 180.0;
servo3_pub.publish(calibrated);
```

`channel.data` 已是弧度，偏移量以度配置，因此只对偏移量进行度转弧度。
实现还检查参数各有 12 项、数值有限且 scale 非零，避免非法校准参数。

新增配置：`src/new_gazebo_model/config/joint_command_calibration.yaml`。

```yaml
# 通道顺序：H1 M1 V1 H2 M2 V2 H3 M3 V3 H4 M4 V4
joint_command_scale:      [1.0, 1.0, 1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0, -1.0]
joint_command_offset_deg: [0.0, 0.0, 3.0, 0.0, 0.0, -27.0, 0.0, 0.0, 1.0, 0.0, 0.0, -27.0]
```

| 关节 | scale | offset | 控制中性角 → 模型中性角 |
| --- | --- | --- | --- |
| V1 | +1 | +3° | -14° → -11° |
| V2 | -1 | -27° | -14° → -13° |
| V3 | +1 | +1° | -14° → -13° |
| V4 | -1 | -27° | -14° → -13° |

偏移满足 `offset = 模型中性角 - scale × 控制中性角`。
例如 V2：`-13 - (-1) × (-14) = -27°`，不能只反号而不补零位。
这组中性角针对当前 X 型姿态（M1=-2°、其余 M=0°），不是所有步态的通用标定。

文件：`src/asrobot/launch/asrobot.launch`，原来没有校准配置参数，现在增加：

```xml
<arg name="joint_command_config" default=""/>
<node if="$(arg enable_joint_control)"
      pkg="asrobot" type="joint_control" name="joint_control_$(arg robot)"
      output="screen">
  <rosparam if="$(eval arg('joint_command_config') != '')"
            file="$(arg joint_command_config)" command="load"/>
  <!-- 原有 remap 省略 -->
</node>
```

`single_water.sh` 显式传入该 YAML；其他入口不传时默认 scale=1、offset=0，
因此旧模型的默认命令方向和角度不变。

**问题 2 对应代码位置（重点是关节转向，不是螺旋桨自转）：**

- `src/asrobot/src/gazebo_commu/joint_control.cpp`：第 **14–15 行**设置默认恒等映射；第 **56–76 行**在 `publish()` 中转换并发布关节角度，其中 V2 是 **70 行**、V4 是 **76 行**；第 **234–249 行**读取并校验私有参数。
- `src/new_gazebo_model/config/joint_command_calibration.yaml`：第 **7–8 行**设置 12 个关节的方向和零位；V2、V4 位于从 0 开始的通道 **5、11**，这两个通道的 `scale=-1`、`offset=-27°`。
- `src/asrobot/launch/asrobot.launch`：第 **17 行**定义可选配置路径，第 **38–42 行**把 YAML 加载到 `joint_control` 的私有参数。
- `src/gazebo_launch/single_water.sh`：第 **57–58 行**启动 `asrobot.launch` 时传入新模型的 YAML，故该脚本会启用校准。

没有更改 `ThrusterPlugin.cc` 的螺旋桨自转方向；上述转换改变的是 V 关节的**姿态命令**，以使四个推进器的垂直推力分量同向。


## 3. 启动命令

首次编译或修改 C++ 后执行（仅修改 xacro/YAML/launch 时通常无需重新编译）：

```bash
cd ~/robot/gazebo_catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps asrobot
source devel/setup.bash
bash src/gazebo_launch/single_water.sh
```
