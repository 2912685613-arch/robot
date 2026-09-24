# 项目功能完整参考文档

## 目录
- [启动脚本总览](#启动脚本总览)
- [单机器人控制命令](#单机器人控制命令)
- [传感器功能](#传感器功能)
- [视觉功能](#视觉功能)
- [多机器人功能](#多机器人功能)
- [参数调节功能](#参数调节功能)
- [话题监控命令](#话题监控命令)

---

## 启动脚本总览

### 单机器人仿真

#### 1. 单机器人陆地环境
```bash
cd ~/robot/gazebo_catkin_ws
./src/gazebo_launch/single_land.sh
```
**说明：** 启动陆地环境单机器人仿真
- World: `flat_land.launch`
- Mode: `land`

#### 2. 单机器人水下环境（基础版）
```bash
cd ~/robot/gazebo_catkin_ws
./src/gazebo_launch/single_water.sh
```
**说明：** 启动基础水下环境单机器人仿真
- World: `underwater.launch`
- Mode: `water`

#### 3. 单机器人水下环境（带 ArUco 标记）
```bash
cd ~/robot/gazebo_catkin_ws
./src/gazebo_launch/single_water_l.sh
```
**说明：** 启动带 ArUco 标记的水下环境，支持视觉对接测试
- World: `underwater_l.launch`
- Mode: `water`
- 包含水下 ArUco 标记（marker_1, marker_5 等）

**⚠️ 注意：** 该脚本仅启动仿真环境和控制器，需要手动启动机器人控制节点：
```bash
roslaunch asrobot asrobot.launch
```

### 多机器人仿真

#### 4. 五机器人编队
```bash
cd ~/robot/gazebo_catkin_ws
./src/gazebo_launch/five_robots.sh
```

#### 5. 八机器人陆地编队
```bash
cd ~/robot/gazebo_catkin_ws
./src/gazebo_launch/eight_land.sh
```

#### 6. 八机器人水下编队
```bash
cd ~/robot/gazebo_catkin_ws
./src/gazebo_launch/eight_water.sh
```

#### 7. 多机器人水下环境
```bash
cd ~/robot/gazebo_catkin_ws
./src/gazebo_launch/multi_water.sh
```

#### 8. 多机器人陆地降落
```bash
cd ~/robot/gazebo_catkin_ws
./src/gazebo_launch/multi_land.sh
```

#### 9. 单机器人陆地降落
```bash
cd ~/robot/gazebo_catkin_ws
./src/gazebo_launch/single_land.sh
```

### 编队控制

#### 10. 编队模式 1
```bash
cd ~/robot/gazebo_catkin_ws
./src/gazebo_launch/Formation1.sh
```

#### 11. 编队模式 2
```bash
cd ~/robot/gazebo_catkin_ws
./src/gazebo_launch/Formation2.sh
```

---

## 单机器人控制命令

### ✅ 已验证可用的核心功能

这些命令在 `mancontrol` 节点输入窗口中使用（提示符：`please input cmd!`）

#### 1. 三维位置控制：`xyz x y z`
```
xyz 0 0.5 -0.5
```
**功能：** 控制机器人移动到世界坐标系下的目标位置
- `x`: X 轴坐标（前后）
- `y`: Y 轴坐标（左右）
- `z`: Z 轴坐标（上下，水下为负值）

**控制模式：** `RobotMode::MANUAL_XYZ_TARGET`  
**控制函数：** `PidXYZControl_X(...)`  
**源码位置：** [motion.cpp:315-321](../src/asrobot/src/motion_control/motion.cpp#L315-L321)

**示例测试序列：**
```
xyz 0 0 -0.5      # 原点，深度 0.5m
xyz 1 0 -0.5      # 前进 1m
xyz 1 1 -0.5      # 向右 1m
xyz 0 0 -0.5      # 返回
st                # 停止
```

#### 2. 航向控制：`yaw 角度`
```
yaw 90
```
**功能：** 控制机器人转到目标航向角（绝对角度，非相对转动）
- 单位：度（°）
- 0° = 正东，90° = 正北，180° = 正西，-90° = 正南

**控制模式：** `RobotMode::MANUAL_YAW`  
**控制函数：** `PidYawControl_H(exp_yaw, cur_yaw)`  
**源码位置：** [motion.cpp:323-327](../src/asrobot/src/motion_control/motion.cpp#L323-L327)

**示例测试：**
```
yaw 0       # 转向正东
yaw 90      # 转向正北
yaw 180     # 转向正西
yaw -90     # 转向正南
st          # 停止
```

#### 3. 深度控制：`dep z值`
```
dep -0.5
```
**功能：** 单独控制深度/Z 轴位置
- 水面：z ≈ 0
- 水下：z < 0（负值）

**控制模式：** `RobotMode::MANUAL_DEPTH`  
**控制函数：** `PidDepthControl_X(exp_p_z, cur_p_z)`  
**源码位置：** [motion.cpp:329-333](../src/asrobot/src/motion_control/motion.cpp#L329-L333)

**示例测试：**
```
dep -0.3    # 下潜到 0.3m
dep -0.8    # 下潜到 0.8m
dep -0.2    # 上浮到 0.2m
st          # 停止
```

#### 4. 深度+航向联合控制：`depyaw 深度 角度`
```
depyaw -0.5 90
```
**功能：** 同时控制深度和航向，不进行 XY 平面移动
- 第一个参数：目标深度
- 第二个参数：目标航向角

**控制模式：** `RobotMode::MANUAL_DEPTH_YAW`  
**控制函数：** `PidDepthYawControl_H(...)`  
**源码位置：** [motion.cpp:335-341](../src/asrobot/src/motion_control/motion.cpp#L335-L341)

**示例测试：**
```
depyaw -0.5 0      # 深度 0.5m，朝东
depyaw -0.5 90     # 深度 0.5m，朝北
depyaw -0.8 180    # 深度 0.8m，朝西
st                 # 停止
```

#### 5. 停止：`st`
```
st
```
**功能：** 停止所有运动
- 四个推进器输出归零
- 清理 ADRC 控制器状态
- 停止步态
- 腿部关节恢复零角度

**控制模式：** `RobotMode::IDLE`  
**源码位置：** [motion.cpp:381-387](../src/asrobot/src/motion_control/motion.cpp#L381-L387)

**执行动作：**
```cpp
vec_ctrl.thrusterforce2speed(0, 0, 0, 0);
clearADRCSpeedControllerVariables();
gait_stop();
legSetAngle(0, 0, 0);
```

### 🔧 实验性功能

#### 6. 抓取/接近模式：`grasp x y z`
```
grasp 1 0 -0.5
```
**功能：** 自动计算朝向目标点的航向并接近
- 自动设置 `exp_yaw = atan2(Δy, Δx) * 57.3`
- 使用 ADRC 位置控制

**控制模式：** `RobotMode::GRASP`  
**控制函数：** `ADRCPosition3DControl_G(...)`  
**源码位置：** [motion.cpp:343-353](../src/asrobot/src/motion_control/motion.cpp#L343-L353)

**⚠️ 注意：** 目前没有物理抓取器闭合控制，仅为接近模式

#### 7. ArUco 视觉对接：`dock`
```
dock
```
**功能：** 启动基于下视相机 ArUco 标记的自动对接
- 需要 `aruco_det` 和 `aruco_posture` 节点运行
- 需要在 `underwater_l.world` 环境中测试

**控制模式：** `RobotMode::DOCKING_ARUCO`  
**数据链路：**
```
下视相机 → aruco_det → /aruco/pose → aruco_posture 
         → /camera_aruco_data → decision → /robot1/mc_message 
         → motion::handle_docking_aruco() → 推进器控制
```

**源码位置：**
- 命令解析：[motion.cpp:389-393](../src/asrobot/src/motion_control/motion.cpp#L389-L393)
- 控制函数：[f_single_robot_control.cpp:569-641](../src/asrobot/src/motion_control/f_single_robot_control.cpp#L569-L641)

**⚠️ 已知问题：**
- `flag_xy` 和 `flag_z` 状态变量未跨回调保存
- 阶段状态机不稳定

**测试前检查：**
```bash
# 1. 确认相机图像
rostopic hz /gazebo_robot1/camera_bottom/camera_bottom/image_raw

# 2. 确认 ArUco 检测
rostopic echo /aruco/pose

# 3. 确认处理后的位姿
rostopic echo /camera_aruco_data
```

### 🦿 腿部关节控制

#### 8. 手动设置关节角度：`a A B C`
```
a 10 20 30
```
**功能：** 同时设置四条腿的三个关节角度
- `A`: 关节 1 角度
- `B`: 关节 2 角度
- `C`: 关节 3 角度
- 单位：度（°）

**源码位置：** [motion.cpp:399-406](../src/asrobot/src/motion_control/motion.cpp#L399-L406)

**执行流程：**
```cpp
legSetAngle(A, B, C)
  ↓
gait_ctrl.legsetangle(_A, _B, _C)
  ↓
Leg1.set_angle(_A, _B, _C, 500)
Leg2.set_angle(_A, _B, _C, 500)
Leg3.set_angle(_A, _B, _C, 500)
Leg4.set_angle(_A, _B, _C, 500)
```

**⚠️ 注意：**
- 必须提供完整的三个参数
- 没有安全范围限制，避免输入过大角度
- 建议从小角度开始测试（±5°, ±10°）

#### 9. 步态控制：`gait 参数`
```
gait 0.5
```
**功能：** 启动步态行走模式（理论支持）

**控制模式：** `RobotMode::GAIT_CONTROL`  
**控制函数：** `gaitControl()`  
**源码位置：** [f_single_robot_control.cpp:676-702](../src/asrobot/src/motion_control/f_single_robot_control.cpp#L676-L702)

**⚠️ 当前不可用：**
- `mode` 固定为 100，需要 `mode == 1` 才能执行
- `dir` 固定为 0，无法选择方向
- 没有命令接口配置 `mode` 和 `dir`

**理论支持的方向：**
```cpp
if (dir == FORWARD)  gait_ctrl.forward4(gait_R);
if (dir == LEFT)     gait_ctrl.round_left(gait_R);
if (dir == RIGHT)    gait_ctrl.round_right(gait_R);
if (dir == BACK)     gait_ctrl.back(gait_R);
```

---

## 参数调节功能

### PID 参数在线调节

⚠️ **仅供开发调试使用，初学者不建议修改**

#### 航向 PID：`pidyaw P I D`
```
pidyaw 0.025 0.011 0.06
```
**调节对象：** 航向角控制器  
**源码位置：** [motion.cpp:354-358](../src/asrobot/src/motion_control/motion.cpp#L354-L358)

#### 深度 PID：`piddep P I D`
```
piddep 0.02 0.01 0.05
```
**调节对象：** 深度控制器  
**源码位置：** [motion.cpp:360-364](../src/asrobot/src/motion_control/motion.cpp#L360-L364)

#### X 轴 PID：`pidx P I D`
```
pidx 0.03 0.01 0.04
```
**调节对象：** X 方向位置控制器  
**源码位置：** [motion.cpp:366-370](../src/asrobot/src/motion_control/motion.cpp#L366-L370)

#### Y 轴 PID：`pidy P I D`
```
pidy 0.03 0.01 0.04
```
**调节对象：** Y 方向位置控制器  
**源码位置：** [motion.cpp:372-376](../src/asrobot/src/motion_control/motion.cpp#L372-L376)

#### 速度 PID：`pidspeed P I D`
```
pidspeed 0.1 0.05 0.02
```
**调节对象：** 速度控制器  
**源码位置：** [motion.cpp:408-412](../src/asrobot/src/motion_control/motion.cpp#L408-L412)

#### Z 速度 PID：`pidvz P I D`
```
pidvz 0.02 0.01 0.03
```
**调节对象：** Z 轴速度内环控制器  
**源码位置：** [motion.cpp:414-418](../src/asrobot/src/motion_control/motion.cpp#L414-L418)

#### Z 位置 PID：`pidpz P I D`
```
pidpz 0.03 0.01 0.04
```
**调节对象：** Z 轴位置外环控制器  
**源码位置：** [motion.cpp:420-424](../src/asrobot/src/motion_control/motion.cpp#L420-L424)

---

## 传感器功能

### 机器人模型包含的传感器

#### 1. IMU（惯性测量单元）
**话题：** `/gazebo_robot1/imu`  
**消息类型：** `sensor_msgs/Imu`  
**提供数据：**
- 姿态四元数
- 角速度
- 线性加速度

**配置位置：** [gazebo_model.xacro](../src/gazebo_model/urdf/robot/gazebo_model.xacro)

**查看数据：**
```bash
rostopic echo /gazebo_robot1/imu
```

#### 2. 压力传感器（深度反馈）
**话题：** `/gazebo_robot1/pressure`  
**消息类型：** `sensor_msgs/FluidPressure`  
**提供数据：**
- 流体压力（用于计算深度）

**查看数据：**
```bash
rostopic echo /gazebo_robot1/pressure
```

#### 3. 下视相机
**话题：** `/gazebo_robot1/camera_bottom/camera_bottom/image_raw`  
**消息类型：** `sensor_msgs/Image`  
**用途：**
- ArUco 标记检测
- 水下对接导航

**查看图像：**
```bash
rosrun image_view image_view image:=/gazebo_robot1/camera_bottom/camera_bottom/image_raw
```

**查看频率：**
```bash
rostopic hz /gazebo_robot1/camera_bottom/camera_bottom/image_raw
```

#### 4. 双目相机
**话题：**
- `/gazebo_robot1/stereo_camera/left/image_raw`
- `/gazebo_robot1/stereo_camera/right/image_raw`

**用途：**
- 立体视觉
- 目标定位
- 深度估计

#### 5. Gazebo 真值位置
**话题：** `/gazebo/model_states`  
**消息类型：** `gazebo_msgs/ModelStates`  
**提供数据：**
- 所有模型的精确位置
- 所有模型的精确速度
- 用于控制算法验证

**查看数据：**
```bash
rostopic echo /gazebo/model_states
```

---

## 视觉功能

### ArUco 标记检测与位姿估计

#### 节点架构
```
下视相机图像
  ↓
[aruco_det 节点]
  ↓ 发布
/aruco/pose (geometry_msgs/PoseStamped)
  ↓ 订阅
[aruco_posture 节点]
  ↓ 发布
/camera_aruco_data (asrobot/camera_aruco)
  ↓ 订阅
[decision 节点]
```

#### 相关节点

##### 1. aruco_det
**包名：** `asrobot`  
**类型：** `aruco_det`  
**源码：** [aruco_det.cpp](../src/asrobot/src/aruco_det/aruco_det.cpp)  
**功能：**
- 订阅下视相机图像
- 检测 ArUco 标记
- 估计标记位姿
- 发布检测结果

**输入：**
- `/gazebo_robot1/camera_bottom/camera_bottom/image_raw`

**输出：**
- `/aruco/pose`

**启动：**
```xml
<node pkg="asrobot" type="aruco_det" name="aruco_det" output="screen"/>
```

##### 2. aruco_posture
**包名：** `asrobot`  
**类型：** `aruco_posture`  
**功能：**
- 订阅 ArUco 检测结果
- 转换坐标系
- 封装为控制消息

**输入：**
- `/aruco/pose`

**输出：**
- `/camera_aruco_data`

**启动：**
```xml
<node pkg="asrobot" type="aruco_posture" name="aruco_posture" output="screen"/>
```

#### 测试流程

1. **启动带 ArUco 的水下环境：**
```bash
./src/gazebo_launch/single_water_l.sh
```

2. **启动控制节点：**
```bash
roslaunch asrobot asrobot.launch
```

3. **检查相机图像：**
```bash
rostopic hz /gazebo_robot1/camera_bottom/camera_bottom/image_raw
```

4. **查看 ArUco 检测：**
```bash
rostopic echo /aruco/pose
```

5. **查看处理后的数据：**
```bash
rostopic echo /camera_aruco_data
```

6. **启动自动对接：**
```
dock
```

---

## 多机器人功能

### 支持的多机器人场景

#### 1. 多机器人编队控制
**脚本：**
- `five_robots.sh`
- `eight_land.sh`
- `eight_water.sh`

**相关源码：**
- [f_multi_robot_plan.cpp](../src/asrobot/src/decision_center/f_multi_robot_plan.cpp)
- [c_multi_agent_algorithm.cpp](../src/asrobot/src/decision_center/c_multi_agent_algorithm.cpp)
- [mpc_formation.cpp](../src/asrobot/src/motion_control/mpc_formation.cpp)

#### 2. 编队形状切换
**脚本：**
- `Formation1.sh`
- `Formation2.sh`

#### 3. 多机器人路径规划
**功能：**
- 位置交换
- 避碰算法
- 协同控制

#### 4. 无人机辅助两栖编队
**启动文件：** `asrobot_quadrotor.launch`  
**说明：** 用于无人机协助的两栖多机器人编队控制

**启动：**
```bash
roslaunch asrobot asrobot_quadrotor.launch
```

⚠️ **注意：** 多机器人功能需要单独测试验证，不能从单机器人脚本直接推断

---

## 话题监控命令

### 常用监控命令

#### 查看所有话题
```bash
rostopic list
```

#### 查看机器人相关话题
```bash
rostopic list | grep gazebo_robot1
```

#### 查看推进器输出
```bash
rostopic echo /robot1/pwm_message
```

或：
```bash
rostopic echo /gazebo_robot1/thrusters/0/input
rostopic echo /gazebo_robot1/thrusters/1/input
rostopic echo /gazebo_robot1/thrusters/2/input
rostopic echo /gazebo_robot1/thrusters/3/input
```

#### 查看控制命令
```bash
rostopic echo /robot1/mc_message
```

#### 查看节点列表
```bash
rosnode list
```

#### 查看关键节点
```bash
rosnode list | grep -E "motion|decision|mancontrol|joint_control|thruster_control"
```

#### 预期节点（单机器人完整控制链）
```
/robot1/motion
/robot1/mancontrol1
/decision_robot1
/joint_control1
/thruster_control1
/aruco_det
/aruco_posture
```

#### 查看话题频率
```bash
rostopic hz /gazebo_robot1/imu
rostopic hz /gazebo_robot1/pressure
rostopic hz /gazebo_robot1/camera_bottom/camera_bottom/image_raw
```

#### 查看话题信息
```bash
rostopic info /robot1/mc_message
```

---

## 推荐的学习测试顺序

### 阶段 1：基础控制验证
1. ✅ **三维位置控制**
   ```
   xyz 0 0 -0.5
   xyz 1 0 -0.5
   xyz 1 1 -0.5
   xyz 0 0 -0.5
   st
   ```

2. ✅ **航向控制**
   ```
   yaw 0
   yaw 90
   yaw 180
   yaw -90
   st
   ```

3. ✅ **深度控制**
   ```
   dep -0.3
   dep -0.8
   dep -0.2
   st
   ```

4. ✅ **组合控制**
   ```
   depyaw -0.5 0
   depyaw -0.5 90
   depyaw -0.8 180
   st
   ```

### 阶段 2：传感器数据验证
5. **IMU 数据**
   ```bash
   rostopic echo -n 1 /gazebo_robot1/imu
   ```

6. **压力传感器**
   ```bash
   rostopic echo -n 1 /gazebo_robot1/pressure
   ```

7. **相机图像**
   ```bash
   rostopic hz /gazebo_robot1/camera_bottom/camera_bottom/image_raw
   rosrun image_view image_view image:=/gazebo_robot1/camera_bottom/camera_bottom/image_raw
   ```

### 阶段 3：视觉功能测试
8. **ArUco 检测**
   ```bash
   rostopic echo /aruco/pose
   rostopic echo /camera_aruco_data
   ```

9. **自动对接测试**
   ```
   dock
   ```
   ⚠️ 需要分析状态机稳定性

### 阶段 4：高级功能研究
10. **腿部关节控制**
    ```
    a 5 5 5
    a 0 0 0
    ```

11. **步态控制**（需要先修复 mode/dir 接口）

12. **多机器人编队**（需要单独启动脚本）

---

## 当前不建议测试的功能

| 功能 | 原因 |
|------|------|
| 步态行走 | 命令接口未配置 `mode`/`dir` 变量 |
| ArUco 自动对接 | 状态机阶段变量未持久化 |
| 多机器人编队 | 需要单独启动和验证流程 |
| 双目视觉跟踪 | 默认 launch 文件中相关节点被注释 |
| 参考轨迹保存 | 回调函数为空实现 |
| 运行数据保存 | 定时器回调为空实现 |

---

## 控制器架构总结

### 控制模式枚举
```cpp
enum class RobotMode {
    IDLE,                    // 空闲（st）
    MANUAL_XYZ_TARGET,       // XYZ 位置控制（xyz）
    MANUAL_YAW,              // 航向控制（yaw）
    MANUAL_DEPTH,            // 深度控制（dep）
    MANUAL_DEPTH_YAW,        // 深度+航向（depyaw）
    GRASP,                   // 抓取模式（grasp）
    DOCKING_ARUCO,           // ArUco 对接（dock）
    GAIT_CONTROL,            // 步态控制（gait）
    // ... 其他模式
};
```

### 控制函数映射
| 模式 | 控制函数 |
|------|---------|
| `MANUAL_XYZ_TARGET` | `PidXYZControl_X()` |
| `MANUAL_YAW` | `PidYawControl_H()` |
| `MANUAL_DEPTH` | `PidDepthControl_X()` |
| `MANUAL_DEPTH_YAW` | `PidDepthYawControl_H()` |
| `GRASP` | `ADRCPosition3DControl_G()` |
| `DOCKING_ARUCO` | `handle_docking_aruco()` |
| `GAIT_CONTROL` | `gaitControl()` |

---

## 文件快速索引

| 功能 | 源码文件 |
|------|---------|
| 命令解析 | [motion.cpp](../src/asrobot/src/motion_control/motion.cpp) |
| 单机器人控制 | [f_single_robot_control.cpp](../src/asrobot/src/motion_control/f_single_robot_control.cpp) |
| 步态控制 | [c_gaitcontrol.cpp](../src/asrobot/src/motion_control/c_gaitcontrol.cpp) |
| 决策中心 | [decision.cpp](../src/asrobot/src/decision_center/decision.cpp) |
| ArUco 检测 | [aruco_det.cpp](../src/asrobot/src/aruco_det/aruco_det.cpp) |
| 多机器人规划 | [f_multi_robot_plan.cpp](../src/asrobot/src/decision_center/f_multi_robot_plan.cpp) |
| MPC 编队 | [mpc_formation.cpp](../src/asrobot/src/motion_control/mpc_formation.cpp) |
| 机器人模型 | [gazebo_model.xacro](../src/gazebo_model/urdf/robot/gazebo_model.xacro) |

---

## 附录：关节控制器列表

机器人有 **12 个关节位置控制器**，对应四条腿各三个关节：

```yaml
joint1_position_controller   # 腿1-关节1
joint2_position_controller   # 腿1-关节2
joint3_position_controller   # 腿1-关节3
joint4_position_controller   # 腿2-关节1
joint5_position_controller   # 腿2-关节2
joint6_position_controller   # 腿2-关节3
joint7_position_controller   # 腿3-关节1
joint8_position_controller   # 腿3-关节2
joint9_position_controller   # 腿3-关节3
joint10_position_controller  # 腿4-关节1
joint11_position_controller  # 腿4-关节2
joint12_position_controller  # 腿4-关节3
```

**配置文件：** [gazebo_control.launch](../src/gazebo_control/launch/gazebo_control.launch)

---

**文档版本：** 1.0  
**最后更新：** 2024  
**维护者：** gjx
