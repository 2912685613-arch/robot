# 机器人仿真系统功能清单

## 项目概述
这是一个基于ROS和Gazebo的水下/地面多功能机器人仿真系统，支持单机器人、多机器人、水陆两栖、编队控制等多种场景。

---

## 一、单机器人仿真

### 1.1 单机器人水下仿真（基础功能）✅
**功能描述**: 在水下环境中启动单个机器人，支持全向运动控制

**启动命令**:
```bash
cd ~/robot/gazebo_catkin_ws/src/gazebo_launch
bash single_water.sh
```

**控制命令**（在robot_control窗口中输入）:
- `xyz 1 1 -1` - 全向运动控制（X方向1m，Y方向1m，深度-1m）
- `yaw 90` - 航向角控制（旋转90度）
- `pz -1` - 深度控制（下潜到-1m）
- `depyaw -1 20` - 深度和航向同时控制

**说明**: 
- 启动4个终端窗口：world(世界)、model(模型)、control(控制器)、robot_control(命令输入)
- 机器人支持H型和X型两种构型切换

---

### 1.2 单机器人地面仿真
**功能描述**: 在陆地环境中启动单个机器人

**启动命令**:
```bash
cd ~/robot/gazebo_catkin_ws/src/gazebo_launch
bash single_land.sh
```

**说明**: 适用于地面环境测试

---

### 1.3 单机器人水陆两栖仿真
**功能描述**: 在水陆交界环境中测试机器人跨域能力

**启动命令**:
```bash
cd ~/robot/gazebo_catkin_ws/src/gazebo_launch
bash single_water_land.sh
```

---

### 1.4 单机器人+UUV仿真
**功能描述**: 单个机器人与水下无人航行器（UUV）协同

**启动命令**:
```bash
cd ~/robot/gazebo_catkin_ws/src/gazebo_launch
bash single_water_uuv.sh
```

---

## 二、多机器人仿真

### 2.1 双机器人水下仿真
**功能描述**: 两个机器人在水下环境中协同工作

**启动命令**:
```bash
cd ~/robot/gazebo_catkin_ws/src/gazebo_launch
bash single_water_two_robot.sh
```

---

### 2.2 多机器人水下仿真（带沙地）
**功能描述**: 多个机器人在带沙地的水下环境中仿真

**启动命令**:
```bash
cd ~/robot/gazebo_catkin_ws/src/gazebo_launch
bash multi_water.sh
```

---

### 2.3 多机器人水下仿真（无沙地）
**功能描述**: 多个机器人在纯水下环境中仿真

**启动命令**:
```bash
cd ~/robot/gazebo_catkin_ws/src/gazebo_launch
bash multi_water_nosand.sh
```

---

### 2.4 多机器人地面仿真
**功能描述**: 多个机器人在地面环境中协同

**启动命令**:
```bash
cd ~/robot/gazebo_catkin_ws/src/gazebo_launch
bash multi_land.sh
```

---

### 2.5 五机器人集群仿真
**功能描述**: 5个机器人同时仿真（包括不同型号）

**启动命令**:
```bash
cd ~/robot/gazebo_catkin_ws/src/gazebo_launch
bash five_robots.sh
```

**说明**: 
- 包含3个普通机器人 + 2个eight型机器人
- 每个机器人有独立的控制器

---

## 三、编队控制

### 3.1 编队模式1
**功能描述**: 多机器人编队控制方案1（包含四旋翼）

**启动命令**:
```bash
cd ~/robot/gazebo_catkin_ws/src/gazebo_launch
bash Formation1.sh
```

**说明**: 
- 包含水陆环境
- 1个四旋翼 + 3个水下机器人
- 初始位置：robot1(6,8,-0.16)、robot2(7.5,-7,-0.16)、robot3(6,-6,-0.16)

---

### 3.2 编队模式2
**功能描述**: 多机器人编队控制方案2

**启动命令**:
```bash
cd ~/robot/gazebo_catkin_ws/src/gazebo_launch
bash Formation2.sh
```

---

## 四、特殊配置仿真

### 4.1 Eight型机器人水下仿真
**功能描述**: Eight型（8推进器）机器人水下仿真

**启动命令**:
```bash
cd ~/robot/gazebo_catkin_ws/src/gazebo_launch
bash eight_water.sh
```

**说明**: 
- Eight型机器人 + 2个普通机器人协同
- 适用于复杂水下操作场景

---

### 4.2 Eight型机器人地面仿真
**功能描述**: Eight型机器人在地面环境测试

**启动命令**:
```bash
cd ~/robot/gazebo_catkin_ws/src/gazebo_launch
bash eight_land.sh
```

---

### 4.3 四旋翼+单机器人水下仿真
**功能描述**: 四旋翼无人机与水下机器人协同

**启动命令**:
```bash
cd ~/robot/gazebo_catkin_ws/src/gazebo_launch
bash quator_single_water.sh
```

**说明**: 空中-水下跨域协同

---

### 4.4 四旋翼+单机器人水陆仿真
**功能描述**: 四旋翼与水陆两栖机器人协同

**启动命令**:
```bash
cd ~/robot/gazebo_catkin_ws/src/gazebo_launch
bash "quator_single_water _land.sh"
```

---

## 五、ROS常用调试命令

### 5.1 话题查看
```bash
# 查看当前所有话题
rostopic list

# 打印指定话题内容（实时调试）
rostopic echo /robot1/motion_debug_topic

# 查看话题信息
rostopic info /robot1/motion_debug_topic
```

---

### 5.2 数据记录
```bash
# 记录指定话题到bag文件
rosbag record -O test_attach_detach.bag /gazebo_robot1/motion_debug_topic /gazebo_robot2/motion_debug_topic

# 播放bag文件
rosbag play test_attach_detach.bag

# 查看bag文件信息
rosbag info test_attach_detach.bag
```

---

### 5.3 机器人对接（抓取/分离）

**抓取命令**（将robot2连接到robot1）:
```bash
rosservice call /link_attacher_node/attach "model_name_1: 'gazebo_robot1'
link_name_1: 'gazebo_robot1/base_link'
model_name_2: 'gazebo_robot2'
link_name_2: 'gazebo_robot2/base_link'"
```

**分离命令**:
```bash
rosservice call /link_attacher_node/detach "model_name_1: 'gazebo_robot1'
link_name_1: 'gazebo_robot1/base_link'
model_name_2: 'gazebo_robot2'
link_name_2: 'gazebo_robot2/base_link'"
```

---

### 5.4 模型生成与查看
```bash
# 从xacro生成URDF模型
rosrun xacro xacro $(find gazebo_model)/urdf/robot/gazebo_model.xacro namespace:=gazebo_robot1 color:=Orange > ~/final_robot.urdf

# 查看ROS节点
rosnode list

# 查看ROS服务
rosservice list
```

---

## 六、机器人控制功能

### 6.1 运动控制模式
机器人支持以下控制模式（通过mancontrol键盘接口输入）：

1. **全向运动**: `xyz x y z` - 同时控制X、Y、Z三个方向
2. **航向控制**: `yaw angle` - 单独控制航向角（H型）
3. **深度控制**: `pz depth` - 单独控制深度
4. **深度+航向**: `depyaw depth yaw` - 同时控制深度和航向

---

### 6.2 支持的机器人构型
- **H型**: 水平推进器配置，适合前进和转向
- **X型**: 对角推进器配置，适合全向运动
- **H_T型**: H型变体
- **G型**: 特殊推进器配置
- **Eight型**: 8推进器配置，冗余度高

---

### 6.3 控制器类型
- **PID控制器**: 位置式/增量式PID，带积分限幅和滤波
- **ADRC控制器**: 自抗扰控制器
- **MPC控制器**: 模型预测控制（用于编队）

---

## 七、视觉功能

### 7.1 ArUco码检测
**功能**: 识别和跟踪ArUco标记，用于机器人定位和对接

**相关文件**: 
- [aruco_det.cpp](../src/asrobot/src/aruco_det/aruco_det.cpp)
- Launch文件: `roslaunch asrobot aruco_det.launch`

---

### 7.2 视觉伺服
**功能**: 基于视觉的闭环控制

**模块**:
- 单目视觉跟踪
- 立体视觉定位
- 视觉伺服控制

---

## 八、高级功能

### 8.1 多机器人编队
- 支持领航-跟随编队
- 基于MPC的分布式编队控制
- 避碰算法

---

### 8.2 路径规划
- 2D路径规划
- 3D路径规划
- 动态轨迹生成

---

### 8.3 对接控制
- 视觉引导对接
- 力控对接
- 自动抓取/释放

---

## 九、调试与开发

### 9.1 PID参数调试
详见README中的调参说明：

**内环调试**（速度环）:
```bash
pidvz 7 0.7 0.4  # 配合vz命令
```

**外环调试**（位置环）:
```bash
pidpz 0.1 0 0.002  # 配合pz命令
```

**调试模式切换**:
在 `c_sim_3d_ctrl.cpp` 中设置：
```cpp
bool test_inner_loop_only = false;  // false为外环，true为内环
```

---

### 9.2 调试话题
- `/robot1/motion_debug_topic` - 运动调试数据
- `/cmd_message` - 命令输入
- `/mc_message` - 主控消息

---

### 9.3 数据保存
系统自动保存以下数据：
- 机器人位置姿态
- 控制器输出
- 传感器数据
- 视觉检测结果

---

## 十、项目结构

### 10.1 主要功能包
- **gazebo_world**: 仿真环境（水下、陆地、水陆环境）
- **gazebo_model**: 机器人模型定义（URDF/Xacro）
- **gazebo_control**: Gazebo控制接口
- **asrobot**: 机器人控制核心
  - `motion_control`: 运动控制（PID、MPC、轨迹生成）
  - `decision_center`: 决策中心（多机协同、路径规划）
  - `man_control`: 键盘控制接口
  - `aruco_det`: ArUco视觉检测
  - `vision_module`: 视觉模块
- **gazebo_ros_link_attacher**: 机器人连接/分离插件
- **uuv_simulator**: UUV仿真器

---

### 10.2 关键代码文件
- [motion.cpp](../src/asrobot/src/motion_control/motion.cpp) - 主控制循环
- [c_sim_3d_ctrl.cpp](../src/asrobot/src/motion_control/c_sim_3d_ctrl.cpp) - 3D控制逻辑
- [c_controller_group.cpp](../src/asrobot/src/motion_control/c_controller_group.cpp) - 控制器实现
- [c_vectoredthrustcontrol.cpp](../src/asrobot/src/motion_control/c_vectoredthrustcontrol.cpp) - 推进器分配
- [f_single_robot_control.cpp](../src/asrobot/src/motion_control/f_single_robot_control.cpp) - 单机器人控制函数

---

## 十一、常见问题

### 11.1 启动失败
1. 检查ROS环境是否source: `source ~/robot/gazebo_catkin_ws/devel/setup.bash`
2. 检查Gazebo是否正常安装
3. 查看终端错误信息

### 11.2 机器人不响应命令
1. 确认在正确的终端窗口（robot_control）输入命令
2. 检查话题是否正常: `rostopic list`
3. 检查控制器节点是否运行: `rosnode list`

### 11.3 性能优化
1. 关闭Gazebo阴影: 使用`noshadow`版本的world文件
2. 减少机器人数量
3. 降低仿真频率

---

## 十二、快速开始

### 推荐学习路径

1. **入门**: 运行 `single_water.sh`，熟悉基本控制命令
2. **单机控制**: 测试xyz、yaw、pz等控制命令
3. **多机协同**: 尝试 `single_water_two_robot.sh`
4. **编队控制**: 运行 `Formation1.sh` 或 `Formation2.sh`
5. **对接功能**: 使用attach/detach服务测试机器人连接
6. **参数调优**: 根据README调试PID参数

---

**最后更新**: 2026-09-19  
**文档版本**: v1.0
