# 当前项目PID控制系统详解

本文档详细说明该水下机器人项目中PID控制的实现架构、控制流程和接口设计。

---

## 一、整体架构

### 1.1 控制层次结构

```
决策层 (decision) 
    ↓ 
运动控制层 (motion_control)
    ↓ PID计算
控制分配层 (vectored thrust control)
    ↓ 推力分配
执行器层 (thruster_control)
    ↓ PWM信号
Gazebo仿真 (推进器/关节)
```

### 1.2 核心文件位置

| 功能模块 | 文件路径 | 作用 |
|---------|---------|------|
| **PID控制器类** | `src/asrobot/include/asrobot/c_controller_group.h` (第23-49行) | 定义PID、DualLoopPID类 |
| **PID实现** | `src/asrobot/src/motion_control/c_controller_group.cpp` (第8-200行) | PID算法实现 |
| **3D组合控制** | `src/asrobot/include/asrobot/c_sim_3d_ctrl.h` (第357-458行) | C_Combine3DPidControl类定义 |
| **3D控制实现** | `src/asrobot/src/motion_control/c_sim_3d_ctrl.cpp` (第1692-2412行) | 3D PID控制逻辑 |
| **单机器人接口** | `src/asrobot/include/asrobot/f_single_robot_control.h` (第249-256行) | 对外接口函数声明 |
| **接口实现** | `src/asrobot/src/motion_control/f_single_robot_control.cpp` (第908-983行) | 高层调用接口 |
| **推进器控制** | `src/asrobot/src/gazebo_commu/thruster_control.cpp` | PWM转推力输出 |

---

## 二、PID控制器基础类

### 2.1 单环PID类 (`ControllerGroup::PID`)

**位置**：`src/asrobot/include/asrobot/c_controller_group.h` (第23-49行)

**核心成员变量**：
```cpp
class PID {
private:
    float fKp, fKi, fKd;           // PID三参数
    float fP, fI, fD;              // 当前PID各项
    float fLastP, fLastI, fLastD;  // 上次PID各项（用于平滑）
    float fCurError, fLastError, fPreError;  // 误差历史
    float fError_i;                // 积分累积
    float fError_d;                // 微分
public:
    float i_limit;                 // 积分限幅（默认15）
};
```

**核心方法**：

```cpp
// 1. 初始化
void initPID(float kp, float ki, float kd);

// 2. 设置/获取参数
void setKpPara(float Kp);
float getKp();
// ... Ki, Kd类似

// 3. 计算控制量（位置式PID + 平滑）
float pidNewCalc(float fError);  // 输入误差
float pidNewCalc(float expect, float real);  // 输入期望和实际值

// 4. 重置
void pidReset();  // 清零积分和历史误差
```

**算法核心**（位置式PID + 双平滑）：

**源码位置**：`src/asrobot/src/motion_control/c_controller_group.cpp` (第111-137行)

```cpp
float PID::pidNewCalc(float fError) {
    // 1. 积分累积（带限幅）
    fError_i += fError;
    if (fError_i > i_limit) fError_i = i_limit;
    else if (fError_i < -i_limit) fError_i = -i_limit;
    
    // 2. 微分
    fError_d = fError - fLastError;
    fLastError = fError;
    
    // 3. 计算PID三项
    fP = fKp * fError;
    fI = fKi * fError_i;
    fD = fKd * fError_d;
    
    // 4. 平滑输出（当前值和上次值的平均）
    speed1_out = (fP + fI + fD + fLastP + fLastI + fLastD) / 2;
    
    // 5. 保存本次值供下次平滑
    fLastP = fP; fLastI = fI; fLastD = fD;
    
    return speed1_out;
}
```

**特点**：
- 采用**位置式PID**而非增量式
- **双重平滑**：当前PID输出与上次输出平均
- **积分抗饱和**：i_limit限幅

---

### 2.2 双环级联PID类 (`DualLoopPID`)

**位置**：`src/asrobot/include/asrobot/c_controller_group.h` (第56-122行)

**结构**：
```cpp
class DualLoopPID {
public:
    PID outer_loop_pid_;  // 外环（位置环）
    PID inner_loop_pid_;  // 内环（速度环）
    
    // 双环计算
    float calculate(float target_pos, float current_pos, float current_vel);
    
    // 仅内环计算（调试用）
    float calculateInnerLoopOnly(float desired_vel, float current_vel);
};
```

**控制流程**：

**源码位置**：`src/asrobot/src/motion_control/c_controller_group.cpp` (第174-192行)

```
目标位置 ──┐
           ├──[外环PID]──> 期望速度 ──┐
当前位置 ──┘                         ├──[内环PID]──> 力/力矩
                       当前速度 ─────┘
```

**实现代码**：
```cpp
float DualLoopPID::calculate(float target_pos, float current_pos, float current_vel) {
    // 1. 外环：位置误差 -> 期望速度
    float pos_error = target_pos - current_pos;
    float desired_vel = outer_loop_pid_.pidNewCalc(pos_error);
    
    // 2. 速度限幅（防止期望速度过大）
    if (desired_vel > 0.08) desired_vel = 0.08;
    else if (desired_vel < -0.08) desired_vel = -0.08;
    
    // 3. 内环：速度误差 -> 最终力
    float vel_error = desired_vel - current_vel;
    float final_force = inner_loop_pid_.pidNewCalc(vel_error);
    
    return final_force;
}
```

---

## 三、3D组合PID控制系统

### 3.1 C_Combine3DPidControl 类

**位置**：`src/asrobot/include/asrobot/c_sim_3d_ctrl.h` (第357-458行)

**作用**：统一管理X、Y、Z、Yaw四个自由度的PID控制，支持H型/X型机器人形态。

**核心成员**：
```cpp
class C_Combine3DPidControl {
public:
    // 控制模式
    enum ControlMode {
        SINGLE_LOOP,  // 单环：位置 -> 力
        DUAL_LOOP     // 双环：位置 -> 速度 -> 力
    };
    
    // 调试模式
    enum ControlDebugMode {
        NAV_NORMAL,        // 正常导航
        DEBUG_Z_ONLY,      // 只调深度
        DEBUG_YAW_ONLY,    // 只调航向
        DEBUG_X_ONLY,      // 只调前进
        DEBUG_DEPYAW_ONLY  // 深度+航向
    };
    
    // 矢量推力控制器（负责力分配）
    C_VectoredThrustControl vec_ctrl;
    
private:
    ControlMode current_mode_;         // 当前控制模式
    ControlDebugMode current_debug_mode_;  // 当前调试模式
    
    // 共享数据（从updateControl接收）
    float cur_p_x_, cur_p_y_, cur_p_z_, cur_yaw_;
    float cur_v_x_, cur_v_y_, cur_v_z_;
    float exp_p_x_, exp_p_y_, exp_p_z_, exp_yaw_;
    int shape_;  // SHAPE_H 或 SHAPE_X
};
```

### 3.2 控制主流程

**入口函数**：`updateControl`

**源码位置**：`src/asrobot/src/motion_control/c_sim_3d_ctrl.cpp` (第1739-1788行)

**调用链**：
```
updateControl(...)
    ├─> 保存输入参数到成员变量
    └─> updatePidControl()
            ├─> 根据shape和mode调用：
            │   ├─ handleShapeH_SingleLoop_Control()
            │   ├─ handleShapeH_DualLoop_Control()
            │   ├─ handleShapeX_SingleLoop_Control()
            │   └─ handleShapeX_DualLoop_Control()
            └─> printDebugInfo()
```

---

## 四、H型单环PID控制详解

### 4.1 H型机器人形态

**H型**：4个推进器呈"H"字形水平布局，适合水面/浅水航行。

**控制自由度**：
- **X方向**：前进/后退（4个推进器同向）
- **Yaw**：转向（左右推进器差速）
- **Z**：深度（无直接控制，依赖浮力/姿态）

### 4.2 NAV_NORMAL模式（正常导航）

**源码位置**：`src/asrobot/src/motion_control/c_sim_3d_ctrl.cpp` (第1907-1994行)

**控制逻辑**：

```cpp
void C_Combine3DPidControl::handleShapeH_SingleLoop_Control() {
    if (current_debug_mode_ == NAV_NORMAL) {
        // 1. 计算世界坐标系下的位置误差
        float error_vec_x = exp_p_x_ - cur_p_x_;
        float error_vec_y = exp_p_y_ - cur_p_y_;
        
        // 2. 计算到目标的直线距离（作为前进控制器输入）
        float distance_error = sqrt(error_vec_x^2 + error_vec_y^2);
        
        // 3. 计算目标航向角（朝向目标点）
        float target_yaw = atan2(error_vec_y, error_vec_x) * 180/π;
        
        // 4. 航向误差（带环绕处理）
        float yaw_error = target_yaw - cur_yaw_;
        if (yaw_error > 180) yaw_error -= 360;
        if (yaw_error < -180) yaw_error += 360;
        
        // 5. PID计算控制量
        float forward_force = vec_ctrl.mcSpeedPID.pidNewCalc(distance_error);
        float turning_torque = vec_ctrl.mcYawPID.pidNewCalc(yaw_error);
        
        // 6. 推力分配（调用矢量控制器）
        vec_ctrl.space3DControl(SHAPE_H, forward_force, 0, 0, turning_torque);
    }
}
```

**关键点**：
- **目标航向自动计算**：`target_yaw = atan2(Δy, Δx)`，机器人自动朝向目标点
- **两个独立PID**：
  - 前进PID：输入=距离误差，输出=前进力
  - 转向PID：输入=航向误差，输出=转向力矩
- **力分配**：`space3DControl`将虚拟力分配到4个推进器

### 4.3 DEBUG_YAW_ONLY 模式（仅调航向）

**源码位置**：`src/asrobot/src/motion_control/c_sim_3d_ctrl.cpp` (第1836-1854行)

```cpp
case DEBUG_YAW_ONLY: {
    float target_yaw = debug_target_value_;  // 固定目标航向
    float yaw_error = target_yaw - cur_yaw_;
    float torque = vec_ctrl.mcYawPID.pidNewCalc(yaw_error);
    
    vec_ctrl.space3DControl(SHAPE_H, 0, 0, 0, torque);  // 仅输出转向力矩
}
```

**用途**：单独调试Yaw PID参数，其他自由度保持0输出。

---

## 五、矢量推力控制（力分配）

### 5.1 space3DControl 函数

**作用**：将虚拟控制力（Fx, Fy, Fz, Tyaw）分配到4个推进器。

**位置**：`C_VectoredThrustControl::space3DControl`（在 `c_vectoredthrustcontrol.cpp` 中）

**调用示例**：
```cpp
vec_ctrl.space3DControl(
    SHAPE_H,         // 机器人形态
    forward_force,   // X方向虚拟力
    0,               // Y方向虚拟力（H型无侧向）
    0,               // Z方向虚拟力（H型无垂直推进器）
    turning_torque   // Yaw方向力矩
);
```

**输出**：
```cpp
vec_ctrl.thrust1, thrust2, thrust3, thrust4  // 4个推进器的推力
```

### 5.2 推力到PWM

**流程**：
```
space3DControl计算推力
    ↓
pwm_msg发布到topic "pwm_message"
    ↓
thruster_control节点订阅
    ↓
转换为Gazebo推进器输入（FloatStamped）
    ↓
发布到 "thrusters/0~3/input"
```

**源码位置**：`src/asrobot/src/gazebo_commu/thruster_control.cpp` (第26-38行)

```cpp
void pwmcallback(const asrobot::pwm::ConstPtr& msg) {
    rotor.data = msg->rotor_speed1;
    Propeller1_pub.publish(rotor);  // 发布到thrusters/0/input
    // ... 重复发布speed2~4
}
```

---

## 六、高层调用接口

### 6.1 位置控制接口

**函数**：`PidXYZControl_H`

**位置**：`src/asrobot/src/motion_control/f_single_robot_control.cpp` (第921-929行)

**原型**：
```cpp
void PidXYZControl_H(
    float exp_p_x,    // 目标位置X（世界坐标系）
    float exp_p_y,    // 目标位置Y
    float exp_p_z,    // 目标深度Z
    float cur_p_x,    // 当前位置X
    float cur_p_y,    // 当前位置Y
    float cur_p_z,    // 当前深度Z
    float cur_yaw     // 当前航向（度）
);
```

**实现**：
```cpp
void PidXYZControl_H(...) {
    combine3DPidCtrl.setDebugMode(NAV_NORMAL);
    combine3DPidCtrl.updateControl(SHAPE_H,
                                   exp_p_x, exp_p_y, 0, exp_p_z,
                                   cur_p_x, cur_p_y, cur_yaw, cur_p_z);
}
```

**使用示例**：
```cpp
// 在决策层或运动规划层调用
PidXYZControl_H(
    10.0, 5.0, -2.0,   // 目标：(10, 5, 深度2m)
    0.0, 0.0, 0.0,     // 当前：原点
    90.0               // 当前航向：90度
);
```

### 6.2 单自由度控制接口

**深度控制**：
```cpp
void PidDepthControl_X(float exp_p_z, float cur_p_z);
```
**源码位置**：`f_single_robot_control.cpp` (第976-983行)

**航向控制**：
```cpp
void PidYawControl_H(float exp_yaw, float cur_yaw);
```
**源码位置**：`f_single_robot_control.cpp` (第966-973行)

### 6.3 PID参数设置接口

**位置**：`src/asrobot/include/asrobot/f_single_robot_control.h` (第249-256行)

```cpp
void setYawPIDPmt(float k_p, float k_i, float k_d);    // 设置Yaw PID
void setDepthPIDPmt(float k_p, float k_i, float k_d);  // 设置深度PID
void setSpeedPIDPmt(float k_p, float k_i, float k_d);  // 设置速度PID
void setXPIDPmt(float k_p, float k_i, float k_d);      // 设置X PID
void setYPIDPmt(float k_p, float k_i, float k_d);      // 设置Y PID
```

**实现位置**：`f_single_robot_control.cpp` (第856-900行左右)

---

## 七、控制周期与频率

**运动控制循环频率**：
```cpp
const int iMotionLoopRate = 5;  // 5Hz
```
**位置**：`f_single_robot_control.h` (第91行)

**推进器控制频率**：
```cpp
ros::Rate loop_rate(1000);  // 1000Hz
```
**位置**：`thruster_control.cpp` (第46行)

---

## 八、为模糊PID准备的接口建议

### 8.1 模糊PID应该替换的位置

当前PID控制器实例位于 `vec_ctrl` 中：
```cpp
// C_VectoredThrustControl 类内部
PID mcSpeedPID;   // 前进/速度PID
PID mcYawPID;     // 转向PID
PID mcDepthPID;   // 深度PID
```

**建议模糊PID接口**：
```cpp
class FuzzyPID {
public:
    // 初始化
    void init(float kp_base, float ki_base, float kd_base);
    
    // 设置模糊规则参数
    void setFuzzyParams(float e_max, float ec_max, 
                        float delta_kp_max, float delta_ki_max, float delta_kd_max);
    
    // 主计算函数（输入误差和误差变化率）
    float calculate(float error, float error_derivative);
    
    // 兼容接口（自动计算误差变化率）
    float calculate(float error);  // 内部保存上次error
    
    // 重置
    void reset();
    
    // 获取当前参数（用于调试）
    float getCurrentKp();
    float getCurrentKi();
    float getCurrentKd();
};
```

### 8.2 替换方式

**方式1**：直接替换vec_ctrl中的PID实例
```cpp
// 修改 c_vectoredthrustcontrol.h
FuzzyPID mcSpeedFuzzyPID;
FuzzyPID mcYawFuzzyPID;
FuzzyPID mcDepthFuzzyPID;
```

**方式2**：作为可选模式（推荐）
```cpp
class C_VectoredThrustControl {
public:
    enum PIDType { CLASSIC_PID, FUZZY_PID };
    void setPIDType(PIDType type);
    
private:
    PIDType current_pid_type_;
    PID classic_speed_pid_;
    FuzzyPID fuzzy_speed_pid_;
    
    float getSpeedControl(float error) {
        if (current_pid_type_ == CLASSIC_PID)
            return classic_speed_pid_.pidNewCalc(error);
        else
            return fuzzy_speed_pid_.calculate(error);
    }
};
```

---

## 九、总结

### 9.1 当前PID控制特点
1. **分层清晰**：决策 → 控制 → 分配 → 执行
2. **支持双环**：位置环+速度环级联
3. **多形态支持**：H型/X型机器人
4. **调试友好**：DEBUG模式可单独调试各自由度
5. **平滑输出**：双重平滑减少抖动

### 9.2 模糊PID集成建议
- **文件位置**：`src/asrobot/include/asrobot/c_fuzzy_pid.h`
- **实现文件**：`src/asrobot/src/motion_control/c_fuzzy_pid.cpp`
- **接口兼容**：保持与PID类相同的 `calculate(error)` 接口
- **测试方式**：先用DEBUG模式单独调试Yaw/Depth，再集成到NAV_NORMAL

### 9.3 关键文件清单
| 文件 | 行数范围 | 内容 |
|-----|---------|------|
| `c_controller_group.h` | 23-122 | PID/DualLoopPID类定义 |
| `c_controller_group.cpp` | 8-200 | PID算法实现 |
| `c_sim_3d_ctrl.h` | 357-458 | 3D组合控制类定义 |
| `c_sim_3d_ctrl.cpp` | 1827-1994 | H型单环控制逻辑 |
| `f_single_robot_control.h` | 249-256 | 参数设置接口 |
| `f_single_robot_control.cpp` | 921-983 | 高层调用接口 |
| `thruster_control.cpp` | 26-38 | PWM发布 |

---

**文档版本**：v1.0  
**创建时间**：2026-10-17  
**适用项目**：ROS Gazebo水下机器人仿真平台
