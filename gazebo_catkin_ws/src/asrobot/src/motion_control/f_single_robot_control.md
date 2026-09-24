# `f_single_robot_control.cpp` 学习说明

> 对应源文件：[`f_single_robot_control.cpp`](f_single_robot_control.cpp)  
> 头文件：[`f_single_robot_control.h`](../../include/asrobot/f_single_robot_control.h)

本文按照源文件从上到下的顺序，解释这个文件中的变量、函数和控制流程。目标不是逐字符翻译代码，而是帮助你建立一个清晰的认识：**上层给出期望位置/速度，控制器计算广义力，最后再把广义力分配给四个推进器或腿部机构。**

---

## 1. 文件的整体职责

这个文件不是某一个 PID、ADRC 或 MPC 算法的底层实现，而是一个**单机控制功能封装层**。它把其他类中的控制器组合起来，给 ROS 节点或其他业务代码提供比较容易调用的函数，例如：

- 设置 PID、ADRC、S 面控制器参数；
- 切换当前控制模式；
- 设置机器人腿型（H 型、X 型等）；
- 直接给机器人施加力和转矩；
- 控制腿端位置、腿端角度和步态；
- 进行 X/Y/Z/航向角的三维位置或速度控制；
- 用 MPC 跟踪参考轨迹；
- 保存辨识、对接和调试数据。

可以把控制过程抽象为：

```text
期望状态 + 当前状态
          │
          ▼
  PID / ADRC / S面 / MPC
          │
          ▼
   广义控制量：Fx、Fy、Fz、Yaw力矩
          │
          ▼
  四个推进器的推力 / 转速
          │
          ▼
        仿真机器人
```

其中 `f_single_robot_control.cpp` 主要负责“**调用谁、传什么参数、把结果保存在哪里**”；具体控制算法主要在 `c_sim_3d_ctrl.h`、`c_controller_group.h`、`c_vectoredthrustcontrol.h`、`MPCController.h` 等文件中实现。

---

## 2. 阅读前需要知道的几个概念

### 2.1 位置、速度和姿态

代码中常见的变量命名如下：

| 前缀/名称 | 含义 |
|---|---|
| `exp_` | expected，期望值，例如 `exp_p_x` 是期望 X 位置 |
| `cur_` | current，当前反馈值，例如 `cur_v_z` 是当前 Z 速度 |
| `p` | position，位置 |
| `v` | velocity，速度 |
| `yaw` | 航向角，绕 Z 轴旋转 |
| `roll` | 横滚角 |
| `pitch` | 俯仰角 |
| `_force_x` | X 方向力 |
| `_torque_z` | 绕 Z 轴的转矩 |

水下机器人控制中，通常会重点控制：

- 水平面位置：`x`、`y`；
- 深度或 Z 方向位置：`z`；
- 航向角：`yaw`；
- 速度：`v_x`、`v_y`、`v_z`、`v_yaw`。

### 2.2 腿型/推进形态

头文件中定义了几种形态常量：

```cpp
SHAPE_H   = 1;
SHAPE_H_T = 2;
SHAPE_X   = 3;
SHAPE_G   = 4;
```

不同腿型会改变四个推进器的正负方向、控制分配方式或腿部姿态。这里的 `H`、`X`、`G` 不是 C++ 语法概念，而是机器人机械/推进器布局的工作形态。

### 2.3 控制器对象的层次

源文件里有多个“组合控制器”对象，初学时容易混淆：

| 对象 | 大致职责 |
|---|---|
| `vec_ctrl` | 直接进行推进器矢量控制、腿型控制、力到转速转换 |
| `comb3Dctrl` | 组合式 ADRC/S 面三维位置控制 |
| `comb3DSpeedCtrl` | 组合式 ADRC 速度控制 |
| `comb3DPositionCtrl` | 旧版本位置 ADRC 控制 |
| `comb3DPositionCtrl_v2` | 新版本位置 ADRC 控制，当前多个调试函数使用 |
| `combine3DPidCtrl` | 组合式 PID 控制，当前 PID 接口使用 |
| `gait_ctrl` | 步态和腿部动作控制 |
| `MPC_tracking` | MPC 轨迹跟踪控制器 |

源文件中的注释特别提醒：`comb3Dctrl` 内部也有一个 `vec_ctrl`，不要和本文件的全局 `vec_ctrl` 混淆。

---

## 3. 第 1 行：头文件包含

### `#include "../../include/asrobot/f_single_robot_control.h"`

这个 `.cpp` 文件直接包含对应的头文件。头文件中声明了：

- ROS 消息和标准库；
- Eigen 矩阵类型；
- PID、ADRC、S 面、步态、推进器控制器；
- 本文件中所有对外函数的声明；
- 控制模式和腿型常量。

通常推荐 `.cpp` 和 `.h` 保持对应：头文件写声明，源文件写实现。阅读本文件时，如果不知道某个类或常量的具体含义，应继续跳转到头文件和相关类定义。

---

## 4. 多机参考轨迹相关的全局变量（约第 14～28 行）

```cpp
int last_ref_num = 0;
int reference_Point_Num;
float vs_reference_PositionX;
float vs_reference_PositionY;
Eigen::MatrixXd robot1_reference_trajectory;
Eigen::MatrixXd robot2_reference_trajectory;
Eigen::MatrixXd robot3_reference_trajectory;
Eigen::MatrixXd virtureRobot_reference_trajectory;
```

这部分变量虽然位于“单机控制”文件中，但实际上为多机器人编队/MPC预留：

- `robot1_reference_trajectory`、`robot2_reference_trajectory`、`robot3_reference_trajectory`：不同机器人的参考轨迹；
- `virtureRobot_reference_trajectory`：虚拟机器人或编队中心的参考轨迹；
- `reference_Point_Num`：参考点数量；
- `last_ref_num`：记录上一次参考点编号；
- `vs_reference_PositionX/Y`：编队中心或虚拟参考位置。

`Robot1_c_position`、`Robot2_c_position`、`Robot3_c_position` 保存三个机器人的当前位置，后面的 `f_mpcTracking()` 会用它们计算编队中心位置。

### 学习重点

这些变量的类型是 `Eigen::MatrixXd`，表示运行时大小可变的 double 矩阵。轨迹通常按“每一行一个时刻/参考点、每一列一个状态量”存储。具体列含义必须结合轨迹生成代码和 `MPCController` 的实现确认。

---

## 5. 单机控制全局变量（约第 30～150 行）

### 5.1 控制频率

```cpp
const int iMotionLoopRate = 30;
```

表示运动控制输出频率。注释中曾经写过 5 Hz，但当前值是 30，说明注释已经过时。实际周期还要看调用这个变量的 ROS 节点如何使用它。

### 5.2 力和推进器输出缓存

```cpp
float force_x, force_y, force_z, force_yaw;
float force_1, force_2, force_3, force_4;
float control_cmd_x, control_cmd_yaw;
float four_thrust1, four_thrust2, four_thrust3, four_thrust4;
```

这些变量主要用于保存中间结果和调试输出：

- `force_x/y/z`：三个方向的合力；
- `force_yaw`：航向控制相关力矩或控制量；
- `force_1~force_4`：四个推进器的中间分配结果；
- `four_thrust1~four_thrust4`：最终记录的四个推进器推力；
- `control_cmd_x`、`control_cmd_yaw`：组合速度控制器输出的 X 方向和偏航控制量。

要注意：同名的局部变量也会在 `f_mpcTracking()` 中出现。局部变量会遮蔽全局变量，所以调试时要看清楚作用域。

### 5.3 数据记录对象

```cpp
SaveData::C_SaveData save_debugdata(...);
SaveData::C_SaveData save_debugdata1(...);
SaveData::C_SaveData identi_data(...);
SaveData::C_SaveData this_robot_docking_data(...);
```

这些对象负责把控制过程写入文本文件：

- `save_debugdata`：MPC 状态和控制量；
- `save_debugdata1`：MPC 当前状态；
- `identi_data`：系统辨识数据；
- `this_robot_docking_data`：对接过程的完整数据。

路径是硬编码的绝对路径，换机器或换工作空间后很可能不可用。这是当前代码维护时需要优先改进的地方，建议改成 ROS 参数或相对/可配置路径。

### 5.4 早期 PID 调试变量

```cpp
int mode = 100;
int dir = 0;
float e_depthvalue, e_navyaw;
float c_depthvalue, c_navyaw;
float e_x, e_y, c_x, c_y;
float gait_R;
int iter_num = 0;
```

这些变量服务于旧的手动控制、腿式步态和早期调试流程：

- `mode`：步态是否运行；
- `dir`：前进、后退、左转、右转；
- `e_*`：旧命名中的期望值；
- `c_*`：旧命名中的当前值；
- `gait_R`：步态转弯半径或步态参数；
- `iter_num`：MPC 当前跟踪点编号。

`MPC_real_force` 保存 MPC 上一次的控制输入，尤其在增量式 MPC 中会作为上一时刻输入使用。

### 5.5 MPC 参数

```cpp
double samplingtime = 0.05;
const int Np = 15;
const int Nc = 8;
bool relax_mode = 1;
const int X_num = 5;
const int Y_num = 5;
const int U_num = 2;
```

含义如下：

| 参数 | 含义 |
|---|---|
| `samplingtime` | MPC 离散采样时间，当前为 0.05 s |
| `Np` | 预测域长度，未来预测 15 步 |
| `Nc` | 控制域长度，未来优化 8 步控制动作 |
| `relax_mode` | 是否允许约束松弛 |
| `X_num` | 状态维数，当前为 5 |
| `Y_num` | 输出维数，当前为 5 |
| `U_num` | 控制输入维数，当前为 2 |

在 `f_mpcTracking()` 中，5 维状态大致是：

```text
[x, y, yaw(rad), 水平速度大小, yaw速度]
```

2 维控制输入是：

```text
[前进方向控制力, 航向控制力矩]
```

`q11~q55` 是 MPC 代价函数的权重。当前代码中启用的是“有山的曲线参数”注释下的一组值：

```cpp
q11 = 1;
q22 = 5;
q33 = 1;
q44 = 10;
q55 = 1;
```

通常权重越大，MPC 越重视对应状态误差，但准确的列对应关系仍要结合 `MPCController` 实现确认。

---

## 6. `initMpcOsqp()`：初始化 MPC（约第 157～200 行）

### 函数作用

`initMpcOsqp()` 给 ASR 机器人建立一个二维动力学模型，并用它初始化 `MPC_tracking`。虽然函数名写的是 OSQP，但 `MPCController` 的构造方式还区分了增量模式和非增量模式，底层求解器由相关库实现。

### 6.1 缩放系数和动力学矩阵

```cpp
float beishu = 0.00031 * 25 * 25;
MassMatrix << ...;
HydroNonLinear << ...;
HydroLinear << ...;
```

这里建立了三个 2×2 矩阵：

- `MassMatrix`：质量/惯性项；
- `HydroNonLinear`：非线性水动力阻尼或阻力项；
- `HydroLinear`：线性水动力项。

`beishu` 是一个经验缩放系数，用于把原始模型参数缩放到当前仿真或实验的量纲范围。矩阵中的数值不是通用常数，而是本项目机器人模型的辨识/调参结果。

然后复制给：

```cpp
ASR_simulation_model.MassMatrix = MassMatrix;
ASR_simulation_model.HydroNonLinMatrix = HydroNonLinear;
ASR_simulation_model.HydroLinMatrix = HydroLinear;
```

### 6.2 控制输入约束

```cpp
const Vector2d umax(5, 4);
const Vector2d umin(-1, -4);
const Vector2d dumax(0.5, 1);
const Vector2d dumin(-0.5, -1);
```

- `umax/umin`：控制输入绝对值上下限；
- `dumax/dumin`：控制输入变化量上下限，仅在 `delta_mode` 下使用。

当前 `delta_mode = 1`，因此构造 MPC 时会传入 `dumax` 和 `dumin`。增量式 MPC 优化的不是直接输入 `u`，而是输入变化量 `Δu`，这样通常可以限制控制动作的突变。

### 6.3 构造 MPC 对象

```cpp
if (!delta_mode) {
    MPC_tracking = MPCController(..., umax, umin);
}
if (delta_mode) {
    MPC_tracking = MPCController(..., umax, umin, dumax, dumin);
}
```

这里根据 `delta_mode` 选择不同构造函数。`Linear_mode` 在 `f_mpcTracking()` 中还会选择两种线性化/更新方式。

### 学习建议

阅读这个函数时，不必先纠结矩阵每一个数字。先理解顺序：

1. 准备模型矩阵；
2. 设置输入约束；
3. 根据是否使用增量输入构造 MPC；
4. 后续由 `f_mpcTracking()` 每个控制周期更新当前状态并求解。

---

## 7. 控制器对象和期望/反馈变量（约第 204～248 行）

### 7.1 控制器对象

```cpp
C_Combine3DControl comb3Dctrl;
C_Combine3DSpeedControl comb3DSpeedCtrl;
C_VectoredThrustControl vec_ctrl;
C_GaitControl gait_ctrl;
C_Combine3DPositionControl comb3DPositionCtrl;
C_Combine3DPositionControl_v2 comb3DPositionCtrl_v2;
CombineCtrl::C_Combine3DPidControl combine3DPidCtrl;
```

这些是全局对象，程序启动时构造，之后由本文件中的包装函数反复调用。使用全局对象的优点是控制器内部状态（积分项、观测器状态、历史误差等）可以在多个周期之间保留；缺点是依赖关系隐蔽、线程安全性差、测试不方便。

### 7.2 ADRC 调试变量

```cpp
float exp_p_x, exp_v_x;
float cur_p_x, cur_v_x;
...
```

这些变量将期望状态和实际状态分开保存。很多 ROS 回调会先更新 `cur_*`，命令回调会更新 `exp_*`，真正的控制函数再读取它们。

### 7.3 相机、深度计和螺旋轨迹变量

```cpp
float cam_exp_p_x, cam_exp_p_y, cam_exp_p_z;
float cam_cur_p_x, cam_cur_p_y, cam_cur_p_z;
float depth_sensor_data;
float heli_traj_radius;
```

这部分服务于视觉对接、深度传感器和螺旋下降轨迹。它们在本文件中主要作为数据接口或记录变量，真正的视觉处理逻辑在其他节点/控制类中。

---

## 8. 数据保存函数（约第 251～324 行）

### 8.1 `save_identi_data(...)` 两个重载

```cpp
void save_identi_data(..., float fx, float fyaw);
void save_identi_data(..., float f1, float f2, float f3, float f4);
```

这两个函数同名但参数数量不同，是 C++ 的**函数重载**：

- 8 个浮点量版本：保存位置、速度、航向、航向速度、X 力和 Yaw 力矩；
- 10 个浮点量版本：保存位置、速度、航向、航向速度和四个推进器输入。

函数本身不做计算，只是打印提示，然后转发给 `identi_data.write_data(...)`。

### 8.2 被注释掉的旧保存函数

中间有一大块 `/* ... */` 注释，包含旧版本的以下函数：

| 被注释的函数 | 原本用途 | 当前状态 |
|---|---|---|
| `save_this_robot_data_H_mode(...)` | 保存 H 型运动数据 | 不编译、不运行 |
| `save_this_robot_data_H_mode_global_cam(...)` | 保存带全局相机数据的 H 型运动数据 | 不编译、不运行 |
| `save_this_robot_data_X_mode(...)` | 保存 X 型运动数据 | 不编译、不运行 |
| `save_thrust_data(...)` | 保存指定时刻的四推进器推力 | 不编译、不运行 |
| 注释块中的旧版 `save_identi_data(...)` | 以四推进器输入为辨识数据保存 | 不编译、不运行 |
| `save_this_robot_data_X_mode_2(...)` | 保存更完整的 X 型位置、参考位置、控制力和推力 | 不编译、不运行 |

它们当前不会编译，也不会运行。阅读时把它们当作历史接口参考即可，不要误以为这些函数现在可调用。

### 8.3 `save_this_robot_docking_data(...)`

这个函数保存一次完整的对接数据，包括：

1. 机器人位置：`x/y/z`；
2. 机器人姿态：`roll/pitch/yaw`；
3. 深度计数据；
4. 螺旋轨迹半径；
5. 参考位置和目标航向；
6. 控制器输出的 `force_x/torque_yaw/force_z`；
7. 四个推进器推力；
8. 四条腿的角度。

它只是对 `this_robot_docking_data.write_data_3(...)` 的包装，方便上层用一条函数调用记录对接过程。

---

## 9. PID 参数设置函数（约第 836～906 行）

### 9.1 `setYawPIDPmt()`、`setPitchPIDPmt()`、`setXPIDPmt()`、`setYPIDPmt()`、`setDepthPIDPmt()`、`setSpeedPIDPmt()`

这些函数的结构完全相同：

```cpp
控制器.setKpPara(_k_p);
控制器.setKiPara(_k_i);
控制器.setKdPara(_k_d);
控制器.enable = true;
```

分别设置：

| 函数 | 负责的 PID |
|---|---|
| `setYawPIDPmt` | 航向角 |
| `setPitchPIDPmt` | 俯仰角 |
| `setXPIDPmt` | X 位置/方向 |
| `setYPIDPmt` | Y 位置/方向 |
| `setDepthPIDPmt` | 深度 |
| `setSpeedPIDPmt` | 速度 |

这里使用的是 `combine3DPidCtrl.vec_ctrl` 内部的 PID，而不是文件顶部的全局 `vec_ctrl`。设置完参数后启用控制器，最后打印当前参数。

### 9.2 `setVelDepPIDPmt()` 和 `setPosDepPIDPmt()`

这两个函数用于深度双环 PID：

- `setVelDepPIDPmt()`：设置深度速度内环；
- `setPosDepPIDPmt()`：设置深度位置外环。

典型双环结构是：

```text
深度位置误差 ──> 外环 PID ──> 期望深度速度
                                  │
当前深度速度 ──────────────────────┘
                                  ▼
                            内环 PID ──> 推力
```

### 9.3 `simCombinePID_3DCtrl(...)`

这是一个 PID 三维控制入口。它接收期望位置、期望速度和当前状态，但当前实现调用 `combine3DPidCtrl.updateControl(...)` 时，主要把位置和姿态参数传入，期望速度参数并没有直接参与本层计算，具体处理由 `updateControl()` 决定。

当前固定使用：

```cpp
SHAPE_X
```

因此它是 X 型形态的组合 PID 控制接口。

### 9.4 便捷 PID 控制接口

#### `PidXYZControl_H(...)`

H 型三维位置控制。函数内部将调试模式设为 `NAV_NORMAL`，但不使用调用者传入的目标航向，而是把 yaw 参数传为 0，表示由组合控制器按正常导航逻辑处理。

#### `PidXYZControl_X(...)` 两个重载

X 型三维位置控制，有两个版本：

- 7 参数版本：只传位置和当前 yaw；
- 8 参数版本：额外传入当前 Z 速度 `cur_v_z`，适合深度速度内环或需要速度反馈的场景。

#### `PidDepVelControl_X(...)`

只控制 X 型机器人的深度速度。位置、XY、Yaw 等无关量都填 0，只给出期望 Z 速度和当前 Z 速度。

#### `PidDepthYawControl_H(...)`

H 型同时控制深度和航向。

#### `PidYawControl_H(...)`

H 型只控制航向角。

#### `PidDepthControl_X(...)`

X 型只控制深度位置。

这些便捷函数的意义是：上层调用时不必每次手工给控制器填写一长串无关参数。

---

## 10. ADRC 位置/姿态参数设置（约第 406～448 行）

### `setPxADRCPmt()`、`setVxADRCPmt()`、`setPyADRCPmt()`、`setVyADRCPmt()`、`setPzADRCPmt()`、`setYawADRCPmt()`

这些函数设置 `comb3Dctrl` 内部各个 ADRC 通道的参数：

| 函数 | 通道 |
|---|---|
| `setPxADRCPmt` | X 位置 |
| `setVxADRCPmt` | X 速度 |
| `setPyADRCPmt` | Y 位置 |
| `setVyADRCPmt` | Y 速度 |
| `setPzADRCPmt` | Z 位置/深度 |
| `setYawADRCPmt` | 航向角 |

参数 `_k_p`、`_k_d`、`_band`、`_b_0` 的一般含义：

- `_k_p`：比例相关增益；
- `_k_d`：微分相关增益；
- `_band`：观测器/控制器带宽；
- `_b_0`：名义输入增益或补偿因子。

`setYawADRCPmt()` 额外把 `m_r_0` 设置为 100。这是航向控制的过渡过程参数，属于本项目调参约定，不是所有 ADRC 实现都必须这样设置。

---

## 11. ADRC 速度参数设置（约第 459～523 行）

速度控制器每个方向通常有两组参数：

### 11.1 `set*TDandESOPmt(...)`

例如：

```cpp
setSpeedXTDandESOPmt(sample_time, r0, w, b0);
```

这些函数调用对应 ADRC 的 TD（跟踪微分器）和 ESO（扩张状态观测器）参数设置接口。分别覆盖：

- X 速度：`setSpeedXTDandESOPmt`；
- Y 速度：`setSpeedYTDandESOPmt`；
- Yaw 速度：`setSpeedYawTDandESOPmt`；
- Z 速度：`setSpeedZTDandESOPmt`。

参数大致含义：

- `_sample_time`：离散采样周期；
- `_r_0`：跟踪过渡速度/加速度相关参数；
- `_w`：观测器带宽；
- `_b_0`：输入增益补偿。

### 11.2 `set*NLSEFPmt(...)`

这些函数配置 ADRC 的非线性误差反馈（NLSEF）部分：

- `setSpeedXNLSEFPmt`；
- `setSpeedYNLSEFPmt`；
- `setSpeedYawNLSEFPmt`；
- `setSpeedZNLSEFPmt`。

参数为 `_k_p`、`_k_i`、`_k_d` 和积分限幅 `_i_limit`。

### 11.3 `setYawTDandESOPmt(...)`

名字虽然包含 `TDandESO`，但实际调用的是：

```cpp
comb3DSpeedCtrl.adrc_yaw_ctrl.SetadrcPmt(...);
```

也就是说它使用了航向控制器自己的参数接口，而不是速度 X/Y/Z 控制器相同的 `SetTDandESOPmt()`。同时把 `m_r_0` 固定为 100。学习时应以实际调用为准，不要只根据函数名猜实现。

---

## 12. S 面参数设置和模式切换（约第 533～576 行）

### 12.1 `setPxSSPmt()`、`setVxSSPmt()`、`setPySSPmt()`、`setVySSPmt()`、`setPzSSPmt()`、`setYawSSPmt()`

这些函数配置 `comb3Dctrl` 内部的 S 面控制器。所有函数都调用：

```cpp
setSSCv2Pmt(_k_p, _k_i, _k_d, _outside_gain);
```

`_outside_gain` 是外部增益，也被注释说明为控制器输出限幅相关参数，不能随便设为 0。S 面控制器的数学细节不在本文件，想深入学习应继续阅读对应控制器类。

### 12.2 `setComb3DCtrlMode(int _mode)`

```cpp
comb3Dctrl.debug_mode = _mode;
```

这个函数只做一件事：修改组合三维控制器的模式。可传入头文件/控制类中定义的常量，例如：

| 模式 | 作用 |
|---|---|
| `ALL_CONTROL_MODE_SINGLE_INTEG` | 单积分/编队相关控制 |
| `ALL_CONTROL_MODE_DOUBLE_INTEG` | 双积分/编队相关控制 |
| `ADJUST_PARAMETERS_MODE` | 调参模式 |
| `MPC_MODE` | MPC 模式 |

它不立即计算控制量，只改变下一次控制函数调用时的分支。

---

## 13. 机器人基本运动控制函数（约第 590～730 行）

### 13.1 `LegShape_H()`

```cpp
vec_ctrl.yawLocomMode_FB();
```

选择 H 型/前后方向推进形态。函数名是项目约定，实际动作由 `yawLocomMode_FB()` 决定。

### 13.2 `LegShape_H_T()`

```cpp
vec_ctrl.yawLocomMode_LR();
```

选择 H_T 型或左右方向推进形态。

### 13.3 `LegShape_X()`

```cpp
vec_ctrl.yawLocomMode_X();
```

选择 X 型腿/推进器形态。

### 13.4 `LegShape_G()`

```cpp
vec_ctrl.shiftMode_G();
```

切换到 G 型工作形态。这个函数在当前 `.cpp` 中有实现，但要确认头文件是否也声明了它；如果外部文件要调用，应确保声明一致。

### 13.5 `simMotorForce(...)`

这是对底层 `vec_ctrl.motorforce(...)` 的一层包装，接收：

```text
_force_x, _force_y, _force_z, _torque_z,
_roll, _yaw, _pitch
```

分别是三个方向力、绕 Z 轴转矩和当前姿态。底层控制器通常需要姿态信息来完成坐标变换或推进器力分配。

它是最直接的“给机器人施加广义力”接口，不经过 PID/ADRC/MPC。

### 13.6 `legSetPosition(_x, _y, _z)`

调用 `gait_ctrl.legsetposition()` 设置腿端位置，并打印设置值。它属于执行机构接口，不负责根据误差自动控制。

### 13.7 `legSetAngle(_A, _B, _C)`

调用 `gait_ctrl.legsetangle()` 设置腿部三个角度。角度单位需要结合 `C_GaitControl` 实现确认。

### 13.8 `gaitControl()`

这个函数根据全局 `mode` 和 `dir` 选择步态：

```text
mode == 1
  dir == FORWARD  -> forward4(gait_R)
  dir == LEFT     -> round_left(gait_R)
  dir == RIGHT    -> round_right(gait_R)
  dir == BACK     -> back(gait_R)
否则
  gaitstatu = Stop
```

方向常量在头文件中定义：

```cpp
FORWARD = 0;
LEFT    = 1;
RIGHT   = 2;
BACK    = 4;
```

这里用的是多个独立的 `if`，而不是 `else if`。当前不同方向常量互不相等，所以结果正常；为了表达“方向只能是一个”，以后可以改为 `else if` 或 `switch`。

### 13.9 `gait_stop()`

直接调用 `gait_ctrl.gait_stop()`，停止步态。它比手动修改 `mode` 更明确，适合急停或模式切换时调用。

---

## 14. ADRC 三维位置控制（约第 729～794 行）

### `simCombineADRC_3DCtrl(...)`

这是组合 ADRC 三维控制的过程接口。输入分为两组：

```text
期望：exp_p_x, exp_v_x, exp_p_y, exp_v_y, exp_p_z, exp_yaw
当前：cur_p_x, cur_v_x, cur_p_y, cur_v_y, cur_p_z, cur_yaw
```

### 14.1 单积分/双积分模式

当模式是 `ALL_CONTROL_MODE_SINGLE_INTEG` 或 `ALL_CONTROL_MODE_DOUBLE_INTEG` 时，先计算 XY 平面距离：

```cpp
sqrt((exp_x-cur_x)^2 + (exp_y-cur_y)^2)
```

如果距离小于 `finish_radius`：

1. 四个推进器停止；
2. 输出“in area now”；
3. `is_judge_direction = false`，不再判断方向。

否则：

1. 调用 `judge_Direction()` 判断朝向；
2. 处理接近 ±180° 的航向角边界；
3. 调用 `adrcSelectControllerAndOutput()` 选择并执行 ADRC。

### 14.2 调参模式

当模式是 `ADJUST_PARAMETERS_MODE` 时，不进行到达区域判断，直接把期望和当前状态交给 ADRC 组合控制器。这适合人工调参或观察控制器响应。

### 14.3 其他模式

如果当前模式不是上面三种，函数只打印提示，不输出控制。换句话说，调用该函数前必须先设置正确的 `debug_mode`。

### 14.4 航向角边界注意事项

代码试图处理当前 yaw 和期望 yaw 一个接近 `+180°`、另一个接近 `-180°` 的情况，避免把很小的角度误差当成接近 360° 的大误差。但源码中出现了：

```cpp
_exp_yaw = -exp_yaw;
```

这里使用的是全局变量 `exp_yaw`，不是函数参数 `_exp_yaw`。这可能是历史命名遗留或潜在错误。阅读和调试时应特别关注这一点。

---

## 15. S 面三维位置控制（约第 795～835 行）

### `simCombineSS_3DCtrl(...)`

参数结构与 `simCombineADRC_3DCtrl()` 类似，但内部调用：

```cpp
comb3Dctrl.ssSelectControllerAndOutput(...);
```

控制流程为：

1. 仅在单积分/双积分模式下执行；
2. 计算 XY 距离；
3. 进入 `FINISH_RADIUS` 范围后直接调用 `vec_ctrl.motorforce(0,0,0,0,0,0,0)` 停止；
4. 未到达时判断方向并调用 S 面控制器；
5. 其他模式只打印提示。

和 ADRC 版本相比，停止接口使用的是 `motorforce()`，而 ADRC 版本使用了 `thrusterforce2speed(0,0,0,0)`。这说明两套历史控制路径的输出层并不完全一致。

---

## 16. PID 三维控制（约第 908～997 行）

前面第 9 节已经介绍了参数设置函数，这里补充三维控制调用关系。

### `simCombinePID_3DCtrl(...)`

把完整的期望/当前状态转交给 `combine3DPidCtrl.updateControl()`。当前固定用 `SHAPE_X`。

### `PidXYZControl_H(...)`

H 型三维位置控制，目标 yaw 由组合 PID 控制器的正常导航逻辑处理。

### `PidXYZControl_X(...)`

X 型三维位置控制。8 参数重载多传入当前 Z 速度，适合有深度速度反馈的版本。

### `PidDepVelControl_X(...)`

X 型深度速度控制。

### `PidDepthYawControl_H(...)`

H 型深度和航向联合控制。

### `PidYawControl_H(...)`

H 型单独航向控制。

### `PidDepthControl_X(...)`

X 型单独深度位置控制。

这些函数的共同特点是：它们把“不需要控制”的维度填成 0，然后统一调用组合 PID 控制器。真正的推力计算和推进器输出由 `C_Combine3DPidControl` 完成。

---

## 17. ADRC 速度控制（约第 998～1065 行）

### `simCombineADRC_3DSpeedCtrl(...)`

输入：

```text
期望速度：_exp_v_x, _exp_v_y, _exp_v_yaw, _exp_v_z
当前速度：_cur_v_x, _cur_v_y, _cur_v_yaw, _cur_v_z
```

函数固定选择 `SHAPE_H`，然后调用：

```cpp
comb3DSpeedCtrl.adrcSelectSpeedControllerAndOutput(...)
```

计算完成后，把组合速度控制器内部结果复制到全局缓存：

```cpp
four_thrust1 = ...thrust1;
...
control_cmd_x = ...controller_x_speed;
control_cmd_yaw = ...controller_yaw_speed;
```

因此，这个函数除了执行控制，还承担“把内部调试结果导出给其他代码”的作用。

### `simCombineADRC_3DSpeedCtrl_v2(...)`

v2 版本多接收一个期望航向 `_exp_yaw` 和当前航向 `_cur_yaw`：

```text
期望：vx、vy、vyaw、vz、yaw
当前：vx、vy、vyaw、vz、yaw
```

它同样默认使用 H 型，并保存四个推进器推力和 X/Yaw 控制量。v2 更适合需要“速度控制 + 航向角约束”的场景。

### `clearADRCSpeedControllerVariables()`

依次清空 X/Y/Z/Yaw 速度 ADRC 控制器以及航向控制器的内部状态。通常在：

- 切换控制阶段；
- 重新开始一次对接；
- 防止积分/观测器历史状态污染下一次实验；

时调用。否则旧的积分项或 ESO 状态可能影响新任务。

---

## 18. 面向具体任务的 ADRC 调试函数（约第 1066～1139 行）

### `ADRCYawControl_H(_exp_yaw, _cur_yaw)`

H 型只调节航向角。它调用 v2 位置组合控制器，并把 X/Y/Z 相关量置零：

```cpp
adrcSelectPositionControllerAndOutput(
    SHAPE_H, 0, 0, exp_yaw, 0,
    0, 0, cur_yaw, 0);
```

适合只做原地转向或对接第二阶段航向修正。

### `ADRCDepthControl_X(_exp_z, _cur_z)`

X 型只调节深度，其他位置和航向量都置零。当前使用 `comb3DPositionCtrl_v2`。

### `ADRCPosition3DControl_H_v2(...)`

H 型位置控制入口，输入目标位置、目标 yaw 和当前状态。但当前函数虽然接收 `_exp_yaw`，实际计算的是：

```cpp
float __exp_yaw = atan2(_exp_y - _cur_y,
                        _exp_x - _cur_x) * 57.3;
```

也就是说，实际传给控制器的是“从当前位置指向目标位置的方位角”，而不是调用者传入的 `_exp_yaw`。函数注释也明确写了这是用于接近目标的阶段。

注意：`57.3` 是对 `180 / PI` 的近似，建议以后统一使用 `180.0 / PI`，以提高可读性和精度。

### `ADRCPosition3DControl_X(...)`

X 型三维位置控制。它把期望 yaw 固定为 0，目标是控制 X/Y/Z 位置而不是航向角。

---

## 19. 调试版 ADRC 参数函数（约第 1148～1240 行）

这组函数针对 `comb3DPositionCtrl` 或 `comb3DPositionCtrl_v2` 的单个轴进行调参，分为 4 参数、5 参数和 6 参数重载。

### 19.1 Z 方向

- `setDebugPzADRCPmt_v2(r0, wc, w0, b0)`：4 参数版本；
- `setDebugPzADRCPmt_v2(r0, kp, kd, w0, b0)`：5 参数版本；
- `setDebugPzADRCPmt(kp, kd, h, r, w, b0)`：旧版 6 参数版本。

### 19.2 Yaw 方向

- `setDebugYawADRCPmt_v2(...)` 两个重载；
- `setDebugYawADRCPmt(...)` 旧版 6 参数接口。

### 19.3 X/Y 方向

- `setDebugPxADRCPmt_v2(...)`、`setDebugPxADRCPmt(...)`；
- `setDebugPyADRCPmt_v2(...)`、`setDebugPyADRCPmt(...)`。

`v2` 版本调用 `comb3DPositionCtrl_v2`，旧版函数调用 `comb3DPositionCtrl`。这些函数的主要工作都是“转发参数 + 打印参数”，算法本身不在此实现。

### 学习重点

参数数量不同代表控制器类中存在不同版本的参数接口。不要仅因为函数名相似就混用；尤其注意：

- `setadrcPosPmt` 与 `SetadrcPosPmt` 大小写不同；
- v2 和旧版对应不同控制器对象；
- 5 参数版本中 `_k_p`、`_k_d` 是显式传入的，4 参数版本则由控制器内部根据带宽计算或采用默认关系。

---

## 20. `ADRCPosition3DControl_G(...)`：G 型抓取/特殊任务控制

这个函数与 H/X 型位置控制类似，但用于 G 型特殊运动模式。它先计算目标方位角：

```cpp
float __exp_yaw = atan2(_exp_y - _cur_y,
                        _exp_x - _cur_x) * 57.3 - 40;
```

相比普通 H 型，额外减去 40 度，说明 G 型机械结构或抓取姿态需要一个固定的角度偏置。若结果小于 -180 度，再加 360 度进行归一化。

之后调用：

```cpp
comb3DPositionCtrl_v2.adrcSelectPositionControllerAndOutput(
    SHAPE_G, ...);
```

这个 `-40` 是任务相关经验参数，不应当当作通用公式；修改前应确认机械安装角、坐标系和目标方向。

---

## 21. 被注释掉的旧 MPC/姿态控制代码（约第 1250～1379 行）

这一大段代码包含旧版：

- 航向角 MPC；
- 旧版轨迹跟踪；
- 固定速度轨迹跟踪；
- 手工推力分配公式。

因为整体处于注释块中，所以当前不会参与编译。它的学习价值主要在于展示项目曾经尝试过的控制链：

```text
航向控制量
   ▼
根据四个推进器布局计算 f1/f2/f3/f4
   ▼
合成为 force_x 和 torque_z
   ▼
vec_ctrl.motorforce(...)
```

如果只想学习当前正在工作的代码，可以先略读；如果要研究项目控制算法演化或旧实验参数，再回头看这一节。

---

## 22. `yawidentification()`：航向系统辨识

这个函数给推进器施加固定的差异输入，用来激励机器人产生航向运动：

```cpp
f1 = 1.2;
f2 = 0.4;
f3 = f1 + 0.1;
f4 = f2;
```

然后根据四个推进器的布局计算：

```cpp
force_x = f1 + f4 - f2 - f3;
torque_z = (f2 + f4 - f1 - f3) * 0.02758;
```

最后调用 `vec_ctrl.motorforce()` 输出。`num` 是静态变量，每次调用自动加 1，因此函数可以记录激励的离散时间；但当前真正使用的是常量输入，`time` 没有参与有效计算。

用途：采集输入推力与航向响应，用于辨识水动力参数或调节航向控制器。

---

## 23. `surgeidentification()`：前进方向系统辨识

这个函数使用正弦变化的 `f1/f4` 和固定的 `f2/f3`：

```cpp
f1 = sin(0.1 * time) + 1;
f4 = f1;
f2 = f3 = 0.4;
```

这样会产生随时间变化的前进方向激励。然后同样通过四推进器组合关系计算 `force_x`、`force_z` 和 `torque_z`，再调用 `vec_ctrl.motorforce()`。

`num1` 是静态计数器。`fz1~fz4` 当前都为 0，说明这个辨识函数只激励水平运动，不激励 Z 方向。

---

## 24. `update_iter()`：更新 MPC 参考点编号

```cpp
if (iter_num < num_trackingpoints) {
    iter_num = iter_num + 1;
}
```

作用很简单：让 MPC 跟踪索引递增，但不超过参考轨迹点数。

当前 `f_mpcTracking()` 中这行调用被注释掉了，因为它会根据当前位置重新查找参考点。所以 `update_iter()` 是保留的旧/备用接口，目前不是 MPC 主流程的关键步骤。

---

## 25. `f_mpcTracking(...)`：MPC 轨迹跟踪主函数

这是本文件中最复杂、最值得重点学习的函数。它把当前机器人状态、参考轨迹和 MPC 优化结果串成一个完整控制闭环。

函数输入：

```cpp
Position c_position;
Attitude c_attitude;
Speed c_speed;
```

分别是当前的位置、姿态和速度。

### 25.1 第一步：准备参考轨迹

```cpp
if (ref_trajectory.rows() <= 0) {
    ref_trajectory = robot1_reference_trajectory;
    vrRobot_ref_trajectory = virtureRobot_reference_trajectory;
}
```

如果内部 `ref_trajectory` 还没有数据，就从全局订阅缓存中拷贝机器人 1 的参考轨迹，同时保存虚拟机器人轨迹。

然后：

```cpp
num_trackingpoints = ref_trajectory.rows();
```

记录轨迹点数。

### 25.2 第二步：计算编队中心位置

```cpp
vs_reference_PositionX =
    (Robot1_c_position.x + Robot2_c_position.x + Robot3_c_position.x) / 3;
```

这里计算三个机器人的 X 坐标平均值，作为编队中心 X 位置。代码打印了 `vs_reference_PositionY`，但在当前片段中没有看到对应的平均值计算，说明 Y 方向逻辑可能尚未完成或在其他回调中更新。

### 25.3 第三步：确认 MPC 模式和轨迹有效

只有在：

```cpp
comb3Dctrl.debug_mode == MPC_MODE
```

并且 `ref_trajectory.rows() > 0` 时，才真正执行后续 MPC 控制。

否则函数不会求解控制量。

### 25.4 第四步：构造当前状态向量

```cpp
VectorXd x_real_current = VectorXd::Zero(X_num);
x_real_current(0) = c_position.x;
x_real_current(1) = c_position.y;
x_real_current(2) = c_attitude.yaw * PI / 180;
x_real_current(3) = sqrt(c_speed.x*c_speed.x + c_speed.y*c_speed.y);
x_real_current(4) = c_speed.yaw;
```

当前状态向量为：

```text
x_real_current = [x, y, yaw(rad), 水平速度大小, yaw速度]^T
```

特别注意：

- 输入姿态 yaw 似乎是角度制，先转换成弧度；
- 速度 `x/y` 被合成为速度大小，丢失了水平速度方向信息；
- `c_speed.yaw` 没有再次转换，单位必须和 MPC 模型保持一致。

### 25.5 第五步：选择当前参考点

代码通过比较当前 X 位置和参考轨迹第一列的排序位置，得到 `iter_num`。

#### 单机模式

```cpp
SINGLE_ROBOT
```

把 `ref_trajectory` 每一行的 X 坐标放到 `ref_point`，再把当前机器人 X 坐标放进去排序，然后查找当前 X 所在位置，得到参考点索引。

#### 多机模式

```cpp
MULTI_ROBOT
```

使用虚拟机器人参考轨迹和编队中心 X 位置进行同样的操作。

### 注意：当前写法存在边界和精度风险

`std::find()` 查找的是浮点数的完全相等值，而排序后的 `vector` 中刚刚插入了同一个 `x_real_current(0)`，一般能找到；但如果后续改成其他计算方式，就不应依赖浮点精确相等。更稳妥的做法是直接记录插入位置，或查找最接近的轨迹点。

### 25.6 第六步：更新 MPC 内部状态

```cpp
MPC_tracking.Update(iter_num,
                    ref_trajectory,
                    MPC_real_force,
                    x_real_current);
```

这里把以下信息交给 MPC：

- 当前参考点编号；
- 完整参考轨迹；
- 上一次控制输入 `MPC_real_force`；
- 当前状态 `x_real_current`。

`Update()` 通常会完成模型线性化、预测矩阵更新或内部参考状态更新，具体要阅读 `MPCController` 类。

### 25.7 第七步：计算状态误差 `x_piao`

```cpp
VectorXd x_refer = ref_trajectory.block(iter_num - 1, 0, 1, X_num).transpose();
VectorXd x_piao = x_real_current - x_refer;
```

`x_piao` 表示当前状态相对参考状态的误差：

```text
x_piao = 当前状态 - 参考状态
```

Yaw 误差需要做角度环绕处理，否则 `179°` 和 `-179°` 会被误判为相差 358°。代码尝试把 yaw 误差调整到 `[-PI, PI]` 附近。

不过源码中第二个分支写成了：

```cpp
if (x_piao(2) > PI) {
    x_piao(2) = x_piao(2) + 2 * PI;
}
```

从常见角度归一化逻辑看，这里通常应当是减去 `2 * PI`，当前写法可能是潜在错误，建议结合实验验证。

### 25.8 第八步：拼接增广状态 `kesi`

```cpp
VectorXd kesi(X_num + U_num);
kesi.head(X_num) = x_piao;
kesi.segment(X_num, U_num) = MPC_real_force;
```

`kesi` 由两部分组成：

```text
kesi = [状态误差; 上一次控制输入]
```

增量式 MPC 需要把上一时刻输入一起放入增广状态，优化结果通常是 `Δu`，再与旧输入相加得到新的 `u`。

### 25.9 第九步：根据误差动态调整代价权重

源码根据 `x_piao(1)`、`x_piao(2)` 等条件修改 `q22`、`q33`、`q44`。其思想是：

- 偏离轨迹较远时，加大横向误差权重；
- 接近轨迹时，调整航向或速度权重；
- 不同阶段关注不同误差。

前面有大量被注释的旧规则，当前启用的规则较少。初学时先把它理解为“**根据当前误差自适应切换 MPC 代价函数权重**”。

### 25.10 第十步：构造 Q 矩阵并求解

```cpp
MatrixXd Q = MatrixXd::Zero(Y_num, Y_num);
Q(0,0) = q11;
Q(1,1) = q22;
Q(2,2) = q33;
Q(3,3) = q44;
Q(4,4) = q55;
```

`Q` 是状态/输出误差权重矩阵，当前是 5×5 对角矩阵。对角线越大，越重视对应误差。

之后根据 `Linear_mode` 和 `delta_mode` 调用不同的求解函数：

- `Linear_mode == 1`：调用 `Update_Control_cmd(...)`；
- `Linear_mode == 2`：调用 `Update2_Control_cmd(...)`；
- `delta_mode == false`：直接得到绝对控制量；
- `delta_mode == true`：得到增量控制量后加上 `MPC_real_force`。

### 25.11 第十一步：保存和打印控制结果

```cpp
MPC_real_force = real_force;
save_debugdata.write_data(iter_num, x_piao, real_force);
save_debugdata1.write_data(iter_num, x_real_current);
```

这里把本次控制结果保存为下一周期的历史输入，同时记录调试数据。

### 25.12 第十二步：把二维控制量分配给四个推进器

MPC 输出的是：

```text
real_force(0)：前进/纵向控制力
real_force(1)：航向控制力矩
```

首先根据纵向力的正负分配到不同推进器：

```cpp
if (real_force(0) > 0) {
    force_2 = real_force(0) / 2;
    force_3 = real_force(0) / 2;
} else {
    force_1 = -real_force(0) / 2;
    force_4 = -real_force(0) / 2;
}
```

然后根据 yaw 力矩的正负叠加到不同推进器：

```cpp
if (real_force(1) > 0) {
    force_1 += real_force(1) / 2;
    force_3 += real_force(1) / 2;
} else {
    force_2 -= real_force(1) / 2;
    force_4 -= real_force(1) / 2;
}
```

最后对各推进器取平方根：

```cpp
force_i = sqrt(abs(force_i));
```

这表示代码假设推进器“推力”和“转速”之间近似满足平方关系：

```text
推力 ∝ 转速²
```

所以用平方根把力换算为速度量级。`abs()` 会丢掉方向信息，因此这里的四个 `force_i` 实际更像非负转速幅值，而不是带符号的力。

### 25.13 第十三步：输出推进器转速并结束任务

```cpp
MPC_vec_ctrl.thrusterforce2speed(
    force_1 / 1.5,
    force_2 / 1.5,
    force_3 / 1.5,
    force_4 / 1.5);
```

再次除以 1.5 是经验缩放，用于限制输出或匹配仿真推进器模型。

当：

```cpp
iter_num == num_trackingpoints - 3
```

时：

1. 设置控制模式为 0；
2. 停止推进器；
3. 把腿角度归零。

这里提前 3 个点停止，可能是为了给机器人留出减速距离，但这是经验逻辑，最好改成显式的“到达阈值 + 速度阈值”判断。

---

## 26. 当前文件的主要调用关系

### 26.1 位置控制

```text
上层目标/传感器反馈
          │
          ▼
simCombineADRC_3DCtrl()
          │
          ▼
comb3Dctrl.adrcSelectControllerAndOutput()
          │
          ▼
内部 ADRC + vec_ctrl
          │
          ▼
推进器输出
```

### 26.2 PID 控制

```text
set*PIDPmt() 设置参数
          │
          ▼
PidXYZControl_*() / simCombinePID_3DCtrl()
          │
          ▼
combine3DPidCtrl.updateControl()
          │
          ▼
推进器/腿部输出
```

### 26.3 速度 ADRC 控制

```text
simCombineADRC_3DSpeedCtrl*()
          │
          ▼
comb3DSpeedCtrl.adrcSelectSpeedControllerAndOutput()
          │
          ▼
读取 speed_vec_ctrl.thrust1~4
          │
          ▼
four_thrust1~4 / control_cmd_x / control_cmd_yaw
```

### 26.4 MPC 控制

```text
参考轨迹 + 当前状态
          │
          ▼
f_mpcTracking()
          │
          ▼
状态误差、约束、Q矩阵
          │
          ▼
MPC_tracking.Update_Control_cmd()
          │
          ▼
real_force[0], real_force[1]
          │
          ▼
四推进器力分配
          │
          ▼
thrusterforce2speed()
```

---

## 27. 初学者推荐阅读顺序

如果第一次看这个文件，建议不要从 1774 行一口气读完，而是按下面顺序：

1. 先看头文件中的控制模式和 `SHAPE_*` 常量；
2. 看全局控制器对象，弄清楚每个对象负责什么；
3. 看 `LegShape_*()`、`simMotorForce()` 和 `legSet*()`，理解最底层输出接口；
4. 看 PID 便捷接口，理解“期望值/当前值”如何传入组合控制器；
5. 看 `simCombineADRC_3DCtrl()`，理解到达区域判断和模式分支；
6. 看 `simCombineADRC_3DSpeedCtrl_v2()`，理解如何读取四个推进器推力；
7. 最后看 `f_mpcTracking()`，重点理解状态向量、误差向量、约束和力分配。

---

## 28. 代码中值得特别注意的问题

以下不是说代码一定不能运行，而是阅读和维护时应该重点验证的地方：

1. **绝对文件路径**：数据保存路径写死在不同开发者的目录中，换环境后容易失败。
2. **全局变量很多**：控制器状态和任务状态分散在全局变量中，模式切换时要确认是否清理历史状态。
3. **yaw 边界处理可疑**：`simCombineADRC_3DCtrl()` 中使用了全局 `exp_yaw`，而不是参数 `_exp_yaw`。
4. **MPC yaw 归一化可疑**：误差大于 `PI` 时代码加 `2*PI`，从通常逻辑看可能应减去 `2*PI`。
5. **MPC 浮点索引**：通过排序和 `find()` 查找参考点，逻辑能工作但不够稳健。
6. **`vs_reference_PositionY`**：函数中打印了它，但当前片段没有看到和 X 一样的平均值更新。
7. **推进器方向信息被 `abs()` 丢弃**：平方根换算之前取绝对值，实际符号由前面的分配逻辑决定，修改时不要随意改变顺序。
8. **经验常数较多**：例如 `0.02758`、`1.5`、`-40`、`57.3`，修改前必须确认其物理意义和单位。
9. **停止条件偏经验**：MPC 在最后 3 个参考点时停止，可能与速度、位置误差并不完全对应。
10. **注释和代码不完全一致**：例如控制频率注释写 5 Hz，但变量当前值为 30；学习时应以实际代码为准。
11. **函数声明一致性**：`LegShape_G()` 在 `.cpp` 中有实现，使用前确认头文件是否声明。
12. **局部变量遮蔽全局变量**：`f_mpcTracking()` 中的 `force_1~force_4` 是局部变量，调试时要注意。

---

## 29. 一句话总结每个主要函数

| 函数 | 一句话功能 |
|---|---|
| `initMpcOsqp` | 准备动力学模型、约束并初始化 MPC |
| `save_identi_data` | 保存系统辨识数据，存在两个重载 |
| `save_this_robot_docking_data` | 保存完整对接过程数据 |
| `set*PIDPmt` | 设置各通道 PID 参数并启用控制器 |
| `set*ADRCPmt` | 设置位置/速度 ADRC 参数 |
| `set*SSPmt` | 设置 S 面控制器参数 |
| `setComb3DCtrlMode` | 修改组合三维控制器模式 |
| `LegShape_H/H_T/X/G` | 切换机器人腿型或推进形态 |
| `simMotorForce` | 直接向底层推进器控制器施加广义力 |
| `legSetPosition` | 设置腿端位置 |
| `legSetAngle` | 设置腿部角度 |
| `gaitControl` | 根据模式和方向执行前后左右步态 |
| `gait_stop` | 停止步态 |
| `simCombineADRC_3DCtrl` | 执行组合 ADRC 三维位置控制 |
| `simCombineSS_3DCtrl` | 执行组合 S 面三维位置控制 |
| `simCombinePID_3DCtrl` | 执行组合 PID 三维控制 |
| `PidXYZControl_*` | 面向 H/X 型的 PID 便捷位置控制接口 |
| `PidDepthYawControl_H` | H 型同时控制深度和航向 |
| `PidYawControl_H` | H 型只控制航向 |
| `PidDepthControl_X` | X 型只控制深度 |
| `simCombineADRC_3DSpeedCtrl*` | 执行组合 ADRC 速度控制并导出推力 |
| `clearADRCSpeedControllerVariables` | 清空速度 ADRC 的历史状态 |
| `ADRCYawControl_H` | H 型单独航向 ADRC 控制 |
| `ADRCDepthControl_X` | X 型单独深度 ADRC 控制 |
| `ADRCPosition3DControl_H_v2` | H 型三维位置控制，自动计算目标方位角 |
| `ADRCPosition3DControl_X` | X 型三维位置控制 |
| `ADRCPosition3DControl_G` | G 型特殊/抓取任务位置控制 |
| `yawidentification` | 用固定推力激励航向系统辨识 |
| `surgeidentification` | 用正弦推力激励前进系统辨识 |
| `update_iter` | 递增 MPC 轨迹索引 |
| `f_mpcTracking` | 完成参考点选择、MPC 求解、推力分配和停止 |

---

## 30. 总结

`f_single_robot_control.cpp` 的核心价值在于把多个控制算法和执行机构接口组织成统一的单机控制 API。理解它时最重要的不是记住所有函数名，而是掌握以下链路：

```text
控制模式
  → 选择腿型/控制器
  → 传入期望状态和当前状态
  → 计算控制力或控制力矩
  → 分配到四个推进器
  → 转换为推进器速度
  → 机器人运动并产生新的反馈
```

其中：

- PID 适合结构清晰、调参直观的闭环控制；
- ADRC 通过观测器和扰动补偿增强鲁棒性；
- S 面是另一种非线性控制路径；
- MPC 使用模型和未来预测轨迹进行约束优化；
- `vec_ctrl`、`gait_ctrl` 和各个组合控制器负责把算法输出落实为机器人执行机构动作。

建议结合 `f_single_robot_control.h`、`c_sim_3d_ctrl.h`、`c_vectoredthrustcontrol.h`、`c_gaitcontrol.h` 和 `MPCController.h` 一起阅读，因为本文件中的很多函数只是“适配层/转发层”，真正的控制公式在这些类中。
