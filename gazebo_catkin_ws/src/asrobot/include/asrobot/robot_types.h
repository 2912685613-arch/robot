#ifndef ASROBOT_ROBOT_TYPES_H // 检查这个宏是否还没被定义
#define ASROBOT_ROBOT_TYPES_H // 如果没有，就定义它

// =================================================================
// ===           项目通用的数据结构定义                      ===
// =================================================================

// 定义姿态结构体
struct Pose
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
};

// 定义速度结构体
struct Velocity
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    // 如果需要，也可以添加角速度
    // float yaw_rate = 0.0f;
};

// 定义完整的机器人状态结构体
struct RobotState
{
    Pose pose;
    Velocity velocity;
};

#endif // ASROBOT_ROBOT_TYPES_H  // 结束头文件保护