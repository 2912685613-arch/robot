# gazebo_pose_publisher/msg/TrackedPose.msg

std_msgs/Header header    # 包含时间戳和frame_id
string model_name         # 被追踪模型的名字
geometry_msgs/Point position
float64 yaw_radians       # 偏航角 (弧度)
bool found                # 是否成功追踪到模型