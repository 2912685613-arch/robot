# asrobot launch files

## asrobot.launch

通用的 ASRobot 运行时启动文件，可用于不同机器人实例。

主要参数：

- `namespace`：Gazebo/ros_control 命名空间，默认 `gazebo_robot1`
- `robot`：ASRobot 应用话题命名空间，默认 `robot1`
- `model_name`：Gazebo 中的模型名称，默认等于 `namespace`
- `enable_decision`：是否启动决策节点
- `enable_joint_control`：是否启动关节命令转发节点
- `enable_thruster_control`：是否启动推进器命令转发节点
- `enable_motion`：是否启动运动控制节点
- `enable_mancontrol`：是否启动手动控制节点
- `enable_aruco`：是否启动 ArUco 检测和位姿节点

第一台机器人：

```bash
roslaunch asrobot asrobot.launch
```

第二台机器人（通常只给一台机器人启用当前全局 ArUco 接口）：

```bash
roslaunch asrobot asrobot.launch \
  namespace:=gazebo_robot2 \
  robot:=robot2 \
  model_name:=gazebo_robot2 \
  enable_aruco:=false
```

## aruco_det.launch

仅启动 `aruco_det` 和 `aruco_posture`。

## multi_comm.launch

启动 `master_discovery_fkie` 和 `master_sync_fkie`，用于多机 ROS Master 通信。
