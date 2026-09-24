## 用软件渲染可行，

gjx@gjx:~/robot/gazebo_catkin_ws$ export LIBGL_ALWAYS_SOFTWARE=1
gjx@gjx:~/robot/gazebo_catkin_ws$ roslaunch gazebo_model view_robot.launch ，

（上面这条只在「单独看模型」时用，仿真已经在跑的时候不要这么用，原因见下。）

## 仿真运行中用 RViz 检查 TF（不要用 view_robot.launch）

### 为什么不能在 five_robots.sh 之后再 roslaunch view_robot.launch
1. 它用 `<param name="robot_description">`（无命名空间）= 全局 `/robot_description`，
   会覆盖 gazebo_eight2.launch 正在使用的同名参数（robot_state_publisher82 读的就是它）；
2. 它起的 robot_state_publisher 与仿真里的 robot_state_publisher82 会重复发布同一批 TF 帧；
3. 它起 joint_state_publisher_gui，会往 `/joint_states` 里灌“假的”关节角；
4. 它默认 namespace:=robot1，帧名是 `robot1/xxx`，而仿真里是 `gazebo_eight_camera2/xxx`，
   Fixed Frame 也对不上，看不到仿真中的机器人。

### 正确做法：watch_robot_tf.launch（只起 RViz，零干扰）
```
roslaunch gazebo_model watch_robot_tf.launch                       # 默认 gazebo_eight_camera2 + water
roslaunch gazebo_model watch_robot_tf.launch namespace:=gazebo_robot1 mode:=land
```
- 不启动 robot_state_publisher / joint_state_publisher_gui，不发布任何 TF 与关节状态；
- URDF 载入到独立参数 `/watch_robot_description`，绝不碰 `/robot_description`；
- 配套配置 `gazebo_model/rviz/watch_robot.rviz`，Fixed Frame = `world`，RobotModel + TF 全开。

### 更快、更权威的 TF 检查
```
rosrun tf view_frames && evince frames.pdf     # 生成整棵 TF 树
rosrun rqt_tf_tree rqt_tf_tree                 # 图形化看树
rostopic list | grep -E "joint_states|pose_gt"
rostopic hz /gazebo_eight_camera2/joint_states
rosrun tf tf_echo world gazebo_eight_camera2/base_link
```

### 两个需要重点确认的疑点
- 关节状态发布在 `/gazebo_eight_camera2/joint_states`（controller_manager 放在该命名空间下），
  而 robot_state_publisher82 默认订阅全局 `/joint_states`，两者不是同一话题，
  且全工程没有找到 remap；所以 12 个腿关节 + 8 个相机关节（全部是 revolute）
  很可能拿不到关节角，TF 里只剩 `Battery`、`Top Shell` 两个 fixed 关节。
- `world -> gazebo_eight_camera2/base_link` 由 uuv_assistants/message_to_tf 提供，
  它订阅的是 `/gazebo_eight_camera2/pose_gt`；但八推机器人 xacro 里没有 pose_gt 传感器插件
  （libuuv_gazebo_ros_pose_gt_plugin.so），这条链路也可能是断的。
  若 RViz 里 Fixed Frame 选 `world` 看不到机器人，可先把 Fixed Frame 改成
  `gazebo_eight_camera2/base_link`，此时能看机器人内部的 TF 是否齐全。
