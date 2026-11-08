# ArUco 代码阅读说明

本目录负责检测视觉标记、估计标记相对相机的位姿，并转换到机器人机体坐标系。源码中的“重要／次要”表示阅读优先级，不表示可以直接删除代码。

## 当前运行链路

```text
底部相机图像
  -> aruco_det.cpp
  -> /aruco/pose（geometry_msgs::PoseStamped）
  -> aruco_posture.cpp
  -> camera_aruco_data（asrobot::camera_data）
  -> decision.cpp 的 downCamCallback
```

当前 CMakeLists.txt 编译 aruco_det 和 aruco_posture 两个节点，launch/aruco_det.launch 启动它们。其他两个 cpp 文件没有在当前 CMakeLists.txt 中配置为可执行程序。

## 每个函数的功能与阅读优先级

| 文件 | 函数 | 功能 | 阅读优先级 |
| --- | --- | --- | --- |
| aruco_det.cpp | cam_imageCallback | 把 ROS 图像转为 BGR8，缓存最新图像 | 重要：主流程输入 |
| aruco_det.cpp | main | 读参数和标定、检测大小码、切换目标、计算并发布位姿 | 最重要：检测主流程 |
| aruco_posture.cpp | modelStatesCallback_ | 读取 gazebo_robot1 的世界位姿，构建 Twb | 可后读：用于计算当前未发布的世界位姿 Twa |
| aruco_posture.cpp | aruco_pos_cb | 接收检测位姿，构建标记到相机的变换 Tca | 重要：转换输入 |
| aruco_posture.cpp | main | 设置相机安装变换，计算 Tba，发布机体系位置和姿态 | 最重要：坐标转换主流程 |
| aruco_det_singlemaker.cpp | cam_imageCallback | 缓存 USB 相机等来源的图像 | 参考：当前未编译 |
| aruco_det_singlemaker.cpp | main | 估计同尺寸标记的位姿，只发布指定 ID | 参考：学习单码检测时阅读 |
| aruco_demo.cpp | state_cb | 缓存飞控连接、模式和解锁状态 | 参考：只用于无人机示例 |
| aruco_demo.cpp | pos_cb | 读取无人机本地位姿，构建 Twb | 参考：只用于无人机示例 |
| aruco_demo.cpp | aruco_pos_cb | 读取标记位姿，构建 Tca | 参考：只用于无人机示例 |
| aruco_demo.cpp | main | 等待飞控，发送位置目标，执行搜索、对准、下降和降落状态机 | 参考：当前机器人检测链路可跳过 |

## 哪些可以后读

- 日志和 cout：用于排查状态，不参与位姿求解。
- drawDetectedMarkers、drawAxis、imshow 和图像发布：用于观察结果；图像话题可能有外部订阅者。若将来关闭窗口显示，需要同时考虑 waitKey 的窗口事件处理作用。
- 被注释掉的旧内参、旧坐标计算和试验代码：当前不执行。
- aruco_posture.cpp 中 Twb 和 Twa 的世界位姿计算：当前输出使用 Tba；以后改为世界坐标输出时，这部分就重要了。
- aruco.txt：手工记录的旋转矩阵及坐标结果，没有程序读取它。
- output/：当前为空，未发现上述源码向它写入结果。

## 核心约定与现有注意点

- Txy 表示从 y 坐标系到 x 坐标系。w 为世界，b 为机体，c 为相机，a 为标记。
- Tba = Tbc * Tca 是当前输出的依据；Twa = Twb * Tbc * Tca 是标记的世界位姿。
- 位移以米为单位，camera_aruco_data 中的姿态角以度为单位，发布的 position_z 额外加 0.06 米。
- 双码默认 ID/边长为 5/0.1 米、18/0.02 米；实际尺寸影响距离尺度，不能只改 ID 而忽略边长。
- 相机标定文件默认路径指向旧的 /home/jll 工作目录，需要通过参数匹配实际环境。
- 双码切换默认阈值 0.5 米和 0.4 米存在重叠，可能来回切换；判断依据是相机 Z 分量。
- 检测循环可能重复处理缓存图像，因此丢失计数不一定等于不同图像的帧数。
- aruco_posture 当前未显式初始化变换矩阵为单位矩阵，也没有首帧等待、有效标志或接收超时判断；没有新检测时会持续发布缓存结果。
- 无人机示例使用带 aruco 后缀的消息包类型，不能直接假定与主检测节点的消息接口兼容。

这些注释记录现有实现，没有修改算法或运行行为。
