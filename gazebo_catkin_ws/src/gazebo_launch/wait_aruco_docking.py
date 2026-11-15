#!/usr/bin/env python3
"""动态 ArUco 启动检查；只读取 ROS 状态，不启动 Gazebo，也不发运动命令。"""
import argparse
import math
import socket
import sys
import time

import rosgraph
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from asrobot.msg import camera_data


def preflight():
    """拒绝复用已存在的 Gazebo/控制节点；允许复用只有 roscore 的 Master。"""
    socket.setdefaulttimeout(2.)
    try:
        publishers, subscribers, services = rosgraph.Master('/aruco_docking_preflight').getSystemState()
    except (OSError, rosgraph.MasterError):
        return
    conflicts = {name for name, _ in services if name.startswith('/gazebo/')}
    nodes = {node for _, group in publishers + subscribers + services for node in group}
    conflicts.update(nodes & {'/gazebo', '/aruco_det', '/aruco_posture', '/aruco_pose_logger',
                              '/decision_robot1', '/robot1/motion', '/joint_control_robot1',
                              '/thruster_control_robot1'})
    if conflicts:
        raise RuntimeError('已有仿真或同名控制节点，请先关闭旧 launch：' + ', '.join(sorted(conflicts)))


def finite_body(msg):
    """机体位置与姿态全部是有限数值时，才能作为启动成功的反馈。"""
    return msg.aruco_valid and math.isfinite(msg.detection_stamp) and msg.detection_stamp > 0 and all(
        math.isfinite(value) for value in (msg.position_x, msg.position_y, msg.position_z,
                                         msg.cam_roll, msg.cam_pitch, msg.cam_yaw))


def fresh_pose(msg, now):
    """要求正确光学参考系、正向距离、有效四元数和最近 0.5 秒的图像时间。"""
    q = msg.pose.orientation
    p = msg.pose.position
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    age = now - msg.header.stamp.to_sec()
    return (msg.header.frame_id == 'gazebo_robot1/camera_bottom_optical_frame'
            and all(math.isfinite(value) for value in values) and p.z > 0
            and q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w > 1e-12 and 0 <= age <= .5)


def wait_detection(timeout):
    """同时确认实际标定、模型、视觉位姿及机体输出；用墙钟限制暂停时的等待。"""
    latest = {}
    def receive(name):
        def callback(msg):
            latest[name] = (msg, time.monotonic())
        return callback
    subscriptions = [
        rospy.Subscriber('/gazebo/model_states', ModelStates, receive('models'), queue_size=1),
        rospy.Subscriber('/gazebo_robot1/camera_bottom/camera_info', CameraInfo, receive('info'), queue_size=1),
        rospy.Subscriber('/aruco/pose', PoseStamped, receive('pose'), queue_size=1),
        rospy.Subscriber('/camera_aruco_data', camera_data, receive('body'), queue_size=1),
    ]
    deadline = time.monotonic() + timeout
    try:
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            wall = time.monotonic()
            if all(name in latest and wall - latest[name][1] <= .75
                   for name in ('models', 'info', 'pose', 'body')):
                models, info, pose, body = (latest[name][0] for name in ('models', 'info', 'pose', 'body'))
                if (all(name in models.name for name in ('gazebo_robot1', 'aruco_demo_marker'))
                        and info.K[0] > 0 and info.K[4] > 0 and fresh_pose(pose, rospy.get_time())
                        and finite_body(body)):
                    print('定位就绪：标记相对机体 xyz=({:.3f}, {:.3f}, {:.3f}) m'.format(
                        body.position_x, body.position_y, body.position_z))
                    return
            time.sleep(.05)
        raise RuntimeError('未收到新鲜有效的模型、camera_info、/aruco/pose 和 /camera_aruco_data')
    finally:
        for subscription in subscriptions:
            subscription.unregister()


def wait_command(timeout):
    """确认决策和运动节点都订阅了命令话题，避免首条 dock 只送到其中一个。"""
    deadline = time.monotonic() + timeout
    master = rosgraph.Master('/aruco_docking_check')
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        _, subscribers, _ = master.getSystemState()
        receivers = dict(subscribers).get('/robot1/cmd_message', [])
        if {'/decision_robot1', '/robot1/motion'} <= set(receivers):
            print('命令接口就绪：decision_robot1 和 robot1/motion')
            return
        time.sleep(.1)
    raise RuntimeError('/robot1/cmd_message 的决策/运动订阅者未就绪')


def physics_pause(paused, timeout):
    """只改变世界运行/暂停状态；不会把动态模型改成 static，也不改变控制算法。"""
    name = '/gazebo/pause_physics' if paused else '/gazebo/unpause_physics'
    rospy.wait_for_service(name, timeout=timeout)
    rospy.ServiceProxy(name, Empty)()
    print('物理仿真已' + ('暂停；相机画面也将停在当前帧。' if paused else '恢复。'))


def prepare(timeout, x, y, z):
    """清除启动期间漂移，获取一帧定位后暂停，等待用户输入 dock。"""
    if not all(math.isfinite(v) for v in (x, y, z)):
        raise ValueError('初始位置必须为有限数值')
    rospy.wait_for_service('/gazebo/set_model_state', timeout=timeout)
    # 重置是明确的教学准备动作，不是控制器自动悬停，日志中保留该动作。
    state = ModelState()
    state.model_name = 'gazebo_robot1'
    state.reference_frame = 'world'
    state.pose.position.x, state.pose.position.y, state.pose.position.z = x, y, z
    state.pose.orientation.w = 1.
    try:
        physics_pause(False, timeout)
        response = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)(state)
        if not response.success:
            raise RuntimeError('重置机器人失败: ' + response.status_message)
        print('已重置机器人位姿 xyz=({}, {}, {})，姿态为零，线速度/角速度为零。'.format(x, y, z))
        wait_detection(timeout)
    finally:
        physics_pause(True, timeout)


def status():
    """输出当前注册节点和话题连接；不保证这些话题的数据此刻仍新鲜。"""
    publishers, subscribers, services = rosgraph.Master('/aruco_docking_status').getSystemState()
    nodes = {node for _, group in publishers + subscribers + services for node in group}
    for name in ('/gazebo', '/decision_robot1', '/robot1/motion', '/aruco_det', '/aruco_posture', '/aruco_pose_logger'):
        print('{}: {}'.format(name, '已注册' if name in nodes else '未注册'))
    for topic in ('/gazebo/model_states', '/gazebo_robot1/camera_bottom/image_raw',
                  '/gazebo_robot1/camera_bottom/camera_info', '/aruco/pose', '/camera_aruco_data'):
        print('{} 发布者: {}'.format(topic, ', '.join(dict(publishers).get(topic, [])) or '无'))
    print('以上为连接状态；dock 前还会重新检查实际位姿数据。')


def main():
    """分派检查阶段；超时或冲突时返回非零退出码，交给 shell 决定是否继续启动。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=('preflight', 'detection', 'command', 'status', 'prepare', 'pause', 'resume'))
    parser.add_argument('--timeout', type=float, default=60.)
    parser.add_argument('--x', type=float, default=0.)
    parser.add_argument('--y', type=float, default=0.)
    parser.add_argument('--z', type=float, default=-.1)
    args = parser.parse_args()
    try:
        if not math.isfinite(args.timeout) or args.timeout <= 0:
            raise ValueError('timeout 必须是有限正数')
        if args.stage == 'preflight':
            preflight()
        else:
            rospy.init_node('aruco_docking_check', anonymous=True, disable_signals=True)
            if args.stage == 'detection':
                wait_detection(args.timeout)
            elif args.stage == 'command':
                wait_command(args.timeout)
            elif args.stage == 'prepare':
                prepare(args.timeout, args.x, args.y, args.z)
            elif args.stage in ('pause', 'resume'):
                physics_pause(args.stage == 'pause', args.timeout)
            else:
                status()
    except (RuntimeError, ValueError, OSError, rosgraph.MasterException, rospy.ROSException, rospy.ServiceException) as exc:
        print('检查失败 [{}]: {}'.format(args.stage, exc), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
