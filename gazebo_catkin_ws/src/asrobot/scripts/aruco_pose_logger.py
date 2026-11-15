#!/usr/bin/env python3
"""记录机器人、相机、标记的位姿关系；只订阅数据，不参与控制或定位。"""
import csv
from datetime import datetime, timezone
import json
import logging
import math
import sys
from pathlib import Path
import tempfile
import threading
import time

import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import CameraInfo
from tf.transformations import euler_from_matrix, quaternion_from_matrix, quaternion_matrix

from asrobot.msg import camera_data

# Txy 表示从 y 系到 x 系；这些名称同时用作 CSV 前缀，避免混淆参考系。
RELATIONS = (
    'robot_world', 'camera_body', 'camera_world', 'marker_camera_est',
    'marker_body_est', 'marker_world_est', 'marker_world_truth',
    'marker_camera_truth', 'marker_body_truth',
)
POSE_FIELDS = ('x_m', 'y_m', 'z_m', 'qx', 'qy', 'qz', 'qw', 'roll_deg', 'pitch_deg', 'yaw_deg')


def wall_stamp():
    """返回带 UTC 时区的实际时间；与日志中的 ROS 仿真时间分开记录。"""
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds')


def pose_matrix(pose):
    """把 ROS Pose 转为刚体矩阵，拒绝无效数值或零四元数，避免产生假位姿。"""
    q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    p = np.array([pose.position.x, pose.position.y, pose.position.z])
    if not np.isfinite(q).all() or not np.isfinite(p).all() or np.linalg.norm(q) < 1e-9:
        raise ValueError('non-finite pose or zero quaternion')
    result = quaternion_matrix(q / np.linalg.norm(q))
    result[:3, 3] = p
    return result


def pose_values(matrix):
    """返回 CSV 的位置、xyzw 四元数和 RPY 角；缺少/过期的数据留空。"""
    if matrix is None:
        return [''] * len(POSE_FIELDS)
    return (matrix[:3, 3].tolist() + quaternion_from_matrix(matrix).tolist()
            + np.degrees(euler_from_matrix(matrix)).tolist())


def relationships(robot, camera_body, marker_camera, marker_world_truth):
    """组合位姿关系；没有机器人真值时仍能记录相机/机体相对视觉结果。"""
    camera_world = None if robot is None else robot @ camera_body
    marker_body = None if marker_camera is None else camera_body @ marker_camera
    marker_world = None if robot is None or marker_body is None else robot @ marker_body
    return {
        'robot_world': robot, 'camera_body': camera_body, 'camera_world': camera_world,
        'marker_camera_est': marker_camera, 'marker_body_est': marker_body,
        'marker_world_est': marker_world, 'marker_world_truth': marker_world_truth,
        'marker_camera_truth': (None if camera_world is None or marker_world_truth is None
                                else np.linalg.inv(camera_world) @ marker_world_truth),
        'marker_body_truth': (None if robot is None or marker_world_truth is None
                              else np.linalg.inv(robot) @ marker_world_truth),
    }


class ArucoPoseLogger:
    """缓存订阅消息，以固定墙钟频率落盘，并记录数据源状态变化与 ROS 诊断消息。"""

    def __init__(self):
        """读取参数并创建独立日志目录；目录不可写时明确报错，不静默丢弃日志。"""
        self.lock = threading.RLock()
        self.model_name = rospy.get_param('~model_name', 'gazebo_robot1')
        self.marker_name = rospy.get_param('~marker_model_name', '')
        self.frame = rospy.get_param('~camera_frame_id', self.model_name + '/camera_bottom_optical_frame')
        self.rate = float(rospy.get_param('~log_rate', 2.0))
        self.use_camera_info = rospy.get_param("~use_camera_info", True)
        self.timeout = float(rospy.get_param('~pose_timeout', 0.5))
        self.source_timeout = float(rospy.get_param('~source_timeout', 2.0))
        if not all(math.isfinite(v) and v > 0 for v in (self.rate, self.timeout, self.source_timeout)):
            raise ValueError('log_rate, pose_timeout and source_timeout must be positive and finite')
        self.camera_body = np.eye(4)
        self.camera_body[:3, :3] = [[0, -1, 0], [-1, 0, 0], [0, 0, -1]]
        self.camera_body[:3, 3] = [rospy.get_param('~camera_' + axis, default)
                                   for axis, default in zip('xyz', (0., 0., -0.12))]
        if not np.isfinite(self.camera_body).all():
            raise ValueError('camera translation must be finite')
        self.robot = self.truth = self.marker_camera = self.published = None
        self.model_wall = self.pose_wall = self.info_wall = self.published_wall = None
        self.model_ros = self.pose_ros = None
        self.calibration = None
        self.states = {}
        self.closed = False
        self.subscribers = []

        # UTC+微秒+随机后缀：重启和同时运行不会覆盖已有日志。
        root = Path(rospy.get_param('~log_dir')).expanduser().resolve()
        root.mkdir(parents=True, exist_ok=True)
        safe_name = ''.join(c if c.isalnum() or c in '_-' else '_' for c in self.model_name)
        prefix = 'aruco_{}_{}_'.format(safe_name, datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%fZ'))
        self.directory = Path(tempfile.mkdtemp(prefix=prefix, dir=str(root)))
        self.log = logging.getLogger('aruco_pose_logger.' + str(self.directory))
        self.log.setLevel(logging.INFO)
        self.log.propagate = False
        self.handler = logging.FileHandler(str(self.directory / 'events.log'), encoding='utf-8')
        self.handler.setFormatter(logging.Formatter('%(asctime)sZ %(levelname)s %(message)s'))
        self.handler.formatter.converter = time.gmtime
        self.log.addHandler(self.handler)
        self.csv_file = (self.directory / 'poses.csv').open('w', newline='', encoding='utf-8', buffering=1)
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(['wall_time_utc', 'ros_time_s', 'image_stamp_s', 'pose_age_s',
                           'model_received_ros_s', 'models_wall_age_s', 'robot_valid',
                           'pose_valid', 'truth_valid', 'camera_info_fresh',
                           'position_error_m']
                          + [relation + '_' + field for relation in RELATIONS for field in POSE_FIELDS]
                          + ['published_body_' + field for field in
                             ('x_m', 'y_m', 'z_m', 'roll_deg', 'pitch_deg', 'yaw_deg')])
        config = rospy.get_param(rospy.get_name(), {})
        # 保存参数服务器配置及日志节点的生效参数；代码内部默认值不一定出现在服务器中。
        config['detection_config'] = rospy.get_param('/aruco_det', {})
        config['posture_config'] = rospy.get_param('/aruco_posture', {})
        config['motion_config'] = rospy.get_param('/robot1/motion/docking', {})
        config['created_utc'] = wall_stamp()
        config['effective_logger_config'] = {
            'model_name': self.model_name, 'marker_model_name': self.marker_name,
            'camera_frame_id': self.frame, 'camera_body_matrix': self.camera_body.tolist(),
            'log_rate_hz': self.rate, 'pose_timeout_s': self.timeout,
            'source_timeout_wall_s': self.source_timeout, 'use_camera_info': self.use_camera_info,
        }
        config['notes'] = ('Txy maps y to x; positions=m; RPY=deg; quaternion=xyzw. '
                           'ModelStates has no header: truth uses latest received sample, not time synchronization. '
                           'marker_model_name must identify the currently detected marker center. '
                           '/aruco/pose does not carry marker ID; no automatic ID association. '
                           'Published body columns contain the latest valid received sample; its source time is not a dedicated CSV column.')
        (self.directory / 'config.json').write_text(json.dumps(config, ensure_ascii=False, indent=2))
        self.log.info('START model=%s marker_model=%s optical_frame=%s log_rate=%.2f Hz',
                      self.model_name, self.marker_name or '(truth comparison disabled)', self.frame, self.rate)
        self.log.info('UNITS position=m RPY=deg quaternion=xyzw; truth and estimate use latest samples, not synchronized.')
        self.log.info('CAMERA_INSTALL %s', self.format_pose(self.camera_body))
        rospy.loginfo('ArUco diagnostic logs: %s', self.directory)
        rospy.on_shutdown(self.close)
        self.subscribers = [
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.models_cb, queue_size=1),
            rospy.Subscriber('/aruco/pose', PoseStamped, self.pose_cb, queue_size=1),
            rospy.Subscriber('/camera_aruco_data', camera_data, self.published_cb, queue_size=1),
            rospy.Subscriber(rospy.get_param('~camera_info_topic'), CameraInfo, self.info_cb, queue_size=1),
            rospy.Subscriber('/rosout_agg', Log, self.rosout_cb, queue_size=100),
        ]

    @staticmethod
    def format_pose(matrix):
        """生成便于阅读的 xyz 和 RPY 文本；完整四元数同时保存在 CSV 中。"""
        if matrix is None:
            return 'unavailable'
        v = pose_values(matrix)
        return 'xyz=({:.4f},{:.4f},{:.4f})m rpy=({:.2f},{:.2f},{:.2f})deg'.format(*(v[:3] + v[7:]))

    def models_cb(self, msg):
        """从同一条 ModelStates 中提取机器人和指定标记真值，模型缺失时清空旧值。"""
        with self.lock:
            if self.closed:
                return
            self.robot = self.truth = None
            for name, pose in zip(msg.name, msg.pose):
                if name not in (self.model_name, self.marker_name):
                    continue
                try:
                    value = pose_matrix(pose)
                except ValueError as exc:
                    self.log.warning('INVALID_MODEL_POSE model=%s reason=%s', name, exc)
                    continue
                if name == self.model_name:
                    self.robot = value
                if self.marker_name and name == self.marker_name:
                    self.truth = value
            self.model_wall, self.model_ros = time.monotonic(), rospy.get_time()

    def pose_cb(self, msg):
        """保留图像采集时间，拒绝错误参考系或无效四元数，避免写出假的坐标关系。"""
        with self.lock:
            if self.closed:
                return
            if msg.header.frame_id != self.frame:
                if self.states.get('frame_error') != msg.header.frame_id:
                    self.log.warning('FRAME_MISMATCH actual=%s expected=%s', msg.header.frame_id, self.frame)
                    self.states['frame_error'] = msg.header.frame_id
                return
            try:
                self.marker_camera = pose_matrix(msg.pose)
            except ValueError as exc:
                self.log.warning('INVALID_DETECTION reason=%s', exc)
                return
            self.pose_ros, self.pose_wall = msg.header.stamp.to_sec(), time.monotonic()

    def published_cb(self, msg):
        """缓存下游实际收到的机体结果；其消息没有 header，单独记录而不假定精确同步。"""
        with self.lock:
            if self.closed:
                return
            if not msg.aruco_valid or not np.isfinite(msg.detection_stamp):
                self.published = None
                self.published_wall = None
                return
            values = [msg.position_x, msg.position_y, msg.position_z,
                      msg.cam_roll, msg.cam_pitch, msg.cam_yaw]
            if not np.isfinite(values).all():
                self.log.warning('INVALID_PUBLISHED_BODY non-finite data')
                return
            self.published, self.published_wall = values, time.monotonic()

    def info_cb(self, msg):
        """首次及标定变化时记录图像尺寸、K/D 和参考系，避免逐帧重复刷标定日志。"""
        with self.lock:
            if self.closed:
                return
            calibration = {'width': msg.width, 'height': msg.height, 'frame': msg.header.frame_id,
                           'distortion_model': msg.distortion_model, 'K': list(msg.K), 'D': list(msg.D)}
            if calibration != self.calibration:
                self.log.info('CAMERA_INFO %s', json.dumps(calibration, ensure_ascii=False))
                self.calibration = calibration
            self.info_wall = time.monotonic()

    def rosout_cb(self, msg):
        """收集检测/转换和对接控制状态、限速、HOLD、丢失；不转发自身日志。"""
        if msg.name not in ('/aruco_det', '/aruco_posture', '/robot1/motion'):
            return
        with self.lock:
            if self.closed:
                return
            level = {Log.DEBUG: logging.DEBUG, Log.INFO: logging.INFO, Log.WARN: logging.WARNING,
                     Log.ERROR: logging.ERROR, Log.FATAL: logging.CRITICAL}.get(msg.level, logging.INFO)
            self.log.log(level, 'ROS node=%s stamp=%.6f %s', msg.name, msg.header.stamp.to_sec(), msg.msg)

    def sample(self):
        """记录一组当前有效的关系和状态；过期数据留空，不继续当作当前定位结果输出。"""
        with self.lock:
            if self.closed:
                return
            now, wall = rospy.get_time(), time.monotonic()
            def fresh(received):
                return received is not None and wall - received <= self.source_timeout
            age = None if self.pose_ros is None else now - self.pose_ros
            robot_ok = fresh(self.model_wall) and self.robot is not None
            truth_ok = fresh(self.model_wall) and self.truth is not None
            pose_ok = fresh(self.pose_wall) and age is not None and 0 <= age <= self.timeout
            info_ok = fresh(self.info_wall)
            current = {'robot': robot_ok, 'detection': pose_ok, 'camera_info': info_ok,
                       'body_output': fresh(self.published_wall)}
            if not self.use_camera_info:
                current.pop('camera_info')
            if self.marker_name:
                current['marker_truth'] = truth_ok
            for name, valid in current.items():
                if self.states.get(name) != valid:
                    self.log.log(logging.INFO if valid else logging.WARNING,
                                 'STATE %s=%s ros_time=%.6f', name, 'ready' if valid else 'missing_or_stale', now)
                    self.states[name] = valid
            result = relationships(self.robot if robot_ok else None, self.camera_body,
                                   self.marker_camera if pose_ok else None, self.truth if truth_ok else None)
            error = (None if result['marker_world_est'] is None or not truth_ok else
                     np.linalg.norm(result['marker_world_est'][:3, 3] - self.truth[:3, 3]))
            # body_output 可能包含 position_z_offset；保留原始发布值供检查，不反向修改估计。
            published = self.published if fresh(self.published_wall) else [''] * 6
            row = [wall_stamp(), now, '' if self.pose_ros is None else self.pose_ros,
                   '' if age is None else age, '' if self.model_ros is None else self.model_ros,
                   '' if self.model_wall is None else wall - self.model_wall,
                   int(robot_ok), int(pose_ok), int(truth_ok), int(info_ok), '' if error is None else error]
            for relation in RELATIONS:
                row.extend(pose_values(result[relation]))
            self.csv.writerow(row + published)
            self.csv_file.flush()
            self.log.info('POSE ros_time=%.6f image_stamp=%s age_s=%s %s position_error_m=%s published_body=%s',
                          now, self.pose_ros, age,
                          ' | '.join(name + ': ' + self.format_pose(result[name]) for name in RELATIONS),
                          'unavailable' if error is None else '{:.6f}'.format(error), published)

    def run(self):
        """使用墙钟限频；即使仿真暂停或 /clock 尚未到达，也能记录等待状态。"""
        while not rospy.is_shutdown():
            start = time.monotonic()
            self.sample()
            rospy.rostime.wallsleep(max(0., 1. / self.rate - (time.monotonic() - start)))

    def close(self):
        """关闭时记录结束时间并刷新文件，与回调和采样互斥，防止写入已关闭文件。"""
        with self.lock:
            if self.closed:
                return
            self.closed = True
            self.log.info('STOP ros_time=%.6f', rospy.get_time())
            self.csv_file.close()
            self.handler.close()
            self.log.removeHandler(self.handler)


def main():
    """启动日志节点；初始化或写盘失败时明确报告错误，始终关闭已创建的文件。"""
    rospy.init_node('aruco_pose_logger')
    logger = None
    try:
        logger = ArucoPoseLogger()
        logger.run()
    except (OSError, ValueError) as exc:
        rospy.logfatal('ArUco pose logging failed: %s', exc)
        return 1
    finally:
        if logger is not None:
            logger.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
