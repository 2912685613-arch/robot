#!/usr/bin/env python3
"""离线验证位姿日志的数值、缺失处理和落盘；不启动 ROS Master/Gazebo。"""
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseStamped
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import CameraInfo
import rospy

PATH = Path(__file__).resolve().parents[1] / 'scripts/aruco_pose_logger.py'
SPEC = importlib.util.spec_from_file_location('aruco_pose_logger', PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def pose(x=0., y=0., z=0.):
    """构造有效单位四元数的测试位姿。"""
    value = Pose()
    value.position.x, value.position.y, value.position.z = x, y, z
    value.orientation.w = 1.
    return value


class ArucoLoggerTests(unittest.TestCase):
    def setUp(self):
        """模拟 ROS 参数和时间，把测试输出隔离到临时目录。"""
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.params = {'~log_dir': self.temp.name, '~marker_model_name': 'aruco_demo_marker',
                       '~camera_info_topic': '/gazebo_robot1/camera_bottom/camera_info'}
        self.now, self.wall = 10., 100.
        def get_param(name, *default):
            if name in self.params:
                return self.params[name]
            if default:
                return default[0]
            raise KeyError(name)
        self.patchers = [
            patch.multiple(MODULE.rospy, get_param=Mock(side_effect=get_param),
                           get_name=Mock(return_value='/aruco_pose_logger'),
                           get_time=Mock(side_effect=lambda: self.now),
                           loginfo=Mock(), on_shutdown=Mock(), Subscriber=Mock()),
            patch.object(MODULE.time, 'monotonic', side_effect=lambda: self.wall),
        ]
        for item in self.patchers:
            item.start()
            self.addCleanup(item.stop)
        self.logger = MODULE.ArucoPoseLogger()
        self.addCleanup(self.logger.close)

    def fill_sources(self):
        """注入默认几何配置的模型真值、检测和发布结果，不发送任何 ROS 消息。"""
        models = ModelStates(name=['gazebo_robot1', 'aruco_demo_marker'],
                             pose=[pose(), pose(z=-1.)])
        self.logger.models_cb(models)
        detection = PoseStamped()
        detection.header.frame_id = 'gazebo_robot1/camera_bottom_optical_frame'
        detection.header.stamp = rospy.Time.from_sec(10.)
        detection.pose = pose(z=0.88)
        self.logger.pose_cb(detection)
        info = CameraInfo()
        info.width, info.height = 640, 480
        info.header.frame_id = detection.header.frame_id
        info.K = [320., 0., 320., 0., 320., 240., 0., 0., 1.]
        self.logger.info_cb(info)
        self.logger.info_cb(info)
        published = MODULE.camera_data()
        published.aruco_valid = True
        published.detection_stamp = 10.
        published.position_z = -1.
        self.logger.published_cb(published)

    def rows(self):
        """读取已刷新到盘的 CSV，验证用户实际获得的日志字段。"""
        with (self.logger.directory / 'poses.csv').open() as stream:
            return list(csv.DictReader(stream))

    def test_geometry_with_rotated_robot(self):
        """机器人转向后，世界系位置仍必须由正确方向的坐标链计算。"""
        robot = np.array([[0., -1., 0., 1.], [1., 0., 0., 2.],
                          [0., 0., 1., 3.], [0., 0., 0., 1.]])
        detection = np.eye(4)
        detection[:3, 3] = [0.2, -0.1, 0.88]
        result = MODULE.relationships(robot, self.logger.camera_body, detection, None)
        np.testing.assert_allclose(result['marker_body_est'][:3, 3], [0.1, -0.2, -1.])
        np.testing.assert_allclose(result['marker_world_est'][:3, 3], [1.2, 2.1, 2.])
        np.testing.assert_allclose(result['camera_world'][:3, 3], [1., 2., 2.88])
        self.assertIsNone(result['marker_world_truth'])

    def test_missing_sources_are_blank(self):
        """未收到机器人/检测时，不能用单位矩阵伪造有效位姿。"""
        self.logger.sample()
        row = self.rows()[0]
        self.assertEqual(row['pose_valid'], '0')
        self.assertEqual(row['robot_valid'], '0')
        self.assertEqual(row['robot_world_z_m'], '')
        self.assertEqual(row['marker_world_est_z_m'], '')
        self.assertEqual(float(row['camera_body_z_m']), -0.12)
        self.assertIn('missing_or_stale', (self.logger.directory / 'events.log').read_text())

    def test_valid_snapshot_and_calibration(self):
        """核对三种坐标关系、真值误差、发布值和标定去重。"""
        self.fill_sources()
        self.logger.sample()
        row = self.rows()[0]
        self.assertEqual(row['pose_valid'], '1')
        self.assertEqual(float(row['marker_camera_est_z_m']), 0.88)
        self.assertAlmostEqual(float(row['marker_body_est_z_m']), -1.)
        self.assertAlmostEqual(float(row['marker_world_est_z_m']), -1.)
        self.assertAlmostEqual(float(row['marker_camera_truth_z_m']), 0.88)
        self.assertAlmostEqual(float(row['position_error_m']), 0.)
        self.assertEqual(float(row['published_body_z_m']), -1.)
        log = (self.logger.directory / 'events.log').read_text()
        self.assertEqual(log.count('CAMERA_INFO '), 1)
        self.assertIn('xyz=', log)
        config = json.loads((self.logger.directory / 'config.json').read_text())
        self.assertIn('not time synchronization', config['notes'])

    def test_timeout_and_paused_clock_do_not_reuse_estimates(self):
        """分别核对图像时间超时和仿真暂停后的墙钟超时。"""
        self.fill_sources()
        self.logger.sample()
        self.now = 11.
        self.logger.sample()
        row = self.rows()[-1]
        self.assertEqual(row['pose_valid'], '0')
        self.assertEqual(row['marker_camera_est_z_m'], '')
        self.assertEqual(row['truth_valid'], '1')
        self.now, self.wall = 10., 103.
        self.logger.sample()
        row = self.rows()[-1]
        self.assertEqual(row['pose_valid'], '0')
        self.assertEqual(row['truth_valid'], '0')
        self.assertEqual(row['published_body_z_m'], '')

    def test_bad_frames_invalid_quaternion_and_removed_models(self):
        """错误参考系、零四元数和被删除模型不会进入位姿日志。"""
        message = PoseStamped()
        message.header.frame_id = 'wrong_frame'
        message.pose = pose(z=.88)
        self.logger.pose_cb(message)
        self.assertIsNone(self.logger.marker_camera)
        message.header.frame_id = self.logger.frame
        message.pose.orientation.w = 0.
        self.logger.pose_cb(message)
        self.assertIsNone(self.logger.marker_camera)
        self.fill_sources()
        self.logger.models_cb(ModelStates())
        self.logger.sample()
        row = self.rows()[-1]
        self.assertEqual(row['robot_valid'], '0')
        self.assertEqual(row['truth_valid'], '0')
        self.assertEqual(row['pose_valid'], '1')
        self.assertEqual(row['marker_world_est_z_m'], '')

    def test_ros_errors_and_run_isolation(self):
        """错误消息落盘，其他节点不刷入文件，重启不覆盖之前的日志。"""
        self.logger.rosout_cb(Log(name='/aruco_det', level=Log.ERROR, msg='detector failure'))
        self.logger.rosout_cb(Log(name='/unrelated', level=Log.INFO, msg='ignore this'))
        self.logger.close()
        self.logger.sample()  # 关闭后不再写文件。
        log = (self.logger.directory / 'events.log').read_text()
        self.assertIn('detector failure', log)
        self.assertNotIn('ignore this', log)
        self.assertIn('STOP', log)
        second = MODULE.ArucoPoseLogger()
        try:
            self.assertNotEqual(self.logger.directory, second.directory)
        finally:
            second.close()


if __name__ == '__main__':
    unittest.main()
