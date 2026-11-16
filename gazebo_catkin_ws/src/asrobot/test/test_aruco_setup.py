#!/usr/bin/env python3
"""离线检查相机安装、标记尺寸与启动参数；不启动 ROS Master 或 Gazebo。"""
import math
from pathlib import Path
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml

SRC = Path(__file__).resolve().parents[2]


def rotation(rpy):
    """按 URDF 的固定轴 RPY 约定构造旋转矩阵，核对光学轴在机体系的方向。"""
    roll, pitch, yaw = map(float, rpy.split())
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll),
                             math.cos(pitch), math.sin(pitch),
                             math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


class ArucoSetupTests(unittest.TestCase):
    def test_bottom_camera_axes_and_optional_enable(self):
        """相机光学 Z 必须向下，且与位姿节点约定的轴向和光心偏移一致。"""
        path = SRC / 'new_gazebo_model/urdf/robot/gazebo_model.xacro'
        base = ['xacro', str(path), 'namespace:=gazebo_robot1', 'color:=Orange', 'debug:=0']
        without = ET.fromstring(subprocess.check_output(base))
        self.assertEqual(without.findall('.//sensor[@type="camera"]'), [])
        enabled = ET.fromstring(subprocess.check_output(
            base + ['enable_bottom_camera:=true', 'static_model:=true']))
        camera = enabled.find('.//sensor[@type="camera"]')
        self.assertIsNotNone(camera)
        plugin = camera.find('plugin')
        self.assertEqual(plugin.findtext('robotNamespace'), '/gazebo_robot1')
        self.assertEqual(plugin.findtext('cameraName'), 'camera_bottom')
        self.assertEqual(plugin.findtext('imageTopicName'), 'image_raw')
        self.assertEqual(plugin.findtext('cameraInfoTopicName'), 'camera_info')
        native = enabled.find('./joint[@name="gazebo_robot1/camera_bottom_joint"]/origin')
        optical = enabled.find('./joint[@name="gazebo_robot1/camera_bottom_optical_joint"]/origin')
        np.testing.assert_allclose(
            rotation(native.get('rpy')) @ rotation(optical.get('rpy')),
            [[0, -1, 0], [-1, 0, 0], [0, 0, -1]], atol=1e-12)
        np.testing.assert_allclose(list(map(float, native.get('xyz').split())), [0, 0, -0.12])
        self.assertEqual(enabled.findtext('./gazebo/static'), 'true')

    def test_static_demo_excludes_dynamics_plugins(self):
        """回归：静态模型加载水动力插件会导致 NaN 加速度断言；校验转换后的实际 SDF。"""
        path = SRC / 'new_gazebo_model/urdf/robot/gazebo_model.xacro'
        command = ['xacro', str(path), 'namespace:=gazebo_robot1', 'color:=Orange',
                   'debug:=0', 'enable_bottom_camera:=true']
        with tempfile.TemporaryDirectory() as temp:
            for static in ('true', 'false'):
                urdf = Path(temp) / ('robot_' + static + '.urdf')
                urdf.write_bytes(subprocess.check_output(command + ['static_model:=' + static]))
                sdf = ET.fromstring(subprocess.check_output(['gz', 'sdf', '-p', str(urdf)]))
                plugins = [p.get('filename') for p in sdf.findall('.//plugin')]
                self.assertEqual(plugins.count('libgazebo_ros_camera.so'), 1)
                if static == 'true':
                    self.assertEqual(sdf.findtext('model/static'), '1')
                    self.assertNotIn('libuuv_underwater_object_ros_plugin.so', plugins)
                    self.assertNotIn('libuuv_thruster_ros_plugin.so', plugins)
                    self.assertNotIn('libgazebo_ros_control.so', plugins)
                    self.assertEqual(plugins, ['libgazebo_ros_camera.so'])
                else:
                    self.assertNotEqual(sdf.findtext('model/static'), '1')
                    self.assertEqual(plugins.count('libuuv_underwater_object_ros_plugin.so'), 1)
                    self.assertEqual(plugins.count('libuuv_thruster_ros_plugin.so'), 4)
                    self.assertEqual(plugins.count('libgazebo_ros_control.so'), 1)

    def test_marker_id_and_metric_pose(self):
        """从实际纹理检测边框，检查材质物理尺寸能恢复已知的针孔相机距离。"""
        model = SRC / 'gazebo_world/models/aruco_demo_marker'
        image = cv2.imread(str(model / 'materials/textures/marker5.png'), cv2.IMREAD_GRAYSCALE)
        dictionary = cv2.aruco.getPredefinedDictionary(10)
        corners, ids, _ = cv2.aruco.detectMarkers(image, dictionary)
        self.assertEqual(set(ids.flatten().tolist()), {5, 18})
        outer_index = ids.flatten().tolist().index(5)
        plane = ET.parse(model / 'model.sdf').find('.//visual/geometry/plane')
        width = float(plane.findtext('size').split()[0])
        # ID5 黑边 800/1120 × 0.28 = 0.20 m；不能假设检测数组的第一项是外码。
        outer = corners[outer_index][0]
        self.assertAlmostEqual(width * (outer[1][0] - outer[0][0] + 1) / image.shape[1], 0.2)
        inner = corners[ids.flatten().tolist().index(18)][0]
        self.assertAlmostEqual(width * (inner[1][0] - inner[0][0] + 1) / image.shape[1], 0.02)
        np.testing.assert_allclose(outer.mean(axis=0), inner.mean(axis=0), atol=0.01)
        # 对纹理缩小并平移，得到已知投影：黑边宽 80 px，中心 (336,232)。
        # 因此 fx=320 时，Z=320*0.2/80=0.8 m，X=0.04 m，Y=-0.02 m。
        scaled = cv2.resize(image, (112, 112), interpolation=cv2.INTER_NEAREST)
        view = np.full((480, 640), 255, dtype=np.uint8)
        view[176:288, 280:392] = scaled
        corners, ids, _ = cv2.aruco.detectMarkers(view, dictionary)
        self.assertIn(5, ids.flatten().tolist())
        corners = [corners[ids.flatten().tolist().index(5)]]
        k = np.array([[320., 0., 320.], [0., 320., 240.], [0., 0., 1.]])
        _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.2, k, np.zeros(5))
        np.testing.assert_allclose(tvecs[0][0], [0.04, -0.02, 0.8], atol=0.015)

    def test_concentric_markers_distance_overlap_and_close_range(self):
        """投影真实双码纹理，确认大码、小码重叠区及 3 cm 近距离可识别并恢复米制位置。"""
        image = cv2.imread(str(SRC / 'gazebo_world/models/aruco_demo_marker/materials/textures/marker5.png'),
                           cv2.IMREAD_GRAYSCALE)
        dictionary = cv2.aruco.getPredefinedDictionary(10)
        k = np.array([[320., 0., 320.5], [0., 320., 240.5], [0., 0., 1.]])
        for depth, required in [(0.78, {5}), (0.5, {5, 18}), (0.4, {5, 18}),
                                (0.35, {5, 18}), (0.3, {5, 18}), (0.1, {18}),
                                (0.05, {18}), (0.03, {18})]:
            with self.subTest(depth=depth):
                scale = 320. * .28 / depth / image.shape[0]
                center = image.shape[0] / 2.
                transform = np.array([[scale, 0, 320.5-center*scale],
                                      [0, scale, 240.5-center*scale]])
                view = cv2.warpAffine(image, transform, (640, 480),
                                     flags=cv2.INTER_LINEAR, borderValue=180)
                corners, ids, _ = cv2.aruco.detectMarkers(view, dictionary)
                found = [] if ids is None else ids.flatten().tolist()
                self.assertTrue(required <= set(found), (depth, found))
                estimates = []
                for marker_id in required:
                    length = .2 if marker_id == 5 else .02
                    _, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [corners[found.index(marker_id)]], length, k, np.zeros(5))
                    estimates.append(tvec[0][0])
                    # 0.5 m 处小码仅约 13 px，距离误差允许 20%；近处按 10%/3 mm 检查。
                    np.testing.assert_allclose(tvec[0][0], [0, 0, depth], atol=max(.003, depth*(.2 if marker_id == 18 and depth >= .4 else .1)))
                if len(estimates) == 2:
                    np.testing.assert_allclose(estimates[0], estimates[1], atol=depth*(.2 if depth >= .4 else .1))

    def test_launch_resolves_topics_and_marker_length(self):
        """解析完整示例，检查精简话题、机体偏移、材质尺寸和标定模式连在一起。"""
        params = yaml.safe_load(subprocess.check_output(
            ['roslaunch', '--dump-params', 'asrobot', 'aruco_sim.launch', 'namespace:=gazebo_robot2']))
        self.assertEqual(params['/aruco_det/sub_image_topic'], '/gazebo_robot2/camera_bottom/image_raw')
        self.assertEqual(params['/aruco_det/camera_info_topic'], '/gazebo_robot2/camera_bottom/camera_info')
        self.assertEqual(params['/aruco_posture/model_name'], 'gazebo_robot2')
        self.assertEqual(params['/aruco_posture/camera_frame_id'], 'gazebo_robot2/camera_bottom_optical_frame')
        self.assertEqual(params['/aruco_det/aruco_length_outer'], 0.2)
        self.assertEqual(params['/aruco_det/aruco_id_inner'], 18)
        self.assertEqual(params['/aruco_det/aruco_length_inner'], 0.02)
        self.assertAlmostEqual(params['/aruco_det/switch_depth_threshold_to_inner'], 0.30)
        self.assertEqual(params['/aruco_det/inner_confirm_frames'], 3)
        self.assertEqual(params['/aruco_det/inner_min_side_pixels'], 24)
        self.assertTrue(params['/aruco_det/use_camera_info'])
        self.assertEqual(params['/aruco_posture/camera_z'], -0.12)
        self.assertEqual(params['/aruco_posture/position_z_offset'], 0)
        self.assertTrue(params['/aruco_pose_logger/use_camera_info'])
        self.assertEqual(params['/aruco_pose_logger/model_name'], 'gazebo_robot2')
        self.assertEqual(params['/aruco_pose_logger/marker_model_name'], 'aruco_demo_marker')
        self.assertEqual(Path(params['/aruco_pose_logger/log_dir']).resolve(), SRC.parent / 'logs')
        self.assertEqual(params['/aruco_pose_logger/camera_info_topic'],
                         '/gazebo_robot2/camera_bottom/camera_info')
        self.assertGreater(params['/aruco_det/switch_depth_threshold_to_outer'],
                           params['/aruco_det/switch_depth_threshold_to_inner'])


if __name__ == '__main__':
    unittest.main()
