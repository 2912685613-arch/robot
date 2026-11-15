#!/usr/bin/env python3
"""准备阶段服务调用检查，不启动 Gazebo，不连接 ROS Master。"""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

PATH = Path(__file__).resolve().parents[1] / 'wait_aruco_docking.py'
SPEC = importlib.util.spec_from_file_location('wait_docking', PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class PrepareTests(unittest.TestCase):
    def test_prepare_resets_pose_twist_then_waits_and_pauses(self):
        calls = []
        def proxy(name, _):
            def invoke(*args):
                calls.append((name, args))
                return Mock(success=True)
            return invoke
        with patch.object(MODULE.rospy, 'wait_for_service'), \
             patch.object(MODULE.rospy, 'ServiceProxy', side_effect=proxy), \
             patch.object(MODULE, 'wait_detection', side_effect=lambda _: calls.append(('detect', ()))):
            MODULE.prepare(1., .1, .2, -.3)
        self.assertEqual([c[0] for c in calls], ['/gazebo/unpause_physics', '/gazebo/set_model_state',
                                                'detect', '/gazebo/pause_physics'])
        state = calls[1][1][0]
        self.assertEqual(state.model_name, 'gazebo_robot1')
        self.assertEqual(state.reference_frame, 'world')
        self.assertEqual(state.pose.position.z, -.3)
        self.assertEqual(state.pose.orientation.w, 1.)
        self.assertEqual(state.twist.linear.x, 0.)
        self.assertEqual(state.twist.angular.z, 0.)

    def test_no_detection_still_pauses(self):
        with patch.object(MODULE, 'physics_pause') as pause, \
             patch.object(MODULE.rospy, 'wait_for_service'), \
             patch.object(MODULE.rospy, 'ServiceProxy', return_value=Mock(return_value=Mock(success=True))), \
             patch.object(MODULE, 'wait_detection', side_effect=RuntimeError('no marker')):
            with self.assertRaises(RuntimeError):
                MODULE.prepare(1., 0., 0., -.1)
        self.assertEqual([c.args[0] for c in pause.call_args_list], [False, True])

    def test_model_service_failure_still_pauses(self):
        with patch.object(MODULE, 'physics_pause') as pause, \
             patch.object(MODULE.rospy, 'wait_for_service'), \
             patch.object(MODULE.rospy, 'ServiceProxy', return_value=Mock(
                 return_value=Mock(success=False, status_message='missing robot'))):
            with self.assertRaises(RuntimeError):
                MODULE.prepare(1., 0., 0., -.1)
        self.assertTrue(pause.call_args_list[-1].args[0])


if __name__ == '__main__':
    unittest.main()
