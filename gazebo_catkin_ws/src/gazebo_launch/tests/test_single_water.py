#!/usr/bin/env python3
"""Model/bringup regressions; run after sourcing devel/setup.bash."""
import math
from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import yaml

SRC = Path(__file__).resolve().parents[2]


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return (np.eye(3) * math.cos(angle)
            + (1 - math.cos(angle)) * np.outer(axis, axis)
            + math.sin(angle) * cross)


class SingleWaterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.models = {}
        model = SRC / "new_gazebo_model/urdf/robot/gazebo_model.xacro"
        for namespace in ("gazebo_robot1", "gazebo_robot2"):
            xml = subprocess.check_output([
                "xacro", str(model), "namespace:=" + namespace, "color:=Orange"
            ])
            cls.models[namespace] = ET.fromstring(xml)
        cls.calibration = yaml.safe_load((
            SRC / "new_gazebo_model/config/joint_command_calibration.yaml"
        ).read_text())

    def test_thruster_plugin_references_exist(self):
        for namespace, model in self.models.items():
            joints = {j.get("name") for j in model.findall("joint")}
            links = {l.get("name") for l in model.findall("link")}
            plugins = [p for p in model.findall("gazebo/plugin")
                       if p.find("thrusterID") is not None]
            self.assertEqual(len(plugins), 4)
            for plugin in plugins:
                with self.subTest(namespace=namespace, plugin=plugin.get("name")):
                    self.assertIn(plugin.findtext("jointName"), joints)
                    self.assertIn(plugin.findtext("linkName"), links)

    def thrust_geometry(self, tilt_deg, horizontal_deg=(0, 0, 0, 0)):
        model = self.models["gazebo_robot1"]
        joints = {j.find("child").get("link"): j for j in model.findall("joint")}
        scale = self.calibration["joint_command_scale"]
        offset = self.calibration["joint_command_offset_deg"]
        self.assertEqual(len(scale), 12)
        self.assertEqual(len(offset), 12)
        angles = {}
        for i in range(4):
            controller = [horizontal_deg[i], -2 if i == 0 else 0, -14 + tilt_deg]
            for k, name in enumerate(("H", "M", "V")):
                channel = 3 * i + k
                angles[name + str(i + 1)] = math.radians(
                    scale[channel] * controller[k] + offset[channel]
                )

        def orientation(link):
            if link not in joints:
                return np.eye(4)
            joint = joints[link]
            r, p, y = map(float, joint.find("origin").get("rpy", "0 0 0").split())
            axis = list(map(float, joint.find("axis").get("xyz").split()))
            transform = np.eye(4)
            transform[:3, :3] = (rotation([0, 0, 1], y)
                    @ rotation([0, 1, 0], p)
                    @ rotation([1, 0, 0], r)
                    @ rotation(axis, angles.get(joint.get("name"), 0)))
            transform[:3, 3] = list(map(float, joint.find("origin").get("xyz", "0 0 0").split()))
            return orientation(joint.find("parent").get("link")) @ transform

        axes, positions = [], []
        for i in range(4):
            link = "gazebo_robot1/thruster_%d" % i
            axis = list(map(float, joints[link].find("axis").get("xyz").split()))
            pose = orientation(link)
            axes.append(pose[:3, :3] @ axis)
            positions.append(pose[:3, 3])
        return positions, axes

    def thrust_axes(self, tilt_deg):
        return self.thrust_geometry(tilt_deg)[1]

    def test_manual_yaw_has_opposing_moments(self):
        # 校验 CAD 几何与关节校准后的真实力臂，而非只校验分配公式的符号。
        for tilt in (-10, 0, 10):
            positions, axes = self.thrust_geometry(tilt, (40, -40, 40, -40))
            moments = [np.cross(p, a)[2] for p, a in zip(positions, axes)]
            for i, moment in enumerate(moments):
                self.assertGreater(moment * (1 if i in (0, 2) else -1), 0.03)
            for speeds, sign in [((55, 45, 55, 45), 1), ((45, 55, 45, 55), -1)]:
                torque = sum(m * 0.00031 * u * u for m, u in zip(moments, speeds))
                self.assertGreater(sign * torque, 0.01)
            neutral = sum(m * 0.00031 * 50 * 50 for m in moments)
            self.assertLess(abs(neutral), 0.002)

    def test_neutral_thrusters_are_horizontal(self):
        for axis in self.thrust_axes(0):
            self.assertLess(abs(axis[2]), 0.001)

    def test_descent_steers_all_four_thrusters_down(self):
        for axis in self.thrust_axes(-10):
            self.assertLess(axis[2], -0.15)

    def test_ascent_steers_all_four_thrusters_up(self):
        for axis in self.thrust_axes(10):
            self.assertGreater(axis[2], 0.15)

    def test_world_is_throttled_without_shadows(self):
        world = ET.parse(SRC / "gazebo_world/worlds/single_water_fast.world").getroot().find("world")
        physics = world.find("physics")
        step = float(physics.findtext("max_step_size"))
        rate = float(physics.findtext("real_time_update_rate"))
        self.assertAlmostEqual(step * rate, 1)
        self.assertEqual(world.findtext("scene/shadows"), "false")
        self.assertTrue(all(light.findtext("cast_shadows") == "false"
                            for light in world.findall("light")))

    def test_start_script_shell_syntax(self):
        subprocess.run(["bash", "-n", str(SRC / "gazebo_launch/single_water.sh")], check=True)


if __name__ == "__main__":
    unittest.main()
