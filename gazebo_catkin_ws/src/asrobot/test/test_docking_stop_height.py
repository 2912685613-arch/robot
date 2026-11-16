#!/usr/bin/env python3
"""编译实际控制/分配头文件；不启动ROS/Gazebo。旧15cm停机测试改为HOLD回归测试。"""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class DockingControlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.binary = Path(cls.temp.name)/'docking_checks'
        subprocess.run(['g++', '-std=c++11', '-O1', '-I'+str(ROOT/'include'),
                        str(ROOT/'test/docking_control_checks.cpp'), '-o',str(cls.binary)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_actual_control_and_allocator(self):
        for case in ('validity','numeric','speed','hold','align','heading','clocks','cached','allocation','plant','recovery','selector','latency_trace','transport_stall','transport_recovery_bad_feedback'):
            with self.subTest(case=case):
                subprocess.run([str(self.binary),case],check=True)

if __name__ == '__main__':
    unittest.main()
