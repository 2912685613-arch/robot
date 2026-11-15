#!/usr/bin/env python3
"""真实替身进程测试：跨 session 清理、精确运行隔离、旧日志标记及 PID 身份核对。"""
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

PATH = Path(__file__).resolve().parents[1] / 'cleanup_aruco_docking.py'
SPEC = importlib.util.spec_from_file_location('cleanup_aruco', PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class CleanupTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.run = Path(self.temp.name) / 'aruco_docking_test'
        self.run.mkdir()
        self.children = []
        self.addCleanup(self.kill_children)

    def kill_children(self):
        for child in self.children:
            if child.poll() is None:
                child.kill()
            child.wait(timeout=3)

    def child(self, env, source='import time; time.sleep(120)'):
        process = subprocess.Popen([sys.executable, '-c', source], env=dict(os.environ, **env),
                                   start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.children.append(process)
        return process

    def test_detached_process_is_cleaned_but_other_run_survives(self):
        owned = self.child({'ARUCO_DOCKING_SESSION': str(self.run)})
        unrelated = self.child({'ARUCO_DOCKING_SESSION': str(self.run) + '_other'})
        deadline = time.monotonic() + 2
        while owned.pid not in {p.pid for p in MODULE.matching_processes(self.run)}:
            self.assertLess(time.monotonic(), deadline)
            time.sleep(.01)
        self.assertEqual(os.getsid(owned.pid), owned.pid)
        self.assertTrue(MODULE.cleanup(self.run, grace=.2))
        owned.wait(timeout=2)
        self.assertIsNone(unrelated.poll())

    def test_old_script_ros_log_marker_is_found(self):
        child = self.child({'ROS_LOG_DIR': str(self.run / 'ros')})
        self.assertIn(child.pid, {p.pid for p in MODULE.matching_processes(self.run)})
        self.assertTrue(MODULE.cleanup(self.run, grace=.2))
        child.wait(timeout=2)

    def test_stubborn_gui_escalates_to_kill(self):
        ready = Path(self.temp.name) / 'ready'
        source = ("import signal,time; from pathlib import Path; "
                  "signal.signal(signal.SIGINT,signal.SIG_IGN); "
                  "signal.signal(signal.SIGTERM,signal.SIG_IGN); "
                  "Path(%r).touch(); time.sleep(120)" % str(ready))
        child = self.child({'ARUCO_DOCKING_SESSION': str(self.run)}, source)
        deadline = time.monotonic() + 2
        while not ready.exists():
            self.assertLess(time.monotonic(), deadline)
            time.sleep(.01)
        self.assertTrue(MODULE.cleanup(self.run, grace=.1))
        child.wait(timeout=2)
        self.assertEqual(child.returncode, -signal.SIGKILL)

    def test_pid_reuse_does_not_signal_new_process(self):
        old = MODULE.Process(123, 1, '100', 'old')
        new = MODULE.Process(123, 1, '101', 'new')
        with patch.object(MODULE, 'inspect', return_value=new), patch.object(MODULE.os, 'kill') as kill:
            MODULE.signal_process(old, signal.SIGTERM)
            kill.assert_not_called()

    def test_scope_rejects_non_run_directory(self):
        with self.assertRaises(ValueError):
            MODULE.resolve_run(self.run.parent, str(self.run.parent))
        self.assertEqual(MODULE.resolve_run(self.run.parent, str(self.run)), self.run)


if __name__ == '__main__':
    unittest.main()
