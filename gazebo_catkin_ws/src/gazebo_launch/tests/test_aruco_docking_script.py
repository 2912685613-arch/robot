#!/usr/bin/env python3
"""用替身命令验证后台启动、故障处理和清理；不启动 ROS/Gazebo。"""
import json
import os
from pathlib import Path
import shutil
import signal
import time
import subprocess
import tempfile
import unittest

SCRIPT_DIR = Path(__file__).resolve().parents[1]
FAKE = r'''#!/usr/bin/python3
import json, os, signal, sys, time
from pathlib import Path
name = Path(sys.argv[0]).name
args = sys.argv[1:]
trace = os.environ['FAKE_TRACE']
def record(event):
    with open(trace, 'a') as stream:
        stream.write(json.dumps({'name':name,'args':args,'event':event,'pid':os.getpid()})+'\n')
record('start')
if name == 'python-check':
    stage = args[1]
    if stage == os.environ.get('FAIL_STAGE'):
        sys.exit(1)
    sys.exit(0)
if name == 'roslaunch':
    def stop(signum, frame):
        record('stop')
        sys.exit(0)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    if os.environ.get('EXIT_WORLD') == '1' and 'gazebo_world' in args:
        time.sleep(.5)
        sys.exit(3)
    while True: time.sleep(.05)
if name == 'rosrun':
    if os.environ.get('FAIL_MARKER') == '1': sys.exit(1)
    sys.exit(0)
if name == 'rostopic': sys.exit(0)
sys.exit(2)
'''


class DockingScriptTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.launch = self.root / 'src/gazebo_launch'
        self.launch.mkdir(parents=True)
        (self.root / 'devel').mkdir()
        (self.root / 'devel/setup.bash').write_text('# fake setup\n')
        shutil.copy(SCRIPT_DIR / 'aruco_docking.sh', self.launch)
        for name in ('wait_single_water.py', 'wait_aruco_docking.py', 'cleanup_aruco_docking.py'):
            (self.launch / name).touch()
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        fake = self.bin / 'fake'
        fake.write_text(FAKE)
        fake.chmod(0o755)
        for name in ('roslaunch', 'rosrun', 'rostopic', 'python-check'):
            (self.bin / name).symlink_to(fake)
        self.env = dict(os.environ, PATH=str(self.bin) + ':' + os.environ['PATH'],
                        FAKE_TRACE=str(self.root / 'trace'), PYTHON_BIN=str(self.bin / 'python-check'),
                        GUI='false', IMAGE_VIEW='false', STARTUP_TIMEOUT='2')

    def run_script(self, commands='', **overrides):
        result = subprocess.run(['bash', str(self.launch / 'aruco_docking.sh')],
                                input=commands, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                env=dict(self.env, **overrides), timeout=20)
        trace = self.root / 'trace'
        events = [json.loads(line) for line in trace.read_text().splitlines()] if trace.exists() else []
        # 每条启动记录的常驻进程必须收到退出信号，不能遗留后台仿真。
        launched = {e['pid'] for e in events if e['name'] == 'roslaunch' and e['event'] == 'start'}
        stopped = {e['pid'] for e in events if e['name'] == 'roslaunch' and e['event'] == 'stop'}
        if not overrides.get('EXIT_WORLD'):
            self.assertEqual(launched, stopped, result.stdout)
        return result, events

    def test_launch_order_commands_and_cleanup(self):
        result, events = self.run_script('   \n dock \nst\nquit\n')
        self.assertEqual(result.returncode, 0, result.stdout)
        launches = [e['args'] for e in events if e['name'] == 'roslaunch' and e['event'] == 'start']
        self.assertEqual([args[1] for args in launches],
                         ['gazebo_world', 'new_gazebo_model', 'gazebo_control', 'asrobot', 'asrobot'])
        self.assertIn('static_model:=false', launches[1])
        self.assertIn('enable_bottom_camera:=true', launches[1])
        self.assertIn('enable_mancontrol:=false', launches[3])
        self.assertIn('enable_aruco:=false', launches[3])
        self.assertIn('aruco_length_outer:=0.2', launches[4])
        publications = [e['args'] for e in events if e['name'] == 'rostopic']
        self.assertEqual([args[-1] for args in publications], ['dock', 'st'])
        self.assertTrue(all('/robot1/cmd_message' in args for args in publications))
        self.assertTrue(list((self.root / 'logs').glob('aruco_docking_*/startup.txt')))
        self.assertNotIn('gnome-terminal', str(launches))

    def test_model_failure_stops_later_launches(self):
        result, events = self.run_script('quit\n', FAIL_STAGE='model')
        self.assertNotEqual(result.returncode, 0)
        launches = [e for e in events if e['name'] == 'roslaunch' and e['event'] == 'start']
        self.assertLessEqual(len(launches), 2)

    def test_no_detection_rejects_dock_but_allows_stop(self):
        result, events = self.run_script('dock\nst\nquit\n', FAIL_STAGE='detection')
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn('本次未发送 dock', result.stdout)
        self.assertEqual([e['args'][-1] for e in events if e['name'] == 'rostopic'], ['st'])

    def test_existing_simulation_prevents_any_launch(self):
        result, events = self.run_script('quit\n', FAIL_STAGE='preflight')
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(any(e['name'] == 'roslaunch' for e in events))

    def test_marker_failure_cleans_up(self):
        result, events = self.run_script('quit\n', FAIL_MARKER='1')
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(any('asrobot.launch' in e['args'] for e in events))

    def test_stdin_eof_cleans_up_without_auto_dock(self):
        result, events = self.run_script()
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertFalse(any(e['name'] == 'rostopic' for e in events))

    def test_waiting_pauses_and_dock_resumes(self):
        result, events = self.run_script('dock\nst\nquit\n')
        self.assertEqual(result.returncode, 0, result.stdout)
        stages = [e['args'][1] for e in events if e['name'] == 'python-check']
        self.assertIn('prepare', stages)
        self.assertLess(stages.index('prepare'), stages.index('resume'))
        self.assertIn('pause', stages[stages.index('resume') + 1:])
        publications = [e for e in events if e['name'] == 'rostopic']
        self.assertEqual([e['args'][-1] for e in publications], ['dock', 'st'])

    def test_reset_stops_and_prepares_again(self):
        result, events = self.run_script('reset\nquit\n')
        self.assertEqual(result.returncode, 0, result.stdout)
        stages = [e['args'][1] for e in events if e['name'] == 'python-check']
        self.assertEqual(stages.count('prepare'), 2)
        self.assertEqual([e['args'][-1] for e in events if e['name'] == 'rostopic'], ['st'])

    def test_waiting_can_be_left_running_explicitly(self):
        result, events = self.run_script('quit\n', WAIT_PAUSED='false')
        self.assertEqual(result.returncode, 0, result.stdout)
        stages = [e['args'][1] for e in events if e['name'] == 'python-check']
        self.assertNotIn('prepare', stages)
        self.assertNotIn('pause', stages)
        self.assertIn('detection', stages)


    def test_signal_exit_cleans_up(self):
        process = subprocess.Popen(['bash', str(self.launch / 'aruco_docking.sh')],
                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   text=True, env=self.env)
        try:
            deadline = time.monotonic() + 5
            trace = self.root / 'trace'
            while time.monotonic() < deadline:
                if trace.exists() and '"prepare"' in trace.read_text():
                    break
                time.sleep(.05)
            else:
                self.fail('script did not reach preparation stage')
            process.send_signal(signal.SIGINT)
            output, _ = process.communicate(timeout=15)
            self.assertEqual(process.returncode, 130, output)
            events = [json.loads(line) for line in trace.read_text().splitlines()]
            launched = {e['pid'] for e in events if e['name'] == 'roslaunch' and e['event'] == 'start'}
            stopped = {e['pid'] for e in events if e['name'] == 'roslaunch' and e['event'] == 'stop'}
            self.assertEqual(launched, stopped, output)
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate()



if __name__ == '__main__':
    unittest.main()
