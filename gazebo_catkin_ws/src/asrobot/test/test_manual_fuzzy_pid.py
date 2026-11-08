#!/usr/bin/env python3
"""隔离 ROS master 上验证手动 PID 路由；先 source devel/setup.bash。

只运行 motion，发布合成反馈；不启动 Gazebo，也不连接现有仿真。
"""
import os
import re
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import xmlrpc.client


def main():
    workspace = Path(__file__).resolve().parents[3]
    executable = workspace / 'devel/lib/asrobot/motion'
    with socket.socket() as candidate:
        candidate.bind(('127.0.0.1', 0))
        port = candidate.getsockname()[1]
    with tempfile.TemporaryDirectory(prefix='manual_pid_test_') as temp:
        os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
        os.environ['ROS_HOME'] = temp
        os.environ['ROS_LOG_DIR'] = temp
        os.environ['ROS_HOSTNAME'] = '127.0.0.1'
        os.environ.pop('ROS_IP', None)
        processes = []
        with open(Path(temp) / 'master.log', 'w') as master_log, \
                open(Path(temp) / 'motion.log', 'w+') as motion_log:
            try:
                processes.append(subprocess.Popen(['roscore', '-p', str(port)],
                    stdout=master_log, stderr=subprocess.STDOUT, start_new_session=True))
                master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
                deadline = time.monotonic() + 15
                while True:
                    try:
                        master.getPid('/manual_pid_test')
                        break
                    except OSError:
                        if time.monotonic() >= deadline:
                            raise RuntimeError('Test ROS master did not start')
                        time.sleep(.05)
                import rospy
                from asrobot.msg import mc, pwm
                from std_msgs.msg import String
                rospy.init_node('manual_pid_test', disable_signals=True)
                commands = rospy.Publisher('/manual_pid_test/cmd_message', String, queue_size=10)
                feedback = rospy.Publisher('/manual_pid_test/mc_message', mc, queue_size=10)
                published_pwm = []
                pwm_subscriber = rospy.Subscriber('/manual_pid_test/pwm_message', pwm,
                    lambda message: published_pwm.append(message), queue_size=100)
                processes.append(subprocess.Popen([str(executable), '__ns:=/manual_pid_test'],
                    stdout=motion_log, stderr=subprocess.STDOUT, start_new_session=True))
                deadline = time.monotonic() + 10
                while commands.get_num_connections() == 0 or feedback.get_num_connections() == 0:
                    if time.monotonic() >= deadline:
                        raise RuntimeError('motion subscribers did not connect')
                    time.sleep(.05)
                state = mc()
                state.c_x, state.c_y, state.c_z = .2, .1, -.5
                state.c_navyaw, state.c_speedz = 10., 0.
                cases = [
                    ('yaw 30', ['axis=Yaw_deg']),
                    ('yaw -30', ['axis=Yaw_deg']),
                    ('dep -1', ['axis=Z_m']),
                    ('depyaw -1 30', ['axis=Yaw_deg', 'axis=Z_m']),
                    ('xyz 1 1 -1', ['[XYZ PID {mode}]']),
                    ('vz', ['axis=Z_velocity_inner']),
                    ('pz -1', ['axis=Z_position_outer', 'axis=Z_velocity_inner']),
                ]
                for mode, switch in [('fuzzy', 'on'), ('classic', 'off')]:
                    commands.publish(String(data='fuzzypid ' + switch))
                    time.sleep(.12)
                    for command, markers in cases:
                        published_pwm.clear()
                        motion_log.flush()
                        start = (Path(temp) / 'motion.log').stat().st_size
                        commands.publish(String(data=command))
                        time.sleep(.12)
                        for _ in range(6):
                            feedback.publish(state)
                            time.sleep(.08)
                        motion_log.seek(start)
                        output = motion_log.read()
                        if processes[-1].poll() is not None:
                            raise AssertionError('motion exited: ' + output)
                        for marker in markers:
                            assert marker.format(mode=mode) in output, (mode, command, marker, output[-3000:])
                        if command.startswith(('yaw', 'dep', 'vz', 'pz')):
                            assert '[MANUAL PID %s]' % mode in output, (mode, command, output[-3000:])
                        if command.startswith(('dep ', 'xyz ')):
                            assert 'limit=6' in output and 'separation_error=0.1' in output
                        if command.startswith(('yaw ', 'depyaw ')):
                            assert '[MANUAL YAW ALLOCATION]' in output
                            angles = {}
                            for message in published_pwm:
                                angles.update(zip(message.channel, message.duty))
                            assert [angles[i] for i in (0, 3, 6, 9)] == [40, -40, 40, -40]
                            if command.startswith('yaw '):
                                last = published_pwm[-1]
                                if float(command.split()[1]) > state.c_navyaw:
                                    assert last.rotor_speed1 > last.rotor_speed2
                                    assert last.rotor_speed3 > last.rotor_speed4
                                else:
                                    assert last.rotor_speed1 < last.rotor_speed2
                                    assert last.rotor_speed3 < last.rotor_speed4
                        print('PASS', mode, command)
                # 航向跨 ±180 度时走最短方向（-179 - 179 => +2 度）。
                commands.publish(String(data='yaw -179'))
                state.c_navyaw = 179.
                time.sleep(.12)
                start = (Path(temp) / 'motion.log').stat().st_size
                for _ in range(5):
                    feedback.publish(state)
                    time.sleep(.08)
                motion_log.seek(start)
                assert 'axis=Yaw_deg e=2 ' in motion_log.read()
                print('PASS yaw wraparound')
                # 重复 off 不应再重置积分。
                state.c_navyaw = 10.
                commands.publish(String(data='yaw 11'))
                time.sleep(.12)
                start = (Path(temp) / 'motion.log').stat().st_size
                for _ in range(4):
                    feedback.publish(state)
                    time.sleep(.08)
                motion_log.seek(start)
                before = re.findall(r'axis=Yaw_deg .*?I_term=([^ ]+)', motion_log.read())
                commands.publish(String(data='fuzzypid off'))
                time.sleep(.12)
                start = (Path(temp) / 'motion.log').stat().st_size
                feedback.publish(state)
                time.sleep(.15)
                motion_log.seek(start)
                after = re.findall(r'axis=Yaw_deg .*?I_term=([^ ]+)', motion_log.read())
                assert before and after and float(after[-1]) > float(before[-1])
                print('PASS repeated switch preserves integral')
            finally:
                for process in reversed(processes):
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGTERM)
                            process.wait(timeout=5)


if __name__ == '__main__':
    main()
