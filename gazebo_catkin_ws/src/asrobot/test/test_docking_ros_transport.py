#!/usr/bin/env python3
"""独立本机ROS Master + 实际decision/motion节点的消息回归；不启动Gazebo/控制器插件。"""
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import unittest

class RosTransportTests(unittest.TestCase):
    def test_first_dock_validity_hold_lost_and_watchdog(self):
        import rosgraph
        root=Path(__file__).resolve().parents[3]
        with socket.socket() as sock:
            sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
        with tempfile.TemporaryDirectory() as temp:
            env=dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:'+str(port),ROS_LOG_DIR=temp)
            # 独立脚本进程，避免rospy单次初始化状态污染其他测试。
            source=r'''
import rospy,time,threading
from pathlib import Path
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose,Twist
from std_msgs.msg import String
from rosgraph_msgs.msg import Log
from asrobot.msg import camera_data,mc,pwm
rospy.init_node('transport_test',anonymous=True,disable_signals=True)
models=rospy.Publisher('/gazebo/model_states',ModelStates,queue_size=1)
vision=rospy.Publisher('/camera_aruco_data',camera_data,queue_size=1)
cmd=rospy.Publisher('/robot1/cmd_message',String,queue_size=1)
seen=[];act=[];messages=[]
rospy.Subscriber('/rosout',Log,lambda m:messages.append(m.msg) if m.name=='/robot1/motion' else None,queue_size=1000)
rospy.Subscriber('/robot1/mc_message',mc,lambda msg: seen.append((time.monotonic(),msg)),queue_size=100)
rospy.Subscriber('/robot1/pwm_message',pwm,lambda msg:act.append((time.monotonic(),msg)),queue_size=100)
mode='invalid';running=True
pose=Pose();pose.orientation.w=1
body=Twist()
def feed():
 while running:
  models.publish(ModelStates(name=['gazebo_robot1'],pose=[pose],twist=[body]))
  m=camera_data()
  if mode!='invalid':
   m.aruco_valid=True;m.detection_stamp=rospy.Time.now().to_sec();m.position_z=-.15
  vision.publish(m)
  time.sleep(.05)
thread=threading.Thread(target=feed,daemon=True);thread.start()
def wait_for(test,timeout=5):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  if test():return
  time.sleep(.03)
 raise AssertionError('condition timeout')
wait_for(lambda:cmd.get_num_connections()>=2 and len(seen)>2)
assert not seen[-1][1].aruco_valid
cmd.publish('dock')
time.sleep(1.3)
log=Path('__MOTION_LOG__')
assert 'WAIT_VISION' in '\n'.join(messages)
assert 'DOCKING STOP' not in '\n'.join(messages)
assert all(max(m.rotor_speed1,m.rotor_speed2,m.rotor_speed3,m.rotor_speed4)==0 for _,m in act[-4:])
mode='valid'
wait_for(lambda:any(m.aruco_valid and m.feedback_valid and m.aruco_stamp>0 for _,m in seen[-5:]))
wait_for(lambda:'-> HOLD' in '\n'.join(messages),timeout=4)
assert any(m.rotor_speed1>0 for _,m in act[-20:])
# 对接时每轮一条，源图时间保留而不是用mc发送时间伪造；没有零值误停。
times=[t for t,m in seen[-6:]]
assert all(b-a>.10 for a,b in zip(times,times[1:])),times
mode='invalid'
wait_for(lambda:'LOST_STOP' in '\n'.join(messages))
time.sleep(.3)
assert max(act[-1][1].rotor_speed1,act[-1][1].rotor_speed2)==0
mode='valid';time.sleep(.5)
assert max(act[-1][1].rotor_speed1,act[-1][1].rotor_speed2)==0 # 丢失锁存，重获不自动冲刺
cmd.publish('dock');time.sleep(1.5)
import rosnode
rosnode.kill_nodes(['/decision_robot1'])
wait_for(lambda:sum('-> LOST_STOP' in m for m in messages)>=2)
time.sleep(.2)
assert max(act[-1][1].rotor_speed1,act[-1][1].rotor_speed2)==0 # 无mc时墙钟看门狗仍能关机
# 人工st清除决策跟踪标志并关电机
cmd.publish('st');time.sleep(.4)
assert max(act[-1][1].rotor_speed1,act[-1][1].rotor_speed2)==0
running=False;thread.join(1)
# 另启实际motion节点，使用冻结的/clock=10，确认墙钟看门狗不会卡在ROS Rate sleep。
import os,signal,subprocess
from rosgraph_msgs.msg import Clock
rospy.set_param('/use_sim_time',True)
clock=rospy.Publisher('/clock',Clock,queue_size=1,latch=True)
clock.publish(Clock(clock=rospy.Time.from_sec(10)))
clock_mc=rospy.Publisher('/clock_robot/mc_message',mc,queue_size=1)
clock_cmd=rospy.Publisher('/clock_robot/cmd_message',String,queue_size=1)
clock_act=[]
rospy.Subscriber('/clock_robot/pwm_message',pwm,lambda m:clock_act.append(m),queue_size=100)
proc=subprocess.Popen(['__MOTION_BINARY__','__ns:=/clock_robot','__name:=motion'],
                      stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT,start_new_session=True)
clock_running=True
try:
 def clock_feed():
  while clock_running:
   message=mc();message.aruco_valid=message.feedback_valid=True
   message.aruco_stamp=message.feedback_stamp=10.;message.cam_e_z=-.15
   clock_mc.publish(message);time.sleep(.05)
 clock_thread=threading.Thread(target=clock_feed,daemon=True);clock_thread.start()
 wait_for(lambda:clock_cmd.get_num_connections()>0 and len(clock_act)>2)
 clock_cmd.publish('dock')
 wait_for(lambda:any(m.rotor_speed1>0 for m in clock_act[-10:]),timeout=4)
 clock_running=False;clock_thread.join(1)
 time.sleep(.85)
 assert clock_act[-1].rotor_speed1==0, 'paused /clock blocked watchdog'
finally:
 clock_running=False
 if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
 try:proc.wait(timeout=3)
 except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
print('ROS transport: first dock waits; valid -> HOLD; loss/st/mc silence and paused-clock watchdog -> stop')
'''
            processes=[];streams=[]
            try:
                def start(argv,name):
                    stream=open(Path(temp)/name,'w');streams.append(stream)
                    proc=subprocess.Popen(argv,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
                    processes.append(proc);return proc
                start(['roscore','-p',str(port)],'master.log')
                end=time.monotonic()+8
                while True:
                    try:
                        with socket.create_connection(('127.0.0.1',port),timeout=.2):
                            break
                    except Exception:
                        if time.monotonic()>end:raise
                        time.sleep(.1)
                start([str(root/'devel/lib/asrobot/decision'),'__name:=decision_robot1',
                       'mc_message:=/robot1/mc_message','cmd_message:=/robot1/cmd_message'],'decision.log')
                start([str(root/'devel/lib/asrobot/motion'),'__ns:=/robot1','__name:=motion'],'motion.log')
                source=source.replace('__MOTION_LOG__',str(Path(temp)/'motion.log')).replace(
                    '__MOTION_BINARY__',str(root/'devel/lib/asrobot/motion'))
                result=subprocess.run(['/usr/bin/python3','-u','-c',source],env=env,text=True,
                                      stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=30)
                if result.returncode:
                    print((Path(temp)/'motion.log').read_text()[-5000:])
                    print((Path(temp)/'decision.log').read_text()[-2000:])
                self.assertEqual(result.returncode,0,result.stdout)
                print(result.stdout)
            finally:
                for proc in processes:
                    if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
                for proc in processes:
                    try:proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                for stream in streams:stream.close()

if __name__=='__main__':unittest.main()
