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
wait_for(lambda:'REACQUIRE_BRAKE' in '\n'.join(messages))
time.sleep(.15)
assert 'LOST_STOP' not in '\n'.join(messages)
assert any(m.rotor_speed1>0 for _,m in act[-5:]) # 有机体反馈时短时制动/保持航向
mode='valid'
wait_for(lambda:sum('-> HOLD' in m for m in messages)>=2,timeout=5)
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
clock_act=[];clock_messages=[]
rospy.Subscriber('/rosout',Log,lambda m:clock_messages.append(m.msg) if m.name=='/clock_robot/motion' else None,queue_size=1000)
rospy.Subscriber('/clock_robot/pwm_message',pwm,lambda m:clock_act.append(m),queue_size=100)
proc=subprocess.Popen(['__MOTION_BINARY__','__ns:=/clock_robot','__name:=motion'],
                      stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT,start_new_session=True)
clock_running=True;clock_advancing=False;clock_value=10.;clock_only_running=False
try:
 def clock_feed():
  global clock_value
  while clock_running:
   if clock_advancing:
    clock_value+=.05
    clock.publish(Clock(clock=rospy.Time.from_sec(clock_value)))
    time.sleep(.02) # 让独立节点先接收/clock，避免测试消息人为来自未来
   message=mc();message.aruco_valid=message.feedback_valid=True
   message.aruco_stamp=message.feedback_stamp=clock_value-(.01 if clock_advancing else 0);message.cam_e_z=-.15
   clock_mc.publish(message);time.sleep(.05)
 clock_thread=threading.Thread(target=clock_feed,daemon=True);clock_thread.start()
 wait_for(lambda:clock_cmd.get_num_connections()>0 and len(clock_act)>2)
 clock_cmd.publish('dock')
 wait_for(lambda:any(m.rotor_speed1>0 for m in clock_act[-10:]),timeout=4)
 clock_running=False;clock_thread.join(1)
 time.sleep(2.7)
 assert clock_act[-1].rotor_speed1==0, 'paused /clock blocked watchdog'
 assert any('WAIT_TRANSPORT' in m for m in clock_messages)
 assert not any('LOST_STOP' in m for m in clock_messages), clock_messages
 # 恢复/clock和新MC；与用户日志中的两秒卡顿相同，确认后应自动继续而非锁死。
 clock_running=True;clock_advancing=True
 clock_thread=threading.Thread(target=clock_feed,daemon=True);clock_thread.start()
 try:
  wait_for(lambda:any('WAIT_TRANSPORT ->' in m and 'LOST_STOP' not in m for m in clock_messages))
  wait_for(lambda:any(m.rotor_speed1>0 for m in clock_act[-5:]))
 except AssertionError:
  print('clock recovery diagnostics:',[m for m in clock_messages if 'DOCK STATE' in m][-10:])
  raise
 # /clock继续走，但MC发送端真的死亡：仍必须在独立时限后锁存LOST_STOP。
 clock_running=False;clock_thread.join(1);clock_only_running=True
 def clock_only():
  global clock_value
  while clock_only_running:
   clock_value+=.05;clock.publish(Clock(clock=rospy.Time.from_sec(clock_value)));time.sleep(.05)
 clock_only_thread=threading.Thread(target=clock_only,daemon=True);clock_only_thread.start()
 wait_for(lambda:any('LOST_STOP' in m and 'mc_transport_timeout' in m for m in clock_messages))
 assert clock_act[-1].rotor_speed1==0
finally:
 clock_running=False;clock_only_running=False
 if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
 try:proc.wait(timeout=3)
 except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()

# 实际aruco_det节点：合成图像验证远处保留大码、近处确认小码、丢小码立即回退。
import cv2,numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import PoseStamped
rospy.set_param('/use_sim_time',False)
image_pub=rospy.Publisher('/target_test/image',Image,queue_size=1)
info_pub=rospy.Publisher('/target_test/info',CameraInfo,queue_size=1,latch=True)
poses=[];target_logs=[]
rospy.Subscriber('/aruco/pose',PoseStamped,lambda m:poses.append(m),queue_size=100)
rospy.Subscriber('/rosout',Log,lambda m:target_logs.append(m.msg) if m.name=='/target_test_detector' else None,queue_size=100)
info=CameraInfo();info.width=640;info.height=480
info.header.frame_id='gazebo_robot1/camera_bottom_optical_frame'
info.K=[320.,0.,320.5,0.,320.,240.5,0.,0.,1.];info.D=[0.]*5;info.distortion_model='plumb_bob'
info_pub.publish(info)
bridge=CvBridge()
texture=cv2.imread('__TEXTURE__',cv2.IMREAD_GRAYSCALE)
dictionary=cv2.aruco.getPredefinedDictionary(10)
outer_only=cv2.copyMakeBorder(cv2.aruco.drawMarker(dictionary,5,800),160,160,160,160,cv2.BORDER_CONSTANT,value=255)
detector=subprocess.Popen(['__ARUCO_BINARY__','__name:=target_test_detector',
    '_sub_image_topic:=/target_test/image','_camera_info_topic:=/target_test/info','_aruco_length_outer:=0.2'],
    stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT,start_new_session=True)
try:
 wait_for(lambda:image_pub.get_num_connections()>0)
 def feed_scene(texture,depth,duration=1.3):
  scale=320.*.28/depth/1120
  transform=np.array([[scale,0,320.5-560*scale],[0,scale,240.5-560*scale]])
  image=cv2.warpAffine(texture,transform,(640,480),flags=cv2.INTER_LINEAR,borderValue=180)
  end=time.monotonic()+duration
  while time.monotonic()<end:
   m=bridge.cv2_to_imgmsg(image,encoding='mono8');m.header.stamp=rospy.Time.now();m.header.frame_id=info.header.frame_id
   image_pub.publish(m);time.sleep(.04)
 feed_scene(texture,.5)
 assert poses and abs(poses[-1].pose.position.z-.5)<.03
 assert not any('5 -> 18' in m for m in target_logs),target_logs
 feed_scene(texture,.20)
 wait_for(lambda:any('5 -> 18' in m for m in target_logs))
 before=len(poses)
 feed_scene(outer_only,.20)
 wait_for(lambda:any('18 -> 5' in m for m in target_logs))
 assert len(poses)>before and abs(poses[-1].pose.position.z-.20)<.02
 feed_scene(texture,.20)
 feed_scene(texture,.03)
 assert abs(poses[-1].pose.position.z-.03)<.003
finally:
 if detector.poll() is None:os.killpg(detector.pid,signal.SIGINT)
 try:detector.wait(timeout=3)
 except subprocess.TimeoutExpired:os.killpg(detector.pid,signal.SIGKILL);detector.wait()
print('ROS transport: first dock/HOLD; transient vision recovery; long loss/watchdogs stop; actual detector dual-marker fallback passed')

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
                    '__MOTION_BINARY__',str(root/'devel/lib/asrobot/motion')).replace(
                    '__ARUCO_BINARY__',str(root/'devel/lib/asrobot/aruco_det')).replace(
                    '__TEXTURE__',str(root/'src/gazebo_world/models/aruco_demo_marker/materials/textures/marker5.png'))
                result=subprocess.run(['/usr/bin/python3','-u','-c',source],env=env,text=True,
                                      stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=50)
                if result.returncode:
                    print((Path(temp)/'motion.log').read_text(errors='replace')[-5000:])
                    print((Path(temp)/'decision.log').read_text(errors='replace')[-2000:])
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
