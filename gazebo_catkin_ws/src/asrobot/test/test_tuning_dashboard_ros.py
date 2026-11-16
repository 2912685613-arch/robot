#!/usr/bin/env python3
"""独立ROS Master+实际motion：HTTP应用参数获得控制器确认，遥测和停止/记录真实连通；无Gazebo。"""
from pathlib import Path
import os
import signal
import socket
import subprocess
import tempfile
import time
import unittest

ROOT=Path(__file__).resolve().parents[3]

class TuningROSTests(unittest.TestCase):
    def test_actual_motion_parameter_reload_and_telemetry(self):
        with socket.socket() as sock:
            sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
        with tempfile.TemporaryDirectory() as temp:
            env=dict(os.environ,ROS_MASTER_URI='http://127.0.0.1:'+str(port),ROS_LOG_DIR=temp)
            children=[];streams=[]
            def launch(args,name):
                f=open(Path(temp)/name,'w');streams.append(f)
                p=subprocess.Popen(args,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
                children.append(p);return p
            try:
                launch(['roscore','-p',str(port)],'core.log')
                deadline=time.monotonic()+8
                while True:
                    try:
                        with socket.create_connection(('127.0.0.1',port),timeout=.2):break
                    except OSError:
                        if time.monotonic()>deadline:raise
                        time.sleep(.1)
                launch([str(ROOT/'devel/lib/asrobot/motion'),'__ns:=/robot1','__name:=motion'],'motion.log')
                source=r'''
import sys,time,threading,json
from pathlib import Path
from urllib.request import urlopen,Request
sys.path.insert(0,'__SCRIPTS__')
from tuning_dashboard import Dashboard,Server
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty,EmptyResponse,Trigger
from asrobot.msg import mc
app=Dashboard(log_dir='__TEMP__')
server=Server(('127.0.0.1',0),app)
thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
base='http://127.0.0.1:'+str(server.server_port)
running=True
publisher=rospy.Publisher('/robot1/mc_message',mc,queue_size=1)
# 模拟决策订阅，运动节点本身仍是实际编译二进制。
commands=[]
subscription=rospy.Subscriber('/robot1/cmd_message',String,lambda m:commands.append(m.data),queue_size=10)
services=[rospy.Service('/gazebo/unpause_physics',Empty,lambda _:EmptyResponse()),
          rospy.Service('/gazebo/pause_physics',Empty,lambda _:EmptyResponse())]
def feed():
 while running:
  m=mc();m.aruco_valid=m.feedback_valid=True
  m.aruco_stamp=m.feedback_stamp=rospy.Time.now().to_sec()
  m.cam_e_z=-.15;m.body_yaw_deg=25
  publisher.publish(m);time.sleep(.05)
worker=threading.Thread(target=feed,daemon=True);worker.start()
def wait(test,timeout=5):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  if test():return
  time.sleep(.05)
 raise AssertionError('timeout')
def post(path,body):
 req=Request(base+path,data=json.dumps(body).encode(),headers={'Content-Type':'application/json'})
 with urlopen(req,timeout=8) as r:return json.load(r)
try:
 wait(lambda:app.latest is not None and app.latest['valid'] and app.command_pub.get_num_connections()>=2)
 config=dict(app.config,position_kp=.42,max_speed=.015)
 answer=post('/api/apply',{'config':config})
 assert answer['config']['position_kp']==.42
 wait(lambda:app.latest['revision']==2 and app.latest['config']['position_kp']==.42)
 assert app.latest['state']=='IDLE'
 assert 'st' in commands
 # 节点自身再次检查非法参数；拒绝后生效配置仍为原值。
 rospy.set_param('/robot1/motion/docking/target_z',-.1)
 response=rospy.ServiceProxy('/robot1/motion/reload_docking_config',Trigger)()
 assert not response.success
 time.sleep(.2)
 assert app.latest['config']['target_z']==config['target_z']
 rospy.set_param('/robot1/motion/docking',config)
 post('/api/command',{'command':'dock'})
 wait(lambda:app.latest['state']=='HOLD',timeout=5)
 assert app.latest['active'] and app.latest['yaw_target']==25
 wait(lambda:app.trial is not None and len(app.trial['rows'])>3,timeout=2)
 assert app.trial,[(t.get('end_reason'),t.get('revision')) for t in app.history]
 post('/api/command',{'command':'stop'})
 wait(lambda:app.latest['state']=='IDLE')
 assert app.history and list(app.log_dir.glob('trial_*.json'))
 assert app.latest['revision']==2
 print('实际motion+HTTP：生效应答/版本、非法参数拒绝、dock/HOLD、st和记录导出通过')
finally:
 running=False;worker.join(1);server.shutdown();server.server_close();thread.join(1);app.close()
'''
                source=source.replace('__SCRIPTS__',str(ROOT/'src/asrobot/scripts')).replace('__TEMP__',temp)
                result=subprocess.run(['/usr/bin/python3','-u','-c',source],env=env,stdout=subprocess.PIPE,
                                      stderr=subprocess.STDOUT,text=True,timeout=25)
                if result.returncode:
                    print((Path(temp)/'motion.log').read_text(errors='replace')[-3500:])
                self.assertEqual(result.returncode,0,result.stdout)
                print(result.stdout)
            finally:
                for child in children:
                    if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
                for child in children:
                    try:child.wait(timeout=3)
                    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
                for stream in streams:stream.close()

if __name__=='__main__':unittest.main()
