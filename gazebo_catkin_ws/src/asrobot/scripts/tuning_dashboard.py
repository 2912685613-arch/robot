#!/usr/bin/env python3
"""本地ArUco参数/响应调试台。标准库HTTP+浏览器Canvas，无需Flask、rosbridge或CDN。"""
import argparse
from collections import deque
import copy
import csv
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from urllib.parse import urlsplit

import yaml
sys.path.insert(0, str(Path(__file__).resolve().parent))
from tuning_dashboard_core import PARAMETERS, schema, validate_config, response_metrics, series_for

PACKAGE = Path(__file__).resolve().parents[1]
if not (PACKAGE/'config/docking_control.yaml').exists():
    import rospkg
    PACKAGE = Path(rospkg.RosPack().get_path('asrobot'))
WORKSPACE = PACKAGE.parents[1]


class Dashboard:
    def __init__(self, demo=False, namespace='robot1', model='gazebo_robot1', log_dir=None):
        self.demo = demo
        self.namespace = namespace.strip('/')
        self.model = model
        self.motion = '/' + self.namespace + '/motion'
        self.lock = threading.RLock()
        self.operation_lock = threading.Lock()
        self.closed = threading.Event()
        self.samples = deque(maxlen=12000)
        self.events = deque(maxlen=80)
        self.received = 0.
        self.latest = None
        self.trial = None
        self.history = []
        self.sequence = 0
        self.defaults = validate_config(yaml.safe_load((PACKAGE/'config/docking_control.yaml').read_text())['docking'])
        self.config = dict(self.defaults)
        self.revision = 1
        self.demo_t = 0.
        self.demo_start = 0.
        self.demo_active = False
        self.demo_paused = False
        self.motors = None
        self.motor_received = 0.
        self.log_dir = Path(log_dir or WORKSPACE/'logs') / ('tuning_dashboard_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        self.log_dir.mkdir(parents=True, exist_ok=True)
        if not demo:
            import rospy
            from std_msgs.msg import String
            from asrobot.msg import pwm
            self.rospy = rospy
            rospy.init_node('asrobot_tuning_dashboard', anonymous=True, disable_signals=True)
            self.command_pub = rospy.Publisher('/'+self.namespace+'/cmd_message', String, queue_size=2)
            self.telemetry_sub = rospy.Subscriber(self.motion+'/docking_telemetry', String, self.ros_telemetry, queue_size=1)
            self.pwm_sub = rospy.Subscriber('/'+self.namespace+'/pwm_message', pwm, self.ros_pwm, queue_size=1)
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()
        self.event('调试台已启动' + (' · 演示数据，不连接机器人' if demo else ' · 等待 motion 遥测'))

    def event(self, message):
        with self.lock:
            self.events.append(dict(wall=datetime.now().strftime('%H:%M:%S'), message=message))

    def ros_pwm(self, msg):
        with self.lock:
            self.motors = [msg.rotor_speed1, msg.rotor_speed2, msg.rotor_speed3, msg.rotor_speed4]
            self.motor_received = time.monotonic()

    def ros_telemetry(self, msg):
        try:
            telemetry = json.loads(msg.data)
            if not isinstance(telemetry, dict) or not all(k in telemetry for k in ('t', 'config', 'valid', 'state')):
                return
            telemetry['config'] = validate_config(telemetry['config'])
            self.ingest(telemetry)
        except (ValueError, TypeError):
            self.event('收到无效遥测，已跳过')

    def ingest(self, telemetry):
        with self.lock:
            self.sequence += 1
            telemetry = copy.deepcopy(telemetry)
            now = time.monotonic()
            telemetry['motors'] = self.motors if now-self.motor_received < .7 else None
            row = dict(seq=self.sequence, wall=now, telemetry=telemetry)
            self.samples.append(row)
            if self.latest and self.latest['state'] != telemetry['state']:
                self.event('状态：{} → {}'.format(self.latest['state'], telemetry['state']))
            self.latest, self.received = telemetry, now
            self.config = dict(telemetry['config'])
            self.revision = telemetry.get('revision', self.revision)
            if self.trial is not None:
                if telemetry.get('revision') != self.trial['revision']:
                    self.finish_trial('参数已变化，结束本段记录')
                elif self.trial['rows'] and telemetry['t'] < self.trial['rows'][-1]['telemetry']['t']:
                    self.finish_trial('仿真时间回退，结束本段记录')
                else:
                    self.trial['rows'].append(row)
                    if len(self.trial['rows']) >= 18000:
                        self.finish_trial('已达到单次记录30分钟上限')

    def demo_sample(self):
        if not self.demo_paused:
            self.demo_t += .1
        tau = self.demo_t-self.demo_start
        damping = min(1.3, .48 + .18*self.config['velocity_ki'])
        natural = .45*math.sqrt(self.config['position_kp']/.3)
        if self.demo_active:
            if damping < 1:
                wd = natural*math.sqrt(1-damping*damping)
                ratio = math.exp(-damping*natural*tau)*(math.cos(wd*tau)+damping/math.sqrt(1-damping*damping)*math.sin(wd*tau))
            else:
                ratio = math.exp(-natural*tau)*(1+natural*tau)
            target = -self.config['target_z']
            distance = target + (.65-target)*ratio
            x = .10*math.exp(-tau*.3)
            y = -.035*math.exp(-tau*.35)
            yaw_error = -6*math.exp(-tau*.2)*math.cos(tau*.5)
            state = 'XY_ALIGN' if tau < 2 else ('HOLD' if tau > 25 else 'APPROACH')
        else:
            target = -self.config['target_z']; distance=.65;x=.1;y=-.035;yaw_error=0;state='IDLE'
        self.motors = [49+2*math.sin(tau+n) if self.demo_active else 0 for n in range(4)]
        self.motor_received = time.monotonic()
        self.ingest(dict(t=self.demo_t, source_t=self.demo_t, feedback_t=self.demo_t, age=.06,
                         valid=True, active=self.demo_active, state=state, reason='demo',
                         distance=distance, x=x,y=y,z=-distance,vx=0,vy=0,vz=-.02*math.exp(-max(tau,0)*.08) if self.demo_active else 0,
                         target_vz=-.02 if self.demo_active else 0,yaw=12-yaw_error,yaw_target=12,yaw_error=yaw_error,
                         fx=x*.6,fy=y*.6,fz=-.12 if self.demo_active else 0,mz=yaw_error*.003,
                         revision=self.revision,config=self.config))

    def loop(self):
        while not self.closed.wait(.1):
            if self.demo:
                with self.lock:
                    self.demo_sample()
            elif self.rospy.is_shutdown():
                break

    def require_connection(self):
        with self.lock:
            if self.latest is None or time.monotonic()-self.received > 1.:
                raise ValueError('motion 遥测未连接或已过期；请启动新编译的 motion 节点')

    def call_service(self, name, payload=None):
        """子进程限制服务总耗时，不让HTTP线程无限卡在ROS服务调用里。"""
        args = ['rosservice', 'call', name]
        if payload is not None:
            args.append(yaml.safe_dump(payload, default_flow_style=True))
        try:
            result = subprocess.run(args, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
        except subprocess.TimeoutExpired:
            raise ValueError('服务应答超时；状态未确认，请刷新遥测：'+name)
        if result.returncode:
            raise ValueError('服务调用失败：'+name+' '+result.stderr.strip()[-250:])
        response = yaml.safe_load(result.stdout) or {}
        if isinstance(response, dict) and response.get('success') is False:
            raise ValueError(response.get('message') or response.get('status_message') or '节点拒绝操作')
        return response

    def publish(self, command):
        if self.command_pub.get_num_connections() < (1 if command=='st' else 2):
            raise ValueError('命令订阅者未就绪，未发送命令')
        self.command_pub.publish(command)

    def apply(self, raw):
        config = validate_config(raw)
        self.require_connection()
        with self.operation_lock:
            if self.demo:
                with self.lock:
                    self.finish_trial('应用参数，结束试验')
                    self.config=config;self.revision+=1;self.demo_active=False
                self.event('演示参数已应用，需重新开始对接')
                return config
            old = self.rospy.get_param(self.motion+'/docking', self.config)
            self.publish('st')
            self.rospy.set_param(self.motion+'/docking', config)
            try:
                result = self.call_service(self.motion+'/reload_docking_config')
                actual = validate_config(json.loads(result['message']))
                if actual != config:
                    raise ValueError('节点返回的生效参数与请求不一致')
            except Exception:
                self.rospy.set_param(self.motion+'/docking',old)
                self.event('参数应用未确认，已恢复参数服务器旧值；请读取节点生效参数')
                raise
            with self.lock:
                self.finish_trial('应用参数，结束试验')
                self.config=dict(actual)
            self.event('参数应用成功，运动节点已停止并重置控制器')
            return actual

    def command(self, name):
        if name not in ('dock','stop','pause','resume','reset'):
            raise ValueError('不支持的操作')
        if name not in ('stop','pause'):
            self.require_connection()
        with self.operation_lock:
            if self.demo:
                with self.lock:
                    if name=='dock':
                        self.start_trial();self.demo_start=self.demo_t;self.demo_active=True;self.demo_paused=False
                    elif name=='reset':
                        self.finish_trial('重置');self.demo_active=False;self.demo_paused=True
                    elif name in ('stop','pause'):
                        self.finish_trial('停止');self.demo_active=False;self.demo_paused=True
                    else:
                        self.demo_paused=False
                self.event('演示操作：'+name)
                return
            if name in ('stop','pause'):
                # 暂停/clock前先给motion处理st的时间；motion墙钟回调即使暂停也可处理。
                self.publish('st');time.sleep(.15)
                self.call_service('/gazebo/pause_physics')
                self.finish_trial('人工停止并暂停')
            elif name=='resume':
                self.call_service('/gazebo/unpause_physics')
            elif name=='reset':
                self.publish('st');self.finish_trial('重置机器人')
                self.call_service('/gazebo/unpause_physics')
                try:
                    payload=dict(model_state=dict(model_name=self.model, reference_frame='world',
                        pose=dict(position=dict(x=0.,y=0.,z=-.1),orientation=dict(x=0.,y=0.,z=0.,w=1.)),
                        twist=dict(linear=dict(x=0.,y=0.,z=0.),angular=dict(x=0.,y=0.,z=0.))))
                    self.call_service('/gazebo/set_model_state',payload)
                    time.sleep(.25)
                finally:
                    self.call_service('/gazebo/pause_physics')
            else:
                before = self.latest.get('source_t',0)
                self.call_service('/gazebo/unpause_physics')
                deadline=time.monotonic()+3
                while time.monotonic()<deadline:
                    with self.lock:
                        if (self.latest and self.latest.get('valid') and self.latest.get('source_t',0)>before
                                and time.monotonic()-self.received<.5):
                            break
                    time.sleep(.05)
                else:
                    self.publish('st');self.call_service('/gazebo/pause_physics')
                    raise ValueError('恢复后未取得新鲜有效定位，未发送 dock，已暂停')
                self.start_trial()
                self.publish('dock')
            self.event('操作：'+name)

    def start_trial(self):
        with self.lock:
            if self.trial:
                self.finish_trial('开始新试验')
            self.trial=dict(id=datetime.now().strftime('%H%M%S_%f'),label='试验 '+datetime.now().strftime('%H:%M:%S'),
                            config=dict(self.config),revision=self.revision,rows=[],complete=False)
            self.event('开始记录 · 等待新的有效采样')
            return self.trial['id']

    def metrics(self, trial):
        rows=trial['rows']
        if not trial['complete'] and self.received and time.monotonic()-self.received>1.:
            rows=rows+[dict(telemetry=dict(t=rows[-1]['telemetry']['t'] if rows else 0,valid=False))]
        metrics={}
        for axis in ('distance','x','y','yaw'):
            selected=rows
            if axis=='distance':
                # 高度响应从开始接近时计算，不把XY等待时间假装为Z响应延迟。
                idx=next((i for i,r in enumerate(rows) if r['telemetry'].get('state') in ('APPROACH','HOLD')),None)
                selected=[] if idx is None else rows[idx:]
            config=trial['config']
            target=-config['target_z'] if axis=='distance' else 0.
            tol=config['z_tolerance'] if axis=='distance' else (config['yaw_tolerance'] if axis=='yaw' else config['xy_tolerance'])
            metrics[axis]=response_metrics(series_for(selected,axis),target,tol,config['settle_time'],complete=trial['complete'])
        return metrics

    def finish_trial(self, reason='结束记录'):
        with self.lock:
            if not self.trial:
                return None
            trial=self.trial;self.trial=None;trial['complete']=True;trial['end_reason']=reason
            trial['metrics']=self.metrics(trial)
            self.history.insert(0,trial)
            self.history=self.history[:12]
            path=self.log_dir/('trial_'+trial['id']+'.json')
            path.write_text(json.dumps(trial,ensure_ascii=False,allow_nan=False,indent=2))
            self.event(reason+' · 已保存 '+path.name)
            return trial

    def snapshot(self, after=0):
        with self.lock:
            summary=lambda t:dict(id=t['id'],label=t['label'],config=t['config'],metrics=t.get('metrics') or self.metrics(t),complete=t['complete'],count=len(t['rows']),revision=t['revision'],start_t=t['rows'][0]['telemetry']['t'] if t['rows'] else None,end_t=t['rows'][-1]['telemetry']['t'] if t['rows'] else None)
            connected=self.latest is not None and time.monotonic()-self.received<1.
            # 增量样本上限600条；浏览器首次加载只取近期600条，导出保留完整试验。
            return dict(demo=self.demo,connected=connected,age=None if not self.received else time.monotonic()-self.received,
                        latest=self.latest,config=self.config,revision=self.revision,defaults=self.defaults,
                        schema=schema(),seq=self.sequence,rows=[r for r in self.samples if r['seq']>after][-600:],
                        trial=summary(self.trial) if self.trial else None,
                        history=[summary(t) for t in self.history],events=list(self.events),log_dir=str(self.log_dir))

    def csv_bytes(self, trial_id=None):
        with self.lock:
            trials=([self.trial] if self.trial else [])+self.history
            trial=next((t for t in trials if t['id']==trial_id),None) if trial_id else (trials[0] if trials else None)
            rows=copy.deepcopy(trial['rows'] if trial else list(self.samples))
        stream=io.StringIO();fields=['seq','t','source_t','feedback_t','state','valid','active','distance','x','y','z','yaw','yaw_target','yaw_error','vz','target_vz','fx','fy','fz','mz','revision']
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for r in rows:
            flat=dict(r['telemetry'],seq=r['seq']);writer.writerow({k:flat.get(k,'') for k in fields})
        return ('\ufeff'+stream.getvalue()).encode('utf-8')

    def close(self):
        self.closed.set();self.thread.join(timeout=1)
        self.finish_trial('调试台关闭')
        if not self.demo:
            self.rospy.signal_shutdown('dashboard closed')


class Server(ThreadingHTTPServer):
    daemon_threads=True
    def __init__(self,address,app):
        self.app=app
        super().__init__(address,Handler)


class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args):
        pass

    def send(self,data,status=200,content_type='application/json; charset=utf-8'):
        if not isinstance(data,bytes):
            data=json.dumps(data,ensure_ascii=False,allow_nan=False).encode()
        self.send_response(status);self.send_header('Content-Type',content_type)
        self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store')
        self.send_header('X-Content-Type-Options','nosniff');self.send_header('X-Frame-Options','DENY')
        self.end_headers()
        try:self.wfile.write(data)
        except (BrokenPipeError,ConnectionResetError):pass

    def do_GET(self):
        from urllib.parse import parse_qs
        url=urlsplit(self.path);query=parse_qs(url.query)
        if url.path=='/api/state':
            try:after=max(0,int(query.get('after',['0'])[0]))
            except ValueError:after=0
            return self.send(self.server.app.snapshot(after))
        if url.path=='/api/trial':
            with self.server.app.lock:
                trial=next((t for t in self.server.app.history if t['id']==query.get('id',[None])[0]),None)
                if trial is None:
                    return self.send({'error':'试验不存在'},404)
                return self.send(dict(id=trial['id'],rows=trial['rows']))
        if url.path=='/api/export.csv':
            return self.send(self.server.app.csv_bytes(query.get('trial',[None])[0]),content_type='text/csv; charset=utf-8')
        if url.path=='/api/config.yaml':
            with self.server.app.lock:config=dict(self.server.app.config)
            return self.send(yaml.safe_dump({'docking':config},sort_keys=False).encode(),content_type='application/yaml; charset=utf-8')
        assets={'/':('index.html','text/html; charset=utf-8'),'/app.js':('app.js','application/javascript; charset=utf-8'),
                '/style.css':('style.css','text/css; charset=utf-8')}
        if url.path in assets:
            name,mime=assets[url.path]
            return self.send((PACKAGE/'web/tuning'/name).read_bytes(),content_type=mime)
        self.send({'error':'not found'},404)

    def do_POST(self):
        # 只接受本地界面发起的JSON操作，不允许别的网站用表单触发机器人命令。
        origin=self.headers.get('Origin')
        if origin and urlsplit(origin).netloc != self.headers.get('Host'):
            return self.send({'error':'拒绝跨站操作'},403)
        if self.headers.get_content_type()!='application/json':
            return self.send({'error':'需要 JSON 请求'},415)
        try:
            size=int(self.headers.get('Content-Length','0'))
            if size<0 or size>16384:raise ValueError('请求过大')
            body=json.loads(self.rfile.read(size) or b'{}')
            if not isinstance(body,dict):raise ValueError('请求必须是对象')
            app=self.server.app;path=urlsplit(self.path).path
            if path=='/api/apply':result={'config':app.apply(body.get('config'))}
            elif path=='/api/command':app.command(body.get('command'));result={'ok':True}
            elif path=='/api/trial/start':result={'id':app.start_trial()}
            elif path=='/api/trial/finish':app.finish_trial();result={'ok':True}
            else:return self.send({'error':'not found'},404)
            self.send(result)
        except (ValueError,TypeError,KeyError) as exc:
            self.send({'error':str(exc)},400)
        except Exception as exc:
            self.server.app.event('操作失败：'+str(exc))
            self.send({'error':'操作失败：'+str(exc)},503)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo',action='store_true',help='演示数据，完全不连接ROS或机器人')
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--robot',default='robot1');parser.add_argument('--model',default='gazebo_robot1')
    parser.add_argument('--log-dir')
    args, _=parser.parse_known_args()
    app=Dashboard(args.demo,args.robot,args.model,args.log_dir)
    server=Server(('127.0.0.1',args.port),app)
    def stop(*_):
        threading.Thread(target=server.shutdown,daemon=True).start()
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    print('ArUco 调参台：http://127.0.0.1:'+str(server.server_port),flush=True)
    print('模式：'+('演示（无机器人）' if args.demo else 'ROS实时')+'；日志：'+str(app.log_dir),flush=True)
    try:server.serve_forever(poll_interval=.2)
    finally:server.server_close();app.close()

if __name__=='__main__':main()
