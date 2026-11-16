#!/usr/bin/env python3
"""调参台指标、校验、演示HTTP、持久化与跨站拒绝；不连接ROS/Gazebo。"""
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from urllib.request import Request, urlopen
from urllib.error import HTTPError

SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))
from tuning_dashboard_core import response_metrics, validate_config
from tuning_dashboard import Dashboard, Server


def samples(values, step=.1):
    return [dict(t=n*step,value=v,valid=True) for n,v in enumerate(values)]


class MetricsTests(unittest.TestCase):
    def test_rising_and_falling_are_direction_symmetric(self):
        values=[0,.1,.2,.4,.6,.8,.9,1.,1.1,1.05,1.]+[1.]*15
        up=response_metrics(samples(values),1,.02,dwell=.5,complete=True)
        down=response_metrics(samples([1-v for v in values]),0,.02,dwell=.5,complete=True)
        for key in ('overshoot','overshoot_pct','rise_time','settling_time','peak_time','rmse'):
            self.assertAlmostEqual(up[key],down[key],msg=key)
        self.assertAlmostEqual(up['overshoot_pct'],10.)
        self.assertAlmostEqual(up['rise_time'],.5)
        self.assertAlmostEqual(up['settling_time'],1.)
        self.assertFalse(up['provisional'])

    def test_not_reached_target_has_no_rise_or_settling(self):
        result=response_metrics(samples([0,.1,.2,.3]),1,.02)
        self.assertIsNone(result['rise_time'])
        self.assertIsNone(result['settling_time'])
        self.assertEqual(result['overshoot'],0.)

    def test_gap_does_not_fabricate_crossings_or_settling(self):
        records=[dict(t=0,value=0),dict(t=.1,value=.05),dict(t=2,value=.95)]
        result=response_metrics(records,1,.1,dwell=1.)
        self.assertIsNone(result['rise_time'])
        self.assertIsNone(result['settling_time'])
        self.assertIn('缺口',result['quality'])
        # 最后的无效样本必须撤销“尾段已稳定”，不能被过滤后误报稳定。
        records=samples([0]+[1.]*20)+[dict(t=2.2,value=0,valid=False)]
        self.assertIsNone(response_metrics(records,1,.02)['settling_time'])

    def test_duplicate_times_dont_count_towards_stability(self):
        result=response_metrics([dict(t=1,value=0)]+[dict(t=2,value=1)]*100,1,.02,dwell=.5)
        self.assertEqual(result['n'],2)
        self.assertIsNone(result['settling_time'])
        self.assertEqual(result['dropped'],99)

    def test_later_excursion_revokes_provisional_settling(self):
        records=samples([0]+[1.]*20)
        self.assertIsNotNone(response_metrics(records,1,.02)['settling_time'])
        records.append(dict(t=2.1,value=1.2))
        self.assertIsNone(response_metrics(records,1,.02)['settling_time'])

    def test_zero_step_nan_and_negative_targets(self):
        self.assertIsNone(response_metrics(samples([1]*20),1,.02)['overshoot_pct'])
        self.assertEqual(response_metrics([dict(t=0,value=math.nan)],1,.1)['n'],0)
        result=response_metrics(samples([-.5,-.6,-.8,-1.,-1.1]+[-1.]*20),-1.,.02,dwell=.5)
        self.assertAlmostEqual(result['overshoot_pct'],20.)


class DashboardTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.app=Dashboard(demo=True,log_dir=self.temp.name)
        self.app.closed.set();self.app.thread.join(1) # 数值/HTTP测试采用确定性样本
        self.server=Server(('127.0.0.1',0),self.app)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.base='http://127.0.0.1:'+str(self.server.server_port)

    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.thread.join(1)
        self.app.close();self.temp.cleanup()

    def sample(self,t,state='APPROACH',distance=.5):
        self.app.ingest(dict(t=t,source_t=t,feedback_t=t,valid=True,active=True,state=state,
            distance=distance,x=.01,y=0,yaw_error=1,config=dict(self.app.config),revision=self.app.revision))

    def post(self,path,body):
        request=Request(self.base+path,data=json.dumps(body).encode(),headers={'Content-Type':'application/json'})
        with urlopen(request,timeout=3) as response:return json.load(response)

    def test_config_rejects_bool_unknown_nan_and_invalid_combination(self):
        valid=self.app.config
        for key,value in [('max_speed',True),('target_z',-.12),('velocity_ki',math.nan),('reacquire_frames',2.2)]:
            with self.subTest(key=key):
                with self.assertRaises(ValueError):validate_config(dict(valid,**{key:value}))
        with self.assertRaises(ValueError):validate_config(dict(valid,unknown=1))
        with self.assertRaises(ValueError):validate_config(dict(valid,near_speed=.03,max_speed=.02))
        self.assertEqual(validate_config(valid),valid)

    def test_http_static_and_state(self):
        self.sample(10)
        with urlopen(self.base+'/') as response:
            self.assertIn('参数工作台',response.read().decode())
        with urlopen(self.base+'/api/state') as response:data=json.load(response)
        self.assertTrue(data['connected']);self.assertTrue(data['demo'])
        self.assertEqual(data['seq'],1)
        with self.assertRaises(HTTPError) as result:urlopen(self.base+'/../../etc/passwd')
        self.assertEqual(result.exception.code,404)

    def test_apply_and_trial_export_roundtrip(self):
        self.sample(10)
        trial=self.post('/api/trial/start',{})['id']
        for n in range(20):self.sample(10+n*.1,distance=.15)
        self.post('/api/trial/finish',{})
        self.assertTrue(list(self.app.log_dir.glob('trial_*.json')))
        with urlopen(self.base+'/api/export.csv?trial='+trial) as response:
            text=response.read().decode('utf-8-sig')
            self.assertIn('source_t',text);self.assertIn('APPROACH',text)
        with urlopen(self.base+'/api/trial?id='+trial) as response:
            self.assertEqual(len(json.load(response)['rows']),20)
        config=dict(self.app.config,position_kp=.4)
        self.assertEqual(self.post('/api/apply',{'config':config})['config']['position_kp'],.4)
        self.assertEqual(self.app.revision,2)
        self.assertFalse(self.app.demo_active)

    def test_cross_origin_and_wrong_content_type_rejected(self):
        req=Request(self.base+'/api/command',data=b'{"command":"dock"}',
                    headers={'Content-Type':'application/json','Origin':'http://untrusted.example'})
        with self.assertRaises(HTTPError) as result:urlopen(req)
        self.assertEqual(result.exception.code,403)
        req=Request(self.base+'/api/command',data=b'command=dock')
        with self.assertRaises(HTTPError) as result:urlopen(req)
        self.assertEqual(result.exception.code,415)

    def test_metrics_start_from_approach_and_end_on_clock_reset(self):
        self.sample(1);self.app.start_trial()
        self.sample(2,state='XY_ALIGN',distance=.8)
        self.assertEqual(self.app.metrics(self.app.trial)['distance']['n'],0)
        self.sample(3,distance=.7);self.sample(3.1,distance=.6)
        self.assertEqual(self.app.metrics(self.app.trial)['distance']['n'],2)
        self.sample(.1)
        self.assertIsNone(self.app.trial)
        self.assertIn('回退',self.app.history[0]['end_reason'])

if __name__=='__main__':unittest.main()
