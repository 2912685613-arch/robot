#!/usr/bin/env python3
"""用真实CAD/校准后的关节轴、力臂独立检查对接Fx/Fy/Fz/Mz分配及UUV转换。"""
import importlib.util
import math
from pathlib import Path
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np
import yaml

SRC=Path(__file__).resolve().parents[2]
SPEC=importlib.util.spec_from_file_location('water_geometry',SRC/'gazebo_launch/tests/test_single_water.py')
GEOMETRY=importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GEOMETRY)

class DockingGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory()
        cls.binary=Path(cls.temp.name)/'allocation'
        subprocess.run(['g++','-std=c++11','-O1','-I'+str(SRC/'asrobot/include'),
                        str(SRC/'asrobot/test/docking_control_checks.cpp'),'-o',str(cls.binary)],check=True)
        model=ET.fromstring(subprocess.check_output(['xacro',str(SRC/'new_gazebo_model/urdf/robot/gazebo_model.xacro'),
                            'namespace:=gazebo_robot1','color:=Orange']))
        cls.joints={j.find('child').get('link'):j for j in model.findall('joint')}
        cls.calibration=yaml.safe_load((SRC/'new_gazebo_model/config/joint_command_calibration.yaml').read_text())

    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()

    def actual_wrench(self,tilts,speeds):
        angles={}
        for i,h in enumerate((40,-40,40,-40)):
            values=(h,-2 if i==0 else 0,-14+tilts[i])
            for k,joint_name in enumerate('HMV'):
                n=3*i+k
                angles[joint_name+str(i+1)]=math.radians(self.calibration['joint_command_scale'][n]*values[k]
                                                       +self.calibration['joint_command_offset_deg'][n])
        def pose(link):
            if link not in self.joints:return np.eye(4)
            joint=self.joints[link]
            r,p,y=map(float,joint.find('origin').get('rpy','0 0 0').split())
            axis=list(map(float,joint.find('axis').get('xyz').split()))
            t=np.eye(4)
            t[:3,:3]=(GEOMETRY.rotation([0,0,1],y)@GEOMETRY.rotation([0,1,0],p)
                      @GEOMETRY.rotation([1,0,0],r)@GEOMETRY.rotation(axis,angles.get(joint.get('name'),0)))
            t[:3,3]=list(map(float,joint.find('origin').get('xyz','0 0 0').split()))
            return pose(joint.find('parent').get('link'))@t
        force=np.zeros(3);moment=np.zeros(3)
        for i in range(4):
            link='gazebo_robot1/thruster_'+str(i)
            t=pose(link)
            axis=np.array(list(map(float,self.joints[link].find('axis').get('xyz').split())))
            f=t[:3,:3]@axis*(.00031*speeds[i]**2)
            force+=f;moment+=np.cross(t[:3,3],f)
        return np.r_[force,moment[2]]

    def test_cad_direction_and_yaw_moment(self):
        for request in [(0,0,0,0),(.1,0,0,0),(0,.1,0,0),(0,0,-.4,0),(0,0,.4,0),
                        (0,0,0,.03),(0,0,0,-.03),(.1,-.1,-.3,.02),(-.1,.1,.3,-.02)]:
            with self.subTest(request=request):
                raw=np.fromstring(subprocess.check_output([str(self.binary),'print-allocation',*map(str,request)]),sep=' ')
                speeds=raw[:8:2];tilts=raw[1:8:2]
                actual=self.actual_wrench(tilts,speeds)
                expected=np.array(request)*np.array([raw[8],raw[8],raw[9],raw[8]])
                np.testing.assert_allclose(actual,expected,atol=.002)

if __name__=='__main__':unittest.main()
