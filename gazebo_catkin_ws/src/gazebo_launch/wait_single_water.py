#!/usr/bin/env python3
"""Wait for real bringup readiness, using wall-clock deadlines (not /clock)."""
import argparse
import math
import sys
import time

import rospy
from controller_manager_msgs.srv import ListControllers
from gazebo_msgs.msg import ModelStates
from asrobot.msg import mc


def wait_ready(stage, timeout):
    deadline = time.monotonic() + timeout
    if stage == "world":
        rospy.wait_for_service("/gazebo/spawn_urdf_model", timeout=timeout)
        return
    if stage == "model":
        rospy.wait_for_service(
            "/gazebo_robot1/controller_manager/list_controllers", timeout=timeout
        )
        while time.monotonic() < deadline:
            msg = rospy.wait_for_message(
                "/gazebo/model_states", ModelStates,
                timeout=max(0.1, deadline - time.monotonic()),
            )
            if "gazebo_robot1" in msg.name:
                return
    elif stage == "controllers":
        service = "/gazebo_robot1/controller_manager/list_controllers"
        rospy.wait_for_service(service, timeout=timeout)
        list_controllers = rospy.ServiceProxy(service, ListControllers)
        expected = {"joint_state_controller"} | {
            "joint%d_position_controller" % i for i in range(1, 13)
        }
        while time.monotonic() < deadline:
            running = {
                c.name for c in list_controllers().controller if c.state == "running"
            }
            if expected <= running:
                return
            time.sleep(0.2)
    elif stage == "feedback":
        while time.monotonic() < deadline:
            msg = rospy.wait_for_message(
                "/robot1/mc_message", mc,
                timeout=max(0.1, deadline - time.monotonic()),
            )
            if all(math.isfinite(v) for v in (msg.c_x, msg.c_y, msg.c_z)):
                return
    raise RuntimeError("%s did not become ready within %ss" % (stage, timeout))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("world", "model", "controllers", "feedback"))
    parser.add_argument("--timeout", type=float, default=60)
    args = parser.parse_args()
    rospy.init_node("wait_single_water", anonymous=True, disable_signals=True)
    try:
        wait_ready(args.stage, args.timeout)
    except (rospy.ROSException, rospy.ServiceException, RuntimeError) as exc:
        print("启动检查失败 [%s]: %s" % (args.stage, exc), file=sys.stderr)
        return 1
    print("启动检查通过: %s" % args.stage)
    return 0


if __name__ == "__main__":
    sys.exit(main())
