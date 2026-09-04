#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

def manual_control():
    rospy.init_node('manual_control', anonymous=False)
    pub = rospy.Publisher('cmd_message', String, queue_size=10)

    # 等待发布者连接
    rospy.sleep(0.5)

    print("=" * 60)
    print("机器人手动控制程序")
    print("=" * 60)
    print("支持的命令:")
    print("  xyz <x> <y> <z>        - 移动到3D坐标")
    print("  depyaw <深度> <yaw>    - 深度+航向控制")
    print("  yaw <角度>             - 航向角控制")
    print("  dep <深度>             - 深度控制")
    print("  st                     - 停止")
    print("  quit/exit              - 退出程序")
    print("=" * 60)
    print("")

    while not rospy.is_shutdown():
        try:
            cmd = input("请输入命令: ")

            # 去除首尾空格
            cmd = cmd.strip()

            # 退出命令
            if cmd.lower() in ['quit', 'exit', 'q']:
                print("退出控制程序...")
                break

            # 空命令跳过
            if not cmd:
                continue

            # 发布命令
            msg = String()
            msg.data = cmd
            pub.publish(msg)

            print("✓ 命令已发送: '{}'".format(cmd))
            print("")

        except KeyboardInterrupt:
            print("\n检测到Ctrl+C，退出程序...")
            break
        except EOFError:
            print("\n输入结束，退出程序...")
            break
        except Exception as e:
            print("错误: {}".format(e))

if __name__ == '__main__':
    try:
        manual_control()
    except rospy.ROSInterruptException:
        pass
