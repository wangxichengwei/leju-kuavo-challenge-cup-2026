#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scene3 走路探测 v2：切 walk + cmd_vel 0.2 走 10s，拍前后图。"""
import os, sys

def _load_launcher():
    import rospkg
    su = os.path.join(rospkg.RosPack().get_path("challenge_cup_simulator"), "utils")
    if su not in sys.path:
        sys.path.insert(0, su)
    from challenge_sim_launcher import ChallengeSimLauncher
    return ChallengeSimLauncher

import rospy
Launcher = _load_launcher()
launcher = Launcher(scene="scene3", seed=0, match_time_limit=0, timer_gui=False)
launcher.start(node_name="scene3_walk", timeout=180)
rospy.wait_for_message("/bot_stand_up_complete", rospy.AnyMsg, timeout=150)

import cv2, numpy as np
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

def snap(path):
    for _ in range(3):
        try:
            m = rospy.wait_for_message('/cam_h/color/image_raw/compressed', CompressedImage, timeout=6)
            img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
            cv2.imwrite(path, img)
            return True
        except Exception:
            rospy.sleep(1.0)
    return False

snap("/tmp/walk_before.png")
rospy.loginfo("WALK: before")

from kuavo_msgs.msg import switchGaitByName
gait_pub = rospy.Publisher("/humanoid_switch_gait_by_name", switchGaitByName, queue_size=5)
cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
stop_pub = rospy.Publisher("/stop_robot", Bool, queue_size=5)
rospy.sleep(1.0)

g = switchGaitByName(); g.header.stamp = rospy.Time.now(); g.gait_name = "walk"
for _ in range(5):
    gait_pub.publish(g); rospy.sleep(0.1)
rospy.loginfo("WALK: gait walk")
rospy.sleep(3.0)  # 等步态稳定

cmd = Twist(); cmd.linear.x = 0.2
r = rospy.Rate(20)
for _ in range(200):  # 10s
    cmd_pub.publish(cmd); r.sleep()
rospy.loginfo("WALK: walked 10s at 0.2")

cmd.linear.x = 0.0
for _ in range(10):
    cmd_pub.publish(cmd); rospy.sleep(0.05)
s = Bool(); s.data = True
for _ in range(5):
    stop_pub.publish(s); rospy.sleep(0.1)
rospy.loginfo("WALK: stopped")
rospy.sleep(2.0)

snap("/tmp/walk_after.png")
rospy.loginfo("WALK_DONE")
rospy.signal_shutdown("done")
