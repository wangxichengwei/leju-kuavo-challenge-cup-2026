#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scene3 走路到货架前 + 检测料盘可达性。"""
import importlib.util, pathlib, os, sys

PKG = pathlib.Path("/root/kuavo_ws/src/challenge_cup_task_template")
spec = importlib.util.spec_from_file_location("challenge_task", PKG / "scripts" / "challenge_task.py")
ct = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ct)
spec3 = importlib.util.spec_from_file_location("detect_scene3", PKG / "scripts" / "detect_scene3_full.py")
d3 = importlib.util.module_from_spec(spec3)
spec3.loader.exec_module(d3)

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
launcher.start(node_name="scene3_full", timeout=180)
rospy.wait_for_message("/bot_stand_up_complete", rospy.AnyMsg, timeout=150)

import cv2, numpy as np
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from kuavo_msgs.msg import switchGaitByName

# 走路到货架前 world (0.6, 0)
gait_pub = rospy.Publisher("/humanoid_switch_gait_by_name", switchGaitByName, queue_size=5)
pose_pub = rospy.Publisher("/cmd_pose_world", Twist, queue_size=5)
rospy.sleep(1.0)
g = switchGaitByName(); g.header.stamp = rospy.Time.now(); g.gait_name = "walk"
for _ in range(5):
    gait_pub.publish(g); rospy.sleep(0.1)
rospy.sleep(2.0)
cmd = Twist(); cmd.linear.x = 0.6; cmd.linear.y = 0.0; cmd.angular.z = 0.0
r = rospy.Rate(10)
for _ in range(1700):  # 170s 走路
    pose_pub.publish(cmd); r.sleep()
rospy.loginfo("S3F: walk 170s done")
rospy.sleep(3.0)

# 检测料盘
m = rospy.wait_for_message('/cam_h/color/image_raw/compressed', CompressedImage, timeout=15)
img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
cv2.imwrite("/tmp/scene3_after_walk.png", img)
trays = d3.detect_trays(img)
rospy.loginfo("S3F: 检测到 %d 个料盘", len(trays))
for px, base, area in trays:
    rospy.loginfo("S3F: tray base=(%.3f,%.3f) area=%d", base[0], base[1], area)
rospy.loginfo("S3F_DONE")
rospy.signal_shutdown("done")
