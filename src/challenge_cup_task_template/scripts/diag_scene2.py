#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scene2 探测：启动仿真，拍头部相机图。"""
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
launcher = Launcher(scene="scene2", seed=0, match_time_limit=0, timer_gui=False)
launcher.start(node_name="scene2_probe", timeout=180)
rospy.loginfo("S2: waiting standup")
try:
    rospy.wait_for_message("/bot_stand_up_complete", rospy.AnyMsg, timeout=150)
except Exception as e:
    rospy.logwarn("S2: standup: %s", e)

import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
for topic, path in [('/cam_h/color/image_raw/compressed', '/tmp/scene2_head.png'),
                    ('/cam_r/color/image_raw/compressed', '/tmp/scene2_wrist.png')]:
    try:
        m = rospy.wait_for_message(topic, CompressedImage, timeout=10)
        img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
        cv2.imwrite(path, img)
        rospy.loginfo("S2: saved %s", path)
    except Exception as e:
        rospy.logwarn("S2: %s fail: %s", topic, e)

rospy.loginfo("S2_DONE")
rospy.signal_shutdown("done")
