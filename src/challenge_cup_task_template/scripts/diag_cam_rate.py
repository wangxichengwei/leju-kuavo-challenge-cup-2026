#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""诊断相机话题发布频率。"""
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
launcher = Launcher(scene="scene1", seed=0, match_time_limit=0, timer_gui=False)
launcher.start(node_name="cam_rate", timeout=180)
rospy.wait_for_message("/bot_stand_up_complete", rospy.AnyMsg, timeout=150)
rospy.sleep(3.0)

from sensor_msgs.msg import CompressedImage
import threading

counts = {}
for topic in ['/cam_h/color/image_raw/compressed', '/cam_h/depth/image_raw/compressedDepth',
              '/cam_r/color/image_raw/compressed']:
    counts[topic] = [0]
    def cb(msg, t=topic):
        counts[t][0] += 1
    rospy.Subscriber(topic, CompressedImage, cb, queue_size=10)

rospy.sleep(10.0)  # 统计 10 秒
for topic, c in counts.items():
    rospy.loginfo("CR: %s = %.1f Hz", topic, c[0] / 10.0)
rospy.loginfo("CR_DONE")
rospy.signal_shutdown("done")
