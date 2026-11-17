#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scene3 TF 探测：base_link 相对 world 的变换。"""
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
launcher.start(node_name="scene3_tf", timeout=180)
rospy.wait_for_message("/bot_stand_up_complete", rospy.AnyMsg, timeout=150)

import tf
lst = tf.TransformListener()
rospy.sleep(2.0)
for target, source in [("base_link", "world"), ("base_link", "map"), ("world", "base_link"), ("map", "base_link")]:
    try:
        lst.waitForTransform(target, source, rospy.Time(0), rospy.Duration(3))
        pos, quat = lst.lookupTransform(target, source, rospy.Time(0))
        rospy.loginfo("S3TF: %s -> %s pos=(%.3f,%.3f,%.3f)", source, target, pos[0], pos[1], pos[2])
    except Exception as e:
        rospy.logwarn("S3TF: %s -> %s fail: %s", source, target, str(e)[:80])

# 列所有 frame（含 world 的）
try:
    frames = lst.getFrameStrings()
    rospy.loginfo("S3TF: frames=%s", [f for f in frames if 'world' in f or 'base' in f or 'map' in f])
except Exception as e:
    rospy.logwarn("S3TF: frames fail %s", e)

rospy.loginfo("S3TF_DONE")
rospy.signal_shutdown("done")
