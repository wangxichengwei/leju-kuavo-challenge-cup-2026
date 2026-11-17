#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scene2 视觉伺服探测：pitch=-45 预抓取位下拍腕相机+头部相机。"""
import importlib.util, pathlib, os, sys, math

PKG = pathlib.Path("/root/kuavo_ws/src/challenge_cup_task_template")
spec = importlib.util.spec_from_file_location("challenge_task", PKG / "scripts" / "challenge_task.py")
ct = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ct)
spec2 = importlib.util.spec_from_file_location("detect_scene2", PKG / "scripts" / "detect_scene2_full.py")
d2 = importlib.util.module_from_spec(spec2)
spec2.loader.exec_module(d2)

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
launcher.start(node_name="scene2_servo", timeout=180)
rospy.wait_for_message("/bot_stand_up_complete", rospy.AnyMsg, timeout=150)
runner = ct.Scene1Runner()
runner.set_arm_mode(ct.ARM_MODE_EXTERNAL_CONTROL, 20.0)
arm_pub = runner.arm_pub
cur = runner.read_current_arm_rad(90)
arm_hold = ct.ArmHold(arm_pub, [math.degrees(v) for v in cur])
arm_hold.start()
rospy.sleep(0.5)
runner.move_arm_deg(arm_hold, ct.PRESET_FINAL_DEG, move_time=2.0, settle=1.2)
runner.set_gripper(ct.GRIP_OPEN, ct.GRIP_OPEN)

import cv2, numpy as np
from sensor_msgs.msg import CompressedImage
m = rospy.wait_for_message('/cam_h/color/image_raw/compressed', CompressedImage, timeout=15)
img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
parts, bins = d2.detect_scene2(img)
target = None
for t, b in parts:
    if t in ("A","B") and b[0] < 0.35:
        target = (t, b); break
if target is None: target = parts[0]
t, (px, py) = target
rospy.loginfo("SV: target type=%s base=(%.3f,%.3f)", t, px, py)

quat = ct.quat_from_ypr([0, -45, 0], [90, 0, 0])
# 到预抓取位（零件上方 z=0.08）
runner.move_right_cartesian_to(arm_hold, [px, py, 0.08], quat, n_points=6)
rospy.sleep(1.0)

# 拍腕相机 + 头部相机
for topic, path in [('/cam_r/color/image_raw/compressed', '/tmp/servo_wrist.png'),
                    ('/cam_h/color/image_raw/compressed', '/tmp/servo_head.png')]:
    try:
        mm = rospy.wait_for_message(topic, CompressedImage, timeout=8)
        im = cv2.imdecode(np.frombuffer(mm.data, np.uint8), cv2.IMREAD_COLOR)
        cv2.imwrite(path, im)
        rospy.loginfo("SV: saved %s", path)
    except Exception as e:
        rospy.logwarn("SV: %s fail %s", topic, e)

rospy.loginfo("SV_DONE")
rospy.signal_shutdown("done")
