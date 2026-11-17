#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scene2 试错式伺服：网格微调 x/y，GRASP_CHECK 反馈。"""
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
launcher.start(node_name="scene2_retry", timeout=180)
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
rospy.loginfo("RT: target type=%s base=(%.3f,%.3f)", t, px, py)

quat = ct.quat_from_ypr([0, -45, 0], [90, 0, 0])
GRID = [(-0.04,0), (-0.02,0), (-0.06,0), (-0.04,0.02), (-0.04,-0.02), (-0.06,0.02), (-0.02,-0.02)]
best = None
for dx, dy in GRID:
    gx, gy = px + dx, py + dy
    try:
        runner.move_right_cartesian_to(arm_hold, [gx, gy, -0.02], quat, n_points=6)
        rospy.sleep(0.3)
        runner.set_gripper(ct.GRIP_OPEN, ct.GRIP_CLOSE)
        rospy.sleep(1.5)
        g = runner.read_gripper_state()
        rospy.loginfo("RT: try (dx=%.2f,dy=%.2f) GRASP=%.4f", dx, dy, g if g else -1)
        if best is None or (g is not None and g < best[0]):
            best = (g, dx, dy)
        runner.set_gripper(ct.GRIP_OPEN, ct.GRIP_OPEN)
        rospy.sleep(0.5)
        runner.move_right_cartesian_to(arm_hold, [gx, gy, 0.08], quat, n_points=3)
        rospy.sleep(0.3)
    except Exception as e:
        rospy.logwarn("RT: try (dx=%.2f,dy=%.2f) fail: %s", dx, dy, e)

if best:
    rospy.loginfo("RT_BEST GRASP=%.4f at (dx=%.2f, dy=%.2f)", best[0], best[1], best[2])
rospy.loginfo("RT_DONE")
rospy.signal_shutdown("done")
