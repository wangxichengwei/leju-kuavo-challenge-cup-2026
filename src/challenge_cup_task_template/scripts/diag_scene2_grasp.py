#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scene2 抓取验证：检测一个管件，用 pitch=-45 姿态抓取。"""
import importlib.util, pathlib, os, sys, math

PKG = pathlib.Path("/root/kuavo_ws/src/challenge_cup_task_template")
spec = importlib.util.spec_from_file_location("challenge_task", PKG / "scripts" / "challenge_task.py")
ct = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ct)

# 加载 scene2 检测
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
launcher.start(node_name="scene2_grasp", timeout=180)
rospy.loginfo("SG: waiting standup")
try:
    rospy.wait_for_message("/bot_stand_up_complete", rospy.AnyMsg, timeout=150)
except Exception as e:
    rospy.logwarn("SG: standup: %s", e)

runner = ct.Scene1Runner()
try:
    runner.set_arm_mode(ct.ARM_MODE_EXTERNAL_CONTROL, 20.0)
except Exception as e:
    rospy.logerr("SG: arm mode fail: %s", e)

arm_pub = runner.arm_pub
cur = runner.read_current_arm_rad(90)
arm_hold = ct.ArmHold(arm_pub, [math.degrees(v) for v in cur])
arm_hold.start()
rospy.sleep(0.5)
runner.move_arm_deg(arm_hold, ct.PRESET_FINAL_DEG, move_time=2.0, settle=1.2)
runner.set_gripper(ct.GRIP_OPEN, ct.GRIP_OPEN)
rospy.loginfo("SG: preset done")

# 读相机图 + 检测
import cv2, numpy as np
from sensor_msgs.msg import CompressedImage
m = rospy.wait_for_message('/cam_h/color/image_raw/compressed', CompressedImage, timeout=15)
img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
parts, bins = d2.detect_scene2(img)
rospy.loginfo("SG: detected parts=%s", [(t, (round(b[0],3), round(b[1],3))) for t,b in parts])
rospy.loginfo("SG: bins=%s", {k:(round(b[0],3),round(b[1],3)) for k,b in bins.items()})

# 选一个管件（A 或 B），排除银杆误检
target = None
for t, b in parts:
    if t in ("A", "B") and b[0] < 0.35:  # 管件在近端 x<0.35
        target = (t, b)
        break
if target is None:
    target = parts[0]
t, (px, py) = target
rospy.loginfo("SG: target type=%s base=(%.3f,%.3f)", t, px, py)

# 管件抓取姿态：pitch=-45
quat = ct.quat_from_ypr([0, -45, 0], [90, 0, 0])
gx, gy, gz = px, py, -0.02  # 管件在桌面，抓取 z 估计

# 笛卡尔直线分段抓取（对齐官方）
try:
    runner.move_right_cartesian_to(arm_hold, [gx, gy, gz], quat, n_points=6)
    rospy.sleep(0.5)
    runner.set_gripper(ct.GRIP_OPEN, ct.GRIP_CLOSE)
    rospy.sleep(2.0)
    g = runner.read_gripper_state()
    rospy.loginfo("SG: GRASP_CHECK right_gripper=%s", g)
    # FK 实测
    cur_rad = runner.read_current_arm_rad(10)
    fk = runner.call_fk(cur_rad, 5.0)
    rp = list(fk.right_pose.pos_xyz)
    rospy.loginfo("SG: FK_right=(%.3f,%.3f,%.3f) vs grasp=(%.3f,%.3f,%.3f)", rp[0], rp[1], rp[2], gx, gy, gz)
    runner.save_head_snapshot("/tmp/scene2_grasp.png")
except Exception as e:
    rospy.logerr("SG: grasp fail: %s", e)

rospy.loginfo("SG_DONE")
rospy.signal_shutdown("done")
