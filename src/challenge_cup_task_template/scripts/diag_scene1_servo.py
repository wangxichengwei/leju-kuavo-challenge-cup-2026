#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scene1 释放伺服探测：到释放点后检测夹爪(白)与称重区(亮绿)像素。"""
import importlib.util, pathlib, os, sys, math

PKG = pathlib.Path("/root/kuavo_ws/src/challenge_cup_task_template")
spec = importlib.util.spec_from_file_location("challenge_task", PKG / "scripts" / "challenge_task.py")
ct = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ct)

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
launcher.start(node_name="scene1_servo", timeout=180)
rospy.wait_for_message("/bot_stand_up_complete", rospy.AnyMsg, timeout=150)
runner = ct.Scene1Runner()
runner.set_arm_mode(ct.ARM_MODE_EXTERNAL_CONTROL, 20.0)
arm_pub = runner.arm_pub
cur = runner.read_current_arm_rad(90)
arm_hold = ct.ArmHold(arm_pub, [math.degrees(v) for v in cur])
arm_hold.start()
rospy.sleep(0.5)
runner.move_arm_deg(arm_hold, ct.PRESET_FINAL_DEG, move_time=2.0, settle=1.2)

# 到释放点
quat = runner.weigh_release_quat
runner.right_arm_to(arm_hold, [ct.WEIGH_RELEASE_IK[0], ct.WEIGH_RELEASE_IK[1], ct.WEIGH_TRANSIT_IK_Z], quat)
runner.right_arm_to(arm_hold, ct.WEIGH_RELEASE_IK, quat)
rospy.sleep(1.0)

import cv2, numpy as np
from sensor_msgs.msg import CompressedImage
m = rospy.wait_for_message('/cam_h/color/image_raw/compressed', CompressedImage, timeout=15)
img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
cv2.imwrite("/tmp/servo_release.png", img)
h, w = img.shape[:2]
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
H = hsv[:,:,0]; S = hsv[:,:,1]; V = hsv[:,:,2]

# 称重区（亮绿）
roi = np.zeros((h,w), np.uint8); roi[int(h*0.5):, int(w*0.5):] = 255
green = ((H>=40)&(H<=85)&(S>60)&(V>120)).astype(np.uint8)*255
green = cv2.bitwise_and(green, roi)
n, labels, stats, cents = cv2.connectedComponentsWithStats(green, 8)
best = None; best_area = 0
for i in range(1, n):
    if stats[i][4] > best_area:
        best_area = stats[i][4]; best = i
if best:
    x,y,bw,bh,area = stats[best]
    gc = (int(x+bw/2), int(y+bh/2))
    rospy.loginfo("S1S: 称重区 center_px=(%d,%d) area=%d", gc[0], gc[1], area)
else:
    gc = None
    rospy.logwarn("S1S: 称重区未检出")

# 夹爪（白色）
white = ((S<60)&(V>150)).astype(np.uint8)*255
n2, labels2, stats2, cents2 = cv2.connectedComponentsWithStats(white, 8)
rospy.loginfo("S1S: 白色连通域：")
for i in range(1, n2):
    x,y,bw,bh,area = stats2[i]
    if area < 200:
        continue
    rospy.loginfo("S1S:   i=%d center=(%d,%d) area=%d", i, int(x+bw/2), int(y+bh/2), area)

rospy.loginfo("S1S_DONE")
rospy.signal_shutdown("done")
