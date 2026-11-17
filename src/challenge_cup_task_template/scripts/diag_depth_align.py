#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""深度图 3D 定位：像素+深度 -> base 坐标，对比平面求交。"""
import importlib.util, pathlib, os, sys, math, numpy as np

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
launcher.start(node_name="depth_align", timeout=180)
rospy.wait_for_message("/bot_stand_up_complete", rospy.AnyMsg, timeout=150)

import cv2
from sensor_msgs.msg import CompressedImage
# 颜色图 + 深度图（同帧近似）
color_msg = rospy.wait_for_message('/cam_h/color/image_raw/compressed', CompressedImage, timeout=15)
depth_msg = rospy.wait_for_message('/cam_h/depth/image_raw/compressedDepth', CompressedImage, timeout=15)
color = cv2.imdecode(np.frombuffer(color_msg.data, np.uint8), cv2.IMREAD_COLOR)
depth = cv2.imdecode(np.frombuffer(depth_msg.data[12:], np.uint8), cv2.IMREAD_UNCHANGED).astype(np.float32)
rospy.loginfo("DA: color=%s depth=%s", color.shape, depth.shape)

# 相机->base 旋转矩阵
def quat_to_mat(q):
    x, y, z, w = q
    return np.array([
        [1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w],
        [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w],
        [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y],
    ])
R = quat_to_mat(ct.CAM_QUAT)
CAM_POS = np.array(ct.CAM_POS)

def px_depth_to_base(u, v):
    """像素 + 深度(mm) -> base 3D 坐标。"""
    d = depth[int(v), int(u)]
    if d <= 0 or d > 4000:
        return None
    dm = d / 1000.0  # 毫米->米
    xc = (u - ct.CAM_CX) / ct.CAM_FX * dm
    yc = (v - ct.CAM_CY) / ct.CAM_FY * dm
    zc = dm
    pcam = np.array([xc, yc, zc])
    pbase = CAM_POS + R @ pcam
    return pbase

# 检测黄色/白色包裹像素 + 称重区像素
hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
H = hsv[:,:,0]; S = hsv[:,:,1]; V = hsv[:,:,2]
# 包裹（黄色：H 20-35, 高S；白色：低S 高V）
parcel_mask = (((H>=18)&(H<=35)&(S>80)) | ((S<50)&(V>160))).astype(np.uint8)*255
# 称重区（亮绿）
roi = np.zeros_like(parcel_mask); roi[int(color.shape[0]*0.5):, int(color.shape[1]*0.5):] = 255
green = ((H>=40)&(H<=85)&(S>60)&(V>120)).astype(np.uint8)*255
green = cv2.bitwise_and(green, roi)

# 检测包裹连通域
n, labels, stats, cents = cv2.connectedComponentsWithStats(parcel_mask, 8)
parcels = []
for i in range(1, n):
    x, y, bw, bh, area = stats[i]
    if area < 300 or area > 6000:
        continue
    u, v = int(x+bw/2), int(y+bh/2)
    p = px_depth_to_base(u, v)
    if p is not None and p[0] < 0.7:
        parcels.append((p, area))
parcels.sort(key=lambda t: t[0][0])
rospy.loginfo("DA: 包裹(深度3D): %d 个", len(parcels))
for p, area in parcels:
    rospy.loginfo("DA:   base=(%.3f,%.3f,%.3f) area=%d", p[0], p[1], p[2], area)

# 称重区
n2, labels2, stats2, cents2 = cv2.connectedComponentsWithStats(green, 8)
best = None; best_area = 0
for i in range(1, n2):
    if stats2[i][4] > best_area:
        best_area = stats2[i][4]; best = i
if best:
    x, y, bw, bh, area = stats2[best]
    u, v = int(x+bw/2), int(y+bh/2)
    p = px_depth_to_base(u, v)
    rospy.loginfo("DA: 称重区 base=(%.3f,%.3f,%.3f) area=%d", p[0], p[1], p[2], area)
rospy.loginfo("DA_DONE")
rospy.signal_shutdown("done")
