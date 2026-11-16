#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线检测对比：在同一张头部相机图上，用几种分割方法找包裹，对比结果。"""
import importlib.util, pathlib, os, sys

PKG = pathlib.Path("/root/kuavo_ws/src/challenge_cup_task_template")
SCRIPT = PKG / "scripts" / "challenge_task.py"
spec = importlib.util.spec_from_file_location("challenge_task", SCRIPT)
ct = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ct)

import cv2
import numpy as np

img = cv2.imread("/tmp/pregrasp_head.png")
if img is None:
    print("FAIL: cannot read /tmp/pregrasp_head.png")
    sys.exit(1)
h, w = img.shape[:2]
print("IMG size:", w, h)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
H = hsv[:, :, 0]; S = hsv[:, :, 1]; V = hsv[:, :, 2]

roi = np.zeros((h, w), np.uint8); roi[int(h*0.25):, :] = 255

def to_base(u, v):
    bx, by = ct._pixel_to_base(u, v, ct.PARCEL_PLANE_Z_BASE)
    bx += ct.CAM_X_CORRECTION
    return bx, by

def report(name, mask, area_lo=300, area_hi=8000):
    mask = cv2.bitwise_and(mask, roi)
    k = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    n, _, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
    items = []
    for i in range(1, n):
        x, y, bw, bh, area = stats[i]
        if area < area_lo or area > area_hi:
            continue
        u, v = cents[i]
        bx, by = to_base(u, v)
        if bx < 0.0 or bx > 0.7:
            continue
        items.append((round(bx,3), round(by,3), int(area)))
    items.sort()
    print("%s: %d blobs -> %s" % (name, len(items), items))

# 方法1：当前 V>165
report("V>165 (current)", ((V > 165)).astype(np.uint8)*255)

# 方法2：非绿色（hue 排除 35~90）+ 亮度
green = ((H >= 35) & (H <= 90)).astype(np.uint8)
nongreen = (1 - green).astype(np.uint8) * 255
report("non-green hue", ((nongreen>0) & (V>90)).astype(np.uint8)*255)

# 方法3：饱和度 S>60（彩色物体）
report("S>60", ((S > 60)).astype(np.uint8)*255)

# 方法4：非绿 且 饱和度足够（彩色盒子，排除白色夹爪/标签）
report("non-green AND S>50", (((nongreen>0) & (S>50))).astype(np.uint8)*255)

# 方法5：亮度较高但排除绿色（盒子+白色物体，含夹爪）
report("non-green AND V>140", (((nongreen>0) & (V>140))).astype(np.uint8)*255)

print("DONE")
