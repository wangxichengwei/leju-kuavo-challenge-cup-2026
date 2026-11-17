#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""detect 误检诊断：在 pregrasp 图上画出 detect 的 blob 框+质心，供 vision 判断。"""
import importlib.util, pathlib, os, sys
PKG = pathlib.Path("/root/kuavo_ws/src/challenge_cup_task_template")
spec = importlib.util.spec_from_file_location("challenge_task", PKG / "scripts" / "challenge_task.py")
ct = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ct)

import cv2
import numpy as np

img = cv2.imread("/tmp/pregrasp_head.png")
if img is None:
    print("FAIL: no /tmp/pregrasp_head.png")
    sys.exit(1)
h, w = img.shape[:2]
print("IMG", w, h)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
V = hsv[:, :, 2]
mask = ((V > 165)).astype(np.uint8) * 255
roi = np.zeros((h, w), np.uint8); roi[int(h*0.25):, :] = 255
mask = cv2.bitwise_and(mask, roi)
k = np.ones((5, 5), np.uint8)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, 8)

vis = img.copy()
for i in range(1, n):
    x, y, bw, bh, area = stats[i]
    if area < 800 or area > 6000:
        continue
    u = x + bw / 2.0; v = y + bh / 2.0
    bx, by = ct._pixel_to_base(u, v, ct.PARCEL_PLANE_Z_BASE)
    bx += ct.CAM_X_CORRECTION
    cv2.rectangle(vis, (x, y), (x+bw, y+bh), (0, 0, 255), 2)
    cv2.circle(vis, (int(u), int(v)), 6, (0, 255, 0), -1)
    cv2.putText(vis, "i%d a%d" % (i, area), (x, max(10, y-6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    print("blob i=%d bbox=(%d,%d %dx%d) area=%d center_px=(%d,%d) base=(%.3f,%.3f)" %
          (i, x, y, bw, bh, area, int(u), int(v), bx, by))

cv2.imwrite("/tmp/detect_mask.png", vis)
print("saved /tmp/detect_mask.png")
