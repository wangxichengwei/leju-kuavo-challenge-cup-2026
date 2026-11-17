#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scene3 料盘检测完整版：V<90 S<90 + 圆点过滤 -> base 坐标。"""
import importlib.util, pathlib, cv2, numpy as np

PKG = pathlib.Path("/root/kuavo_ws/src/challenge_cup_task_template")
spec = importlib.util.spec_from_file_location("challenge_task", PKG / "scripts" / "challenge_task.py")
ct = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ct)

def detect_trays(img):
    h, w = img.shape[:2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    H = hsv[:,:,0]; S = hsv[:,:,1]; V = hsv[:,:,2]
    roi = np.zeros((h,w), np.uint8); roi[180:440, 560:840] = 255
    mask = ((V < 90) & (S < 90)).astype(np.uint8) * 255
    mask = cv2.bitwise_and(mask, roi)
    k = np.ones((3,3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
    trays = []
    for i in range(1, n):
        x, y, bw, bh, area = stats[i]
        if area < 150 or area > 1500:
            continue
        ratio = max(bw, bh) / float(max(min(bw, bh), 1))
        if ratio >= 2.5:
            continue
        u, v = x + bw/2.0, y + bh/2.0
        bx, by = ct._pixel_to_base(u, v, ct.PARCEL_PLANE_Z_BASE)
        bx += ct.CAM_X_CORRECTION
        trays.append(((int(u), int(v)), (bx, by), int(area)))
    trays.sort(key=lambda t: t[1][0])
    return trays

if __name__ == "__main__":
    img = cv2.imread("/tmp/scene3_head.png")
    trays = detect_trays(img)
    print("检测到 %d 个料盘：" % len(trays))
    for px, base, area in trays:
        print("  px=(%d,%d) base=(%.3f,%.3f) area=%d" % (px[0], px[1], base[0], base[1], area))
    print("DONE")
