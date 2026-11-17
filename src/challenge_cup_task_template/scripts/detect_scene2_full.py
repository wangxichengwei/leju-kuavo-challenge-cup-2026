#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scene2 完整检测：颜色分割 -> base 坐标 + 类型。返回 parts 和 bins。"""
import importlib.util, pathlib, cv2, numpy as np

PKG = pathlib.Path("/root/kuavo_ws/src/challenge_cup_task_template")
spec = importlib.util.spec_from_file_location("challenge_task", PKG / "scripts" / "challenge_task.py")
ct = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ct)

def to_base(u, v):
    bx, by = ct._pixel_to_base(u, v, ct.PARCEL_PLANE_Z_BASE)
    bx += ct.CAM_X_CORRECTION
    return bx, by

def detect_scene2(img):
    h, w = img.shape[:2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    H = hsv[:,:,0]; S = hsv[:,:,1]; V = hsv[:,:,2]
    parcel_roi = np.zeros((h,w), np.uint8); parcel_roi[int(h*0.55):, :] = 255

    def blobs(mask, roi=None, area_lo=200, area_hi=5000, is_box=False):
        if roi is not None:
            mask = cv2.bitwise_and(mask, roi)
        k = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        if is_box:
            area_lo, area_hi = 5000, 30000
        n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
        out = []
        for i in range(1, n):
            x, y, bw, bh, area = stats[i]
            if area < area_lo or area > area_hi:
                continue
            u, v = x+bw/2.0, y+bh/2.0
            out.append((to_base(u, v), (int(u), int(v)), int(area)))
        return out

    red = ((H < 10) | (H > 170)) & (S > 80) & (V > 60)
    black = (V < 90) & (S < 100)
    silver = (S < 60) & (V > 120)
    blue = (H >= 90) & (H <= 130) & (S > 80) & (V > 60)
    orange = (H >= 5) & (H <= 25) & (S > 60) & (V > 40)
    purple = (H >= 130) & (H <= 160) & (S > 80) & (V > 60)

    parts = []  # (type, base_xy)
    for base_xy, px, area in blobs(red.astype(np.uint8)*255, parcel_roi):
        parts.append(("C", base_xy))
    for base_xy, px, area in blobs(black.astype(np.uint8)*255, parcel_roi):
        parts.append(("B", base_xy))
    # 银管件：排除螺丝刀银杆（在红螺丝刀附近、细长）
    silver_blobs = blobs(silver.astype(np.uint8)*255, parcel_roi)
    red_px = [p[1] for p in [b[1] for b in []]]  # placeholder
    for base_xy, px, area in silver_blobs:
        # 简单启发：银管件面积大(>500)，螺丝刀银杆面积小
        if area > 500:
            parts.append(("A", base_xy))

    bins = {}
    for base_xy, px, area in blobs(blue.astype(np.uint8)*255, is_box=True):
        bins["A"] = base_xy
    for base_xy, px, area in blobs(orange.astype(np.uint8)*255, is_box=True):
        bins["B"] = base_xy
    for base_xy, px, area in blobs(purple.astype(np.uint8)*255, is_box=True):
        bins["C"] = base_xy

    return parts, bins

if __name__ == "__main__":
    img = cv2.imread("/tmp/scene2_head.png")
    parts, bins = detect_scene2(img)
    print("PARTS:")
    for t, b in sorted(parts):
        print("  type=%s base=(%.3f, %.3f)" % (t, b[0], b[1]))
    print("BINS:")
    for k in sorted(bins):
        print("  bin_%s base=(%.3f, %.3f)" % (k, bins[k][0], bins[k][1]))
    print("DONE")
