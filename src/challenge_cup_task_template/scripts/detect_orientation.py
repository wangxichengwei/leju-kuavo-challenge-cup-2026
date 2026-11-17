#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scene2 朝向检测：颜色分割 + PCA 主轴，输出每个零件的朝向角度。"""
import cv2
import numpy as np

img = cv2.imread("/tmp/scene2_head.png")
h, w = img.shape[:2]
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
H = hsv[:,:,0]; S = hsv[:,:,1]; V = hsv[:,:,2]
parcel_roi = np.zeros((h,w), np.uint8); parcel_roi[int(h*0.55):, :] = 255

def orientation(name, mask, area_lo=200, area_hi=5000):
    mask = cv2.bitwise_and(mask, parcel_roi)
    k = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
    print("=== %s ===" % name)
    for i in range(1, n):
        x, y, bw, bh, area = stats[i]
        if area < area_lo or area > area_hi:
            continue
        ys, xs = np.where(labels == i)
        if len(xs) < 10:
            continue
        pts = np.stack([xs, ys], axis=1).astype(np.float32)
        # PCA 主轴
        mean, eigvec = cv2.PCACompute(pts, mean=None)
        v = eigvec[0]  # 第一主成分（主轴方向）
        angle = np.degrees(np.arctan2(v[1], v[0]))  # -180~180
        # 长宽比（用特征值）
        cov = np.cov(pts.T)
        evals, evecs = np.linalg.eig(cov)
        evals = sorted(evals, reverse=True)
        ratio = (evals[0]/evals[1]) if evals[1] > 1e-6 else 999
        # minAreaRect 对照
        rect = cv2.minAreaRect(pts)
        ra = rect[2]
        print("  blob i=%d area=%d center=(%d,%d) pca_angle=%.1f deg lw_ratio=%.1f minrect_angle=%.1f" %
              (i, area, int(x+bw/2), int(y+bh/2), angle, ratio, ra))

red = ((H < 10) | (H > 170)) & (S > 80) & (V > 60)
orientation("RED(C screwdriver)", red.astype(np.uint8)*255)
black = (V < 90) & (S < 100)
orientation("BLACK(B)", black.astype(np.uint8)*255)
silver = (S < 60) & (V > 120)
orientation("SILVER(A)", silver.astype(np.uint8)*255)
print("DONE")
