#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scene2 离线颜色分割 v2：调橙色 + 零件 ROI。"""
import cv2
import numpy as np

img = cv2.imread("/tmp/scene2_head.png")
h, w = img.shape[:2]
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
H = hsv[:,:,0]; S = hsv[:,:,1]; V = hsv[:,:,2]

# 零件在画面底部（y > h*0.6），箱子在画面中部
parcel_roi = np.zeros((h,w), np.uint8); parcel_roi[int(h*0.6):, :] = 255

def detect(name, mask, roi=None, area_lo=200, area_hi=20000):
    if roi is not None:
        mask = cv2.bitwise_and(mask, roi)
    k = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
    items = []
    for i in range(1, n):
        x, y, bw, bh, area = stats[i]
        if area < area_lo or area > area_hi:
            continue
        items.append((int(x+bw/2), int(y+bh/2), int(area)))
    items.sort(key=lambda t: (t[0], t[1]))
    print("%s: %d -> %s" % (name, len(items), items))

red = ((H < 10) | (H > 170)) & (S > 80) & (V > 60)
detect("RED", red.astype(np.uint8)*255, parcel_roi)

black = (V < 90) & (S < 100)
detect("BLACK", black.astype(np.uint8)*255, parcel_roi)

silver = (S < 60) & (V > 120)
detect("SILVER", silver.astype(np.uint8)*255, parcel_roi)

# 橙箱：H 5-25 试
for hlo, hhi in [(5,25),(8,20),(10,18)]:
    orange = (H >= hlo) & (H <= hhi) & (S > 80) & (V > 60)
    cnt = int(np.sum(orange))
    print("orange H[%d,%d]: px=%d" % (hlo, hhi, cnt))

blue = (H >= 90) & (H <= 130) & (S > 80) & (V > 60)
detect("BLUE", blue.astype(np.uint8)*255)

purple = (H >= 130) & (H <= 160) & (S > 80) & (V > 60)
detect("PURPLE", purple.astype(np.uint8)*255)

# 橙色用最宽的试 detect
orange = (H >= 5) & (H <= 25) & (S > 60) & (V > 40)
detect("ORANGE(wide)", orange.astype(np.uint8)*255)

print("DONE")
