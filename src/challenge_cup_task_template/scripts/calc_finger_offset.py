#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""计算释放姿态下手指尖相对 end_effector 的 base 偏移。"""
import importlib.util, pathlib, math, numpy as np

PKG = pathlib.Path("/root/kuavo_ws/src/challenge_cup_task_template")
spec = importlib.util.spec_from_file_location("challenge_task", PKG / "scripts" / "challenge_task.py")
ct = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ct)

quat = ct.quat_from_ypr(ct.WEIGH_RELEASE_FIRST_YPR, ct.WEIGH_RELEASE_SECOND_YPR)
print("释放姿态 quat(xyzw)=", [round(q,4) for q in quat])

def quat_to_mat(q):
    x, y, z, w = q
    return np.array([
        [1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w],
        [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w],
        [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y],
    ])

R = quat_to_mat(quat)
print("旋转矩阵：")
print(np.round(R, 3))

fingertip_local = np.array([0.0, 0.0, -0.047])
fingertip_base = R @ fingertip_local
print("手指尖局部(0,0,-0.047) -> base offset =", np.round(fingertip_base, 4))

z_local = np.array([0.0, 0.0, -1.0])
z_base = R @ z_local
print("局部 -z 轴在 base 的投影 =", np.round(z_base, 4))
print("DONE")
