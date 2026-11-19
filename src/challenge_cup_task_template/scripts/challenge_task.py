#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Scene1 快递称重与摆放 —— 第一版基线实现。

流程（每件快递）：双臂预设 -> 感知(相机/回退) -> 右手预抓取 -> 抓取闭爪 -> 抬升
                 -> 称重区释放 -> 二次抓取 -> (第二阶段再补换手入箱)。

只使用官方合法接口：相机、/ik/*、/kuavo_arm_traj、/gripper/command、/sensors_data_raw。
"""
import argparse
import math
import os
import sys
import threading

import numpy as np

# ---------- 常量（来自官方参考实现） ----------
WORLD_TO_IK_OFFSET = [0.565966, -0.013886, -0.923783]
PARCEL_NAMES = ["parcel_1", "parcel_2", "parcel_3", "parcel_4"]
PARCEL_BASE_WORLD = {
    "parcel_1": [-0.26, -0.31, 0.880],
    "parcel_2": [-0.26, -0.09, 0.880],
    "parcel_3": [-0.11, -0.31, 0.880],
    "parcel_4": [-0.11, -0.09, 0.880],
}
ARM_JOINT_NAMES = ["arm_joint_%d" % i for i in range(1, 15)]
ARM_TRAJ_HZ = 50.0
MOVE_TIME = 1.6
SETTLE = 0.8
GRIP_OPEN = 0.0
GRIP_CLOSE = 255.0
RIGHT_PICK_IK_Z = -0.03
# 深度抓取：末端相对包裹顶面下压量（指尖比末端低约 4.7cm）
GRASP_BELOW_TOP = 0.07
RIGHT_PICK_TRANSIT_IK_Z = 0.12
RIGHT_PICK_NEAR_FAR_Y_THRESHOLD = -0.20
RIGHT_PICK_OFFSET_FAR_ROW = [-0.047, 0.02, 0.0]
RIGHT_PICK_OFFSET_NEAR_ROW = [-0.047, 0.0, 0.03]
LIFT_Z_OFFSET = 0.30
WEIGH_TRANSIT_IK_Z = 0.326217
WEIGH_RELEASE_IK = [0.40, -0.574, 0.146217]
# 换手 + 入箱（官方 handoff 常量）
RIGHT_HANDOFF_TO_LEFT_IK = [0.246, -0.044645, 0.3016983]
RIGHT_HANDOFF_TRANSIT_IK_Z = 0.40
LEFT_HANDOFF_RECEIVE_IK = [0.266, 0.04645, 0.2216983]
BOX_DROP_IK = [0.605966, 0.226114, 0.556217]
PLACE_APPROACH_Z_OFFSET = 0.060
# ===== scene2 零件分拣 =====
# 抓取位姿与官方 scene2_part_grasp_ik.py 对齐：yaw_adaptive 叠加在工件自身 yaw 上。
SCENE2_GRASP_PITCH_DEG = -45.0            # pitch_adaptive（官方 -pi/4）
SCENE2_YAW_MANUAL_DEG = 90.0              # yaw_manual（官方 pi/2；左手自动取反）
SCENE2_C_YAW_ADAPTIVE_DEG = -45.0         # C 螺丝刀额外 yaw_adaptive（官方 -pi/4）
SCENE2_NARROW_EDGE_OFFSET_DEG = 90.0      # 窄边自动选 ±90°（官方 GRASP_NARROW_EDGE_YAW_OFFSET）
SCENE2_PLACE_JOINTS = {
    # 关节空间放置位（官方原值；已弃用，改为 IK 笛卡尔放置 SCENE2_BIN_XY，保留作参考/回退）
    ("right", "A"): [-30, -10, -10, -80, 70, 0, 0],
    ("right", "B"): [-30, 0, 30, -80, 70, 0, 0],
    ("left", "B"): [-30, 0, -30, -80, -70, 0, 0],
    ("left", "C"): [-30, 10, 10, -80, -70, 0, 0],
}
# 各箱中心（base 系 x,y，官方 PRESET_PLACE_TARGETS 含 PLACE_TARGET_X_BIAS=-0.03）
SCENE2_BIN_XY = {"A": (0.535, -0.444), "B": (0.535, -0.014), "C": (0.535, 0.416)}
SCENE2_BIN_TARGET_ARM = {"A": "right", "B": None, "C": "left"}  # B 按零件 y 分左右手
WEIGH_REGRASP_IK = [0.40, -0.574, -0.04]
IK_MODE_POS_HARD_ORI_HARD = 0x03
IK_MODE_THREE_POINT_MIXED = 0x06
ARM_MODE_EXTERNAL_CONTROL = 2
ARM_MODE_SERVICE = "/arm_traj_change_mode"
# 官方参考实现的左手等待位（锁左臂用；若锁"当前FK"会导致右臂IK无解）
LEFT_PRESET_2_IK = [0.313, 0.239, 0.282]
LEFT_PRESET_2_FIRST_YPR = [-146.440, 4.966, 0.0]
LEFT_PRESET_2_SECOND_YPR = [0.0, 0.0, 96.580]
RIGHT_PICK_FIRST_YPR = [0, -90, 0.0]
RIGHT_PICK_SECOND_YPR = [90, 0.0, 0.0]
WEIGH_RELEASE_FIRST_YPR = [0, -90, 0.0]
WEIGH_RELEASE_SECOND_YPR = [90, 0.0, 0.0]
WEIGH_REGRASP_FIRST_YPR = [0, -60, 0.0]
WEIGH_REGRASP_SECOND_YPR = [90, 0.0, 0.0]
PRESET_FINAL_DEG = [29.89, 30.67, 29.889, -139.1, -59.33, 0, 0,
                    29.89, -30.67, -29.889, -139.1, 59.33, 0, 0]

# ---- 相机感知标定参数 ----
CAM_FX = 392.9
CAM_FY = 392.9
CAM_CX = 640.0
CAM_CY = 360.0
CAM_POS = [0.1187, 0.0, 0.6388]
CAM_QUAT = [0.6159, -0.6159, 0.3474, -0.3474]  # xyzw
CAM_X_CORRECTION = -0.057  # 标定得到的 x 系统偏移
PARCEL_PLANE_Z_BASE = -0.044
CAMR_FX = 623.5
CAMR_FY = 623.5
CAMR_CX = 640.0
CAMR_CY = 360.0
CAMR_FRAME = "Right wrist Camera View"


def _cross3(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def _rot_vec(v, qq):
    qv = [qq[0], qq[1], qq[2]]
    qw = qq[3]
    x = _cross3(qv, v)
    y = _cross3(qv, x)
    return [v[0]+2*qw*x[0]+2*y[0], v[1]+2*qw*x[1]+2*y[1], v[2]+2*qw*x[2]+2*y[2]]


def _pixel_to_base(u, v, zplane, cam_pos=None, cam_quat=None):
    if cam_pos is None:
        cam_pos = CAM_POS
    if cam_quat is None:
        cam_quat = CAM_QUAT
    dcam = [(u-CAM_CX)/CAM_FX, (v-CAM_CY)/CAM_FY, 1.0]
    dbase = _rot_vec(dcam, cam_quat)
    t = (zplane - cam_pos[2]) / dbase[2]
    return [cam_pos[0]+t*dbase[0], cam_pos[1]+t*dbase[1]]


def _pixel_depth_to_base(u, v, depth, cam_pos=None, cam_quat=None):
    """像素 + 深度图 -> base 3D 坐标（米）。depth 为 uint16 毫米数组。

    cam_pos/cam_quat 可选：头部相机当前位姿（TF 查询）；None 则用标定常量 CAM_POS/CAM_QUAT。
    """
    if depth is None:
        return None
    if cam_pos is None:
        cam_pos = CAM_POS
    if cam_quat is None:
        cam_quat = CAM_QUAT
    h, w = depth.shape[:2]
    iu, iv = int(round(u)), int(round(v))
    if iu < 1 or iv < 1 or iu >= w - 1 or iv >= h - 1:
        return None
    vals = []
    for dv in (-1, 0, 1):
        for du in (-1, 0, 1):
            d = float(depth[iv + dv, iu + du])
            if 0 < d < 4000:
                vals.append(d)
    if not vals:
        return None
    vals.sort()
    dmm = vals[len(vals) // 2]
    dm = dmm / 1000.0
    xc = (u - CAM_CX) / CAM_FX * dm
    yc = (v - CAM_CY) / CAM_FY * dm
    p = _rot_vec([xc, yc, dm], cam_quat)
    return [cam_pos[0] + p[0], cam_pos[1] + p[1], cam_pos[2] + p[2]]


def _get_head_cam_pose(timeout=3.0):
    """查询 TF 获取头部相机当前位姿（base_link -> Head Camera View）。

    低头后相机位姿会变，固定 CAM_POS/CAM_QUAT 会失准；返回 (pos, quat_xyzw)，失败返回 (None, None)。
    """
    import rospy
    try:
        import tf
        listener = tf.TransformListener()
        for frame in ("Head Camera View", "head_camera", "head_camera_view", "head_camera_optical_frame"):
            try:
                listener.waitForTransform("base_link", frame, rospy.Time(0), rospy.Duration(timeout))
                trans, rot = listener.lookupTransform("base_link", frame, rospy.Time(0))
                return list(trans), list(rot)
            except Exception:
                continue
    except Exception:
        pass
    return None, None


def detect_parcels(logfn):
    import rospy
    import cv2
    import numpy as np
    from sensor_msgs.msg import CompressedImage
    msg = None
    dmsg = None
    for _attempt in range(3):
        try:
            msg = rospy.wait_for_message('/cam_h/color/image_raw/compressed', CompressedImage, timeout=15)
            dmsg = rospy.wait_for_message('/cam_h/depth/image_raw/compressedDepth', CompressedImage, timeout=15)
            if msg is not None and dmsg is not None:
                break
        except Exception:
            rospy.sleep(1.0)
    if msg is None:
        return []
    arr = np.frombuffer(msg.data, np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    depth = None
    if dmsg is not None:
        try:
            depth = cv2.imdecode(np.frombuffer(dmsg.data[12:], np.uint8), cv2.IMREAD_UNCHANGED).astype(np.float32)
        except Exception:
            depth = None
    h, _ = img.shape[:2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = ((hsv[:, :, 2] > 165)).astype(np.uint8) * 255
    roi = np.zeros_like(mask)
    roi[int(h*0.25):, :] = 255
    mask = cv2.bitwise_and(mask, roi)
    k = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    n, _l, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
    found = []
    for i in range(1, n):
        x, y, bw, bh, area = stats[i]
        if area < 800 or area > 6000:
            continue
        # 用 bounding box 中心，而非亮斑质心（质心会被反光/标签的不均匀亮度拉偏）
        u = x + bw / 2.0
        v = y + bh / 2.0
        p3 = _pixel_depth_to_base(u, v, depth)
        if p3 is not None:
            bx, by, bz = p3[0], p3[1], p3[2]
            # 过滤桌面/手臂误检：包裹顶面 z 应在 -0.03 ~ 0.10（桌面以下或手臂高度均排除）
            if bz < -0.03 or bz > 0.10:
                continue
        else:
            bx, by = _pixel_to_base(u, v, PARCEL_PLANE_Z_BASE)
            bx += CAM_X_CORRECTION
            bz = None
        if bx < 0.0 or bx > 0.7:
            continue
        found.append((bx, by, bz))
    found.sort()
    logfn("scene1 detect: %d parcels %s", len(found),
          [(round(a, 3), round(b, 3), (round(c, 3) if c is not None else None)) for a, b, c in found])
    return found


def detect_scene2_parts(logfn):
    """scene2 检测：颜色分割 + 深度定位 -> parts=[(type,x,y,z),...], bins={type:(x,y,z)}。"""
    import rospy, cv2, numpy as np
    from sensor_msgs.msg import CompressedImage
    cm = None; dm = None
    for _a in range(3):
        try:
            cm = rospy.wait_for_message('/cam_h/color/image_raw/compressed', CompressedImage, timeout=15)
            dm = rospy.wait_for_message('/cam_h/depth/image_raw/compressedDepth', CompressedImage, timeout=15)
            if cm is not None and dm is not None:
                break
        except Exception:
            rospy.sleep(1.0)
    if cm is None:
        return [], {}
    color = cv2.imdecode(np.frombuffer(cm.data, np.uint8), cv2.IMREAD_COLOR)
    depth = cv2.imdecode(np.frombuffer(dm.data[12:], np.uint8), cv2.IMREAD_UNCHANGED).astype(np.float32)
    cam_pos, cam_quat = _get_head_cam_pose()
    if cam_pos is None:
        cam_pos, cam_quat = CAM_POS, CAM_QUAT
    h, w = color.shape[:2]
    hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    H = hsv[:,:,0]; S = hsv[:,:,1]; V = hsv[:,:,2]

    def to3d(u, v):
        p = _pixel_depth_to_base(u, v, depth, cam_pos, cam_quat)
        # 零件在桌面附近（z -0.25~0.15），排除胸前手臂（z>0.3）和地面
        if p is not None and -0.25 <= p[2] <= 0.15:
            return p
        return None

    def _blob_yaw(labels, label_id, zplane):
        """2D 图像 PCA 取主轴，沿主轴两端点反投影到 zplane 平面，算世界 yaw（弧度）。

        图像像素取主轴（不受深度噪声影响）；两端点用平面求交反投影，两点连线方向即工件长轴 yaw。
        """
        ys, xs = np.where(labels == label_id)
        if len(xs) < 8:
            return None
        pts = np.stack([xs.astype(np.float32), ys.astype(np.float32)], axis=1)
        mean, eigvec = cv2.PCACompute(pts, mean=None)
        v = eigvec[0]
        mu, mv = float(mean[0, 0]), float(mean[0, 1])
        L = 40.0
        p1 = _pixel_to_base(mu + v[0] * L, mv + v[1] * L, zplane, cam_pos, cam_quat)
        p2 = _pixel_to_base(mu - v[0] * L, mv - v[1] * L, zplane, cam_pos, cam_quat)
        dx, dy = p1[0] - p2[0], p1[1] - p2[1]
        n = math.hypot(dx, dy)
        if n < 1e-6:
            return None
        return math.atan2(dy, dx)

    def blobs(mask, area_lo, area_hi, min_aspect=None, max_aspect=None):
        k = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
        out = []
        for i in range(1, n):
            x, y, bw, bh, area = stats[i]
            if area < area_lo or area > area_hi:
                continue
            aspect = max(bw, bh) / float(max(min(bw, bh), 1))
            if min_aspect is not None and aspect < min_aspect:
                continue
            if max_aspect is not None and aspect > max_aspect:
                continue
            u, v = cents[i]
            p = to3d(u, v)
            if p is not None:
                out.append((p, int(area), _blob_yaw(labels, i, p[2])))
        return out

    red = ((H < 10) | (H > 170)) & (S > 80) & (V > 60)
    black = (V < 90) & (S < 100)
    silver = (S < 60) & (V > 120)

    parts = []
    # 组合 mask（红柄 + 银杆 = 完整螺丝刀），用于 C 朝向的 2D PCA
    _combined = red | silver
    _ck = np.ones((3, 3), np.uint8)
    _cm = cv2.morphologyEx(_combined.astype(np.uint8) * 255, cv2.MORPH_CLOSE, _ck)
    _cn, _clabels, _cstats, _ccents = cv2.connectedComponentsWithStats(_cm, 8)

    def _combined_screwdriver_yaw(u, v, zplane):
        """对「红柄+银杆」合成连通域做 2D PCA 取主轴，反投影到 zplane 平面算 yaw。"""
        iu, iv = int(round(u)), int(round(v))
        if not (0 <= iv < _clabels.shape[0] and 0 <= iu < _clabels.shape[1]):
            return None
        _cl = _clabels[iv, iu]
        if _cl == 0:
            return None
        ys, xs = np.where(_clabels == _cl)
        if len(xs) < 8:
            return None
        pts = np.stack([xs.astype(np.float32), ys.astype(np.float32)], axis=1)
        mean, eigvec = cv2.PCACompute(pts, mean=None)
        v = eigvec[0]
        mu, mv = float(mean[0, 0]), float(mean[0, 1])
        L = 60.0
        p1 = _pixel_to_base(mu + v[0] * L, mv + v[1] * L, zplane, cam_pos, cam_quat)
        p2 = _pixel_to_base(mu - v[0] * L, mv - v[1] * L, zplane, cam_pos, cam_quat)
        dx, dy = p1[0] - p2[0], p1[1] - p2[1]
        n = math.hypot(dx, dy)
        if n < 1e-6:
            return None
        # 反投影带常数 ~45° 旋转误差（C_0/C_5 实测偏移一致 45.4°），减去校准
        return math.atan2(dy, dx) - math.radians(45.0)

    # C 螺丝刀：单独检测红柄，朝向用组合连通域 PCA
    _rk = np.ones((5, 5), np.uint8)
    _rm = cv2.morphologyEx(red.astype(np.uint8) * 255, cv2.MORPH_OPEN, _rk)
    _rm = cv2.morphologyEx(_rm, cv2.MORPH_CLOSE, _rk)
    _rn, _rlabels, _rstats, _rcents = cv2.connectedComponentsWithStats(_rm, 8)
    for _i in range(1, _rn):
        if _rstats[_i][4] < 200 or _rstats[_i][4] > 4000:
            continue
        _u, _v = _rcents[_i]
        _p = to3d(_u, _v)
        if _p is None:
            continue
        _cy = _combined_screwdriver_yaw(_u, _v, _p[2])
        parts.append(("C", _p[0], _p[1], _p[2], _cy))
    for p, area, yaw in blobs(black.astype(np.uint8) * 255, 200, 4000):
        parts.append(("B", p[0], p[1], p[2], yaw))
    # A 银管件：area>=100（含小管件）、长宽比 <2.0（螺丝刀银颈 aspect 2.4~2.8 排除）。
    # 头部下看后 A 管件 blob 面积约 144~199；手臂灰色圆柱面积 6000+ 由 z 过滤排除。
    for p, area, yaw in blobs(silver.astype(np.uint8) * 255, 100, 10000, max_aspect=2.0):
        parts.append(("A", p[0], p[1], p[2], yaw))
    parts.sort(key=lambda t: (t[1], t[2]))
    logfn("scene2 detect: %d parts %s", len(parts),
          [(t, round(x, 3), round(y, 3), (round(math.degrees(yw), 1) if yw is not None else None)) for t, x, y, z, yw in parts])
    return parts, {}


def _matmul3(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _rot_mat_to_quat_xyzw(m):
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s; x = (m[2][1] - m[1][2]) / s; y = (m[0][2] - m[2][0]) / s; z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s; x = 0.25 * s; y = (m[0][1] + m[1][0]) / s; z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s; x = (m[0][1] + m[1][0]) / s; y = 0.25 * s; z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s; x = (m[0][2] + m[2][0]) / s; y = (m[1][2] + m[2][1]) / s; z = 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return [x / n, y / n, z / n, w / n]


def quat_from_ypr(first, second=None):
    yaw, pitch, _ = [math.radians(v) for v in first]
    cy, sy = math.cos(yaw), math.sin(yaw); cp, sp = math.cos(pitch), math.sin(pitch)
    m = [[cy * cp, -sy, cy * sp], [sy * cp, cy, sy * sp], [-sp, 0.0, cp]]
    if second is not None:
        my, mp, mr = [math.radians(v) for v in second]
        mm = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        if abs(my) > 0.01:
            c, s = math.cos(my), math.sin(my); mm = _matmul3([[c, -s, 0], [s, c, 0], [0, 0, 1]], mm)
        if abs(mp) > 0.01:
            c, s = math.cos(mp), math.sin(mp); mm = _matmul3([[c, 0, s], [0, 1, 0], [-s, 0, c]], mm)
        if abs(mr) > 0.01:
            c, s = math.cos(mr), math.sin(mr); mm = _matmul3([[1, 0, 0], [0, c, -s], [0, s, c]], mm)
        m = _matmul3(m, mm)
    return _rot_mat_to_quat_xyzw(m)

def _scene2_yaw_bias_deg(part_type):
    """官方 yaw_adaptive 偏置：仅 C 螺丝刀额外 -45°。"""
    return SCENE2_C_YAW_ADAPTIVE_DEG if part_type == "C" else 0.0


def _scene2_tool_forward_x(part_type, object_yaw_rad, active_arm, narrow_deg):
    """官方 _tool_forward_x_score：夹爪前进方向（局部 -Z）在世界系 x 分量。

    grasp 姿态 pitch_manual=roll_manual=0，故 forward_x ∝ cos(yaw_adaptive)，等价于 cos(yaw_total)。
    """
    yaw_total_rad = object_yaw_rad + math.radians(_scene2_yaw_bias_deg(part_type) + narrow_deg)
    return math.cos(yaw_total_rad)


def _scene2_select_narrow_edge(part_type, object_yaw_rad, active_arm):
    """官方 _select_narrow_edge_yaw_offset：在 ±90° 里选让夹爪朝机器人前方(+x)的那一侧。"""
    return max((SCENE2_NARROW_EDGE_OFFSET_DEG, -SCENE2_NARROW_EDGE_OFFSET_DEG),
               key=lambda off: _scene2_tool_forward_x(part_type, object_yaw_rad, active_arm, off))


def _scene2_should_flip_narrow_edge(part_type, object_yaw_rad, active_arm, object_y):
    """官方 _part_should_grasp_other_narrow_edge_by_axis（yaw-only 近似）。

    A/B 按工件局部 Y 轴在世界 xy 的方向判断是否翻到窄边另一头；C 仅右手在 y∈[-0.3,0] 有特殊规则。
    """
    if part_type == "C":
        if active_arm == "right" and -0.3 <= object_y <= 0.0:
            c, s = math.cos(object_yaw_rad), math.sin(object_yaw_rad)
            vx, vy, vz = -c, -s, 1.0  # 局部 [-1,0,1] 轴经 yaw 旋转
            n = math.sqrt(vx * vx + vy * vy + vz * vz)
            if n <= 1e-9:
                return False
            return (-vy / n) >= math.cos(math.radians(20.0))
        return False
    if part_type == "A" and -0.2 <= object_y <= 0.1:
        tx, ty, thr = 1.0, 1.0, 45.0
    elif part_type == "B" and 0.0 <= object_y <= 0.2:
        tx, ty, thr = 1.0, -1.0, 45.0
    elif part_type == "B" and -0.2 <= object_y < 0.0:
        tx, ty, thr = 1.0, 1.0, 45.0
    else:
        return False
    ax, ay = -math.sin(object_yaw_rad), math.cos(object_yaw_rad)
    n = math.hypot(ax, ay)
    tn = math.hypot(tx, ty)
    if n <= 1e-9:
        return True
    cosang = abs((ax * tx + ay * ty) / (n * tn))
    return cosang < math.cos(math.radians(thr))


def scene2_grasp_quat(part_type, object_yaw_rad, active_arm, object_y=None):
    """构造 scene2 抓取四元数（xyzw），与官方 get_object_grasp_quat_xyzw 对齐。

    object_yaw_rad: 相机检测到的工件自身 yaw（绕 base z，弧度）。
    active_arm: 'left' / 'right'（左手 yaw_manual 取反）。
    object_y: 工件 world/base y，用于窄边翻转启发；None 则跳过翻转。
    """
    narrow_deg = _scene2_select_narrow_edge(part_type, object_yaw_rad, active_arm)
    if object_y is not None and _scene2_should_flip_narrow_edge(part_type, object_yaw_rad, active_arm, object_y):
        narrow_deg = -narrow_deg
    yaw_manual_deg = SCENE2_YAW_MANUAL_DEG if active_arm == "right" else -SCENE2_YAW_MANUAL_DEG
    yaw_total_deg = math.degrees(object_yaw_rad) + _scene2_yaw_bias_deg(part_type) + narrow_deg
    return quat_from_ypr([yaw_total_deg, SCENE2_GRASP_PITCH_DEG, 0.0],
                         [yaw_manual_deg, 0.0, 0.0])


def publish_head_down():
    """官方 _publish_head_target：头部下看 20°，让近端零件进入头部相机视野。

    scene2 检测前必须调用，否则近端零件被画面下边缘裁切，检测会漏件。
    """
    import rospy
    from kuavo_msgs.msg import robotHeadMotionData
    pub = rospy.Publisher("/robot_head_motion_data", robotHeadMotionData, queue_size=10)
    rospy.sleep(0.5)
    msg = robotHeadMotionData()
    msg.joint_data = [0.0, 20.0]
    for _ in range(5):
        pub.publish(msg)
        rospy.sleep(0.1)
    rospy.sleep(0.4)


class ArmHold:
    def __init__(self, pub, init_deg):
        self._pub = pub
        self._lock = threading.Lock()
        self._deg = [float(v) for v in init_deg]
        self._stop = threading.Event()
        self._thr = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thr.start()

    def set_degrees(self, deg):
        with self._lock:
            self._deg = [float(v) for v in deg]

    def _run(self):
        import rospy
        from sensor_msgs.msg import JointState
        r = rospy.Rate(ARM_TRAJ_HZ)
        while not self._stop.is_set() and not rospy.is_shutdown():
            with self._lock:
                deg = list(self._deg)
            m = JointState(); m.header.stamp = rospy.Time.now(); m.name = ARM_JOINT_NAMES; m.position = deg
            try:
                self._pub.publish(m); r.sleep()
            except rospy.ROSException:
                break


class GripperHold:
    """持续发布夹爪命令（对齐官方 GripperHold），避免发几次就停导致闭合不稳定。"""
    def __init__(self, pub, left, right):
        self._pub = pub
        self._lock = threading.Lock()
        self._left = float(left)
        self._right = float(right)
        self._stop = threading.Event()
        self._thr = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thr.start()

    def set_command(self, left, right):
        with self._lock:
            self._left = float(left)
            self._right = float(right)

    def _run(self):
        import rospy
        from sensor_msgs.msg import JointState
        r = rospy.Rate(20.0)
        while not self._stop.is_set() and not rospy.is_shutdown():
            with self._lock:
                left = self._left
                right = self._right
            m = JointState(); m.header.stamp = rospy.Time.now()
            m.name = ["left_gripper_joint", "right_gripper_joint"]
            m.position = [left, right]
            try:
                self._pub.publish(m); r.sleep()
            except rospy.ROSException:
                break


class Scene1Runner:
    def __init__(self):
        import rospy
        from sensor_msgs.msg import JointState
        self.arm_pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)
        self.gripper_pub = rospy.Publisher("/gripper/command", JointState, queue_size=10)
        self.gripper_hold = GripperHold(self.gripper_pub, GRIP_OPEN, GRIP_OPEN)
        self.gripper_hold.start()
        self.right_pick_quat = quat_from_ypr(RIGHT_PICK_FIRST_YPR, RIGHT_PICK_SECOND_YPR)
        self.weigh_release_quat = quat_from_ypr(WEIGH_RELEASE_FIRST_YPR, WEIGH_RELEASE_SECOND_YPR)
        self.weigh_regrasp_quat = quat_from_ypr(WEIGH_REGRASP_FIRST_YPR, WEIGH_REGRASP_SECOND_YPR)
        self.log = rospy.loginfo

    # ---- 基础原语 ----
    def read_current_arm_rad(self, timeout=5.0):
        import rospy
        from kuavo_msgs.msg import sensorsData
        msg = rospy.wait_for_message("/sensors_data_raw", sensorsData, timeout=timeout)
        q = list(msg.joint_data.joint_q)
        if len(q) >= 27:
            return q[13:27]
        if len(q) >= 26:
            return q[12:26]
        raise RuntimeError("joint_q len %d" % len(q))

    def move_arm_deg(self, arm_hold, target_deg, move_time=MOVE_TIME, settle=SETTLE, lock_left=False):
        import rospy
        cur_rad = self.read_current_arm_rad()
        start = [math.degrees(v) for v in cur_rad]
        target = [float(v) for v in target_deg]
        if lock_left:
            # 严格锁左臂：左手 7 关节保持 start 值，只插值右手
            target[:7] = start[:7]
        steps = max(1, int(round(move_time * ARM_TRAJ_HZ)))
        rate = rospy.Rate(ARM_TRAJ_HZ)
        for i in range(steps + 1):
            a = float(i) / steps
            arm_hold.set_degrees([start[k] + (target[k] - start[k]) * a for k in range(14)])
            if i < steps:
                rate.sleep()
        rospy.sleep(settle)

    def set_gripper(self, left, right, hold_time=0.0):
        """更新持续 hold 的命令值；hold_time>0 时等待该时长（闭爪保持）。"""
        self.gripper_hold.set_command(left, right)
        if hold_time > 0:
            import rospy
            rospy.sleep(hold_time)

    def wait_gripper(self, side, target, tolerance=0.05, timeout=3.0):
        """读 /gripper/state 确认夹爪到达目标（对齐官方 _wait_for_gripper_open）。"""
        import rospy
        from sensor_msgs.msg import JointState
        name = side + "_gripper_joint"
        deadline = rospy.Time.now() + rospy.Duration(float(timeout))
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            try:
                msg = rospy.wait_for_message("/gripper/state", JointState, timeout=0.2)
            except rospy.ROSException:
                continue
            if name not in msg.name:
                continue
            i = msg.name.index(name)
            if i >= len(msg.position):
                continue
            if abs(float(msg.position[i]) - float(target)) <= tolerance:
                return True
        return False

    def set_arm_mode(self, mode=ARM_MODE_EXTERNAL_CONTROL, timeout=10.0):
        import rospy
        from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest
        rospy.wait_for_service(ARM_MODE_SERVICE, timeout=timeout)
        proxy = rospy.ServiceProxy(ARM_MODE_SERVICE, changeArmCtrlMode)
        req = changeArmCtrlModeRequest()
        req.control_mode = mode
        resp = proxy(req)
        if not resp.result:
            raise RuntimeError("arm mode %s rejected: %s" % (mode, resp.message))
        self.log("scene1: arm mode -> %s OK (%s)", mode, resp.message)

    def read_gripper_state(self, side="right"):
        import rospy
        from sensor_msgs.msg import JointState
        name = side + "_gripper_joint"
        try:
            msg = rospy.wait_for_message("/gripper/state", JointState, timeout=3.0)
            for i, n in enumerate(msg.name):
                if n == name and i < len(msg.position):
                    return float(msg.position[i])
        except Exception:
            pass
        return None

    def save_head_snapshot(self, path, retries=3):
        import rospy
        import cv2
        import numpy as np
        from sensor_msgs.msg import CompressedImage
        for attempt in range(retries):
            try:
                m = rospy.wait_for_message("/cam_h/color/image_raw/compressed", CompressedImage, timeout=5.0)
                a = np.frombuffer(m.data, np.uint8)
                img = cv2.imdecode(a, cv2.IMREAD_COLOR)
                if img is not None:
                    cv2.imwrite(path, img)
                    self.log("scene1 snapshot saved %s" % path)
                    return
            except Exception as e:
                if attempt == retries - 1:
                    self.log("scene1 snapshot failed %s" % e)
                else:
                    rospy.sleep(1.0)

    def measure_weigh_center_3d(self):
        """深度图实测称重区中心 base 3D 坐标（修正几何计算的偏差）。"""
        import rospy, cv2, numpy as np
        from sensor_msgs.msg import CompressedImage
        color_msg = None
        depth_msg = None
        for _attempt in range(3):
            try:
                color_msg = rospy.wait_for_message('/cam_h/color/image_raw/compressed', CompressedImage, timeout=6)
                depth_msg = rospy.wait_for_message('/cam_h/depth/image_raw/compressedDepth', CompressedImage, timeout=6)
                if color_msg is not None and depth_msg is not None:
                    break
            except Exception:
                rospy.sleep(1.0)
        if color_msg is None or depth_msg is None:
            return None
        color = cv2.imdecode(np.frombuffer(color_msg.data, np.uint8), cv2.IMREAD_COLOR)
        depth = cv2.imdecode(np.frombuffer(depth_msg.data[12:], np.uint8), cv2.IMREAD_UNCHANGED).astype(np.float32)
        h, w = color.shape[:2]
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        H = hsv[:,:,0]; S = hsv[:,:,1]; V = hsv[:,:,2]
        roi = np.zeros((h, w), np.uint8); roi[int(h*0.5):, int(w*0.5):] = 255
        green = ((H>=40)&(H<=85)&(S>60)&(V>120)).astype(np.uint8)*255
        green = cv2.bitwise_and(green, roi)
        n, labels, stats, cents = cv2.connectedComponentsWithStats(green, 8)
        best = None; best_area = 0
        for i in range(1, n):
            if stats[i][4] > best_area:
                best_area = stats[i][4]; best = i
        if best is None:
            return None
        # 连通域内深度中位数（稳定，抗单像素噪声）+ 质心
        zone = (labels == best)
        ds = depth[zone]
        ds = ds[(ds > 0) & (ds < 4000)]
        if len(ds) < 10:
            return None
        d = float(np.median(ds))
        u = float(cents[best][0]); v = float(cents[best][1])
        dm = d / 1000.0
        xc = (u - CAM_CX) / CAM_FX * dm
        yc = (v - CAM_CY) / CAM_FY * dm
        zc = dm
        pcam = np.array([xc, yc, zc])
        qx, qy, qz, qw = CAM_QUAT
        R = np.array([
            [1-2*qy*qy-2*qz*qz, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
            [2*qx*qy+2*qz*qw, 1-2*qx*qx-2*qz*qz, 2*qy*qz-2*qx*qw],
            [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx*qx-2*qy*qy],
        ])
        pbase = np.array(CAM_POS) + R @ pcam
        # 合理性校验：称重区应在右前下方，排除误检到的绿色物体
        if not (0.30 <= pbase[0] <= 0.58 and -0.78 <= pbase[1] <= -0.42):
            return None
        return [float(pbase[0]), float(pbase[1]), float(pbase[2])]

    def call_ik(self, left_pos, left_quat, right_pos, right_quat, current_rad, timeout=5.0,
                constraint_mode=IK_MODE_THREE_POINT_MIXED):
        import rospy
        from kuavo_msgs.msg import twoArmHandPoseCmd, ikSolveParam
        from kuavo_msgs.srv import twoArmHandPoseCmdSrv
        req = twoArmHandPoseCmd()
        p = ikSolveParam()
        p.major_optimality_tol = 1e-3; p.major_feasibility_tol = 1e-3; p.minor_feasibility_tol = 1e-3
        p.major_iterations_limit = 100; p.oritation_constraint_tol = 1e-3; p.pos_constraint_tol = 1e-3
        p.pos_cost_weight = 1.0; p.constraint_mode = constraint_mode
        req.ik_param = p
        req.use_custom_ik_param = True
        req.joint_angles_as_q0 = True
        req.hand_poses.left_pose.joint_angles = list(current_rad[:7])
        req.hand_poses.right_pose.joint_angles = list(current_rad[7:])
        req.hand_poses.left_pose.elbow_pos_xyz = [0.0, 0.0, 0.0]
        req.hand_poses.right_pose.elbow_pos_xyz = [0.0, 0.0, 0.0]
        req.hand_poses.left_pose.pos_xyz = list(left_pos)
        req.hand_poses.left_pose.quat_xyzw = list(left_quat)
        req.hand_poses.right_pose.pos_xyz = list(right_pos)
        req.hand_poses.right_pose.quat_xyzw = list(right_quat)
        rospy.wait_for_service("/ik/two_arm_hand_pose_cmd_srv", timeout=timeout)
        resp = rospy.ServiceProxy("/ik/two_arm_hand_pose_cmd_srv", twoArmHandPoseCmdSrv)(req)
        if not resp.success:
            raise RuntimeError("IK failed: %s" % getattr(resp, "error_reason", ""))
        q = list(resp.q_arm)
        if len(q) < 14:
            l = list(resp.hand_poses.left_pose.joint_angles); r = list(resp.hand_poses.right_pose.joint_angles)
            if len(l) == 7 and len(r) == 7:
                q = l + r
        if len(q) < 14:
            raise RuntimeError("IK returned %d joints" % len(q))
        return q[:14]  # rad

    def call_fk(self, joint_rad, timeout=10.0):
        import rospy
        from kuavo_msgs.srv import fkSrv
        rospy.wait_for_service("/ik/fk_srv", timeout=timeout)
        resp = rospy.ServiceProxy("/ik/fk_srv", fkSrv)(list(joint_rad))
        if not resp.success:
            raise RuntimeError("fk failed")
        return resp.hand_poses

    # 右手到目标位姿（左手锁官方等待位，避免联合IK无解）
    def right_arm_to(self, arm_hold, ik_pos, quat,
                     constraint_mode=IK_MODE_THREE_POINT_MIXED, lock_left_preset=True):
        cur = self.read_current_arm_rad()
        if lock_left_preset:
            if not hasattr(self, "_left_quat_fixed"):
                self._left_quat_fixed = quat_from_ypr(
                    LEFT_PRESET_2_FIRST_YPR, LEFT_PRESET_2_SECOND_YPR)
            left_pos = LEFT_PRESET_2_IK
            left_quat = self._left_quat_fixed
        else:
            fk = self.call_fk(cur)
            left_pos = list(fk.left_pose.pos_xyz)
            left_quat = list(fk.left_pose.quat_xyzw)
        q = self.call_ik(
            left_pos, left_quat, ik_pos, quat, cur,
            constraint_mode=constraint_mode)
        # 左手保持当前
        cur_deg = [math.degrees(v) for v in cur]
        q_deg = [math.degrees(v) for v in q]
        target = cur_deg[:7] + q_deg[7:14]
        self.move_arm_deg(arm_hold, target, lock_left=True)
        self.wait_converge(3.0)

    def right_grasp_to(self, arm_hold, ik_pos, quat):
        """最终下探优先使用位姿硬约束；无解时保留原混合约束回退。"""
        try:
            self.right_arm_to(
                arm_hold, ik_pos, quat,
                constraint_mode=IK_MODE_POS_HARD_ORI_HARD,
                lock_left_preset=True)
        except RuntimeError as exc:
            self.log("scene1 hard grasp IK failed, fallback to mixed IK: %s" % exc)
            self.right_arm_to(
                arm_hold, ik_pos, quat,
                constraint_mode=IK_MODE_THREE_POINT_MIXED,
                lock_left_preset=True)

    def left_arm_to(self, arm_hold, ik_pos, quat,
                    constraint_mode=IK_MODE_THREE_POINT_MIXED):
        """左手到目标位姿（右手锁当前 FK，避免联合 IK 无解）。"""
        cur = self.read_current_arm_rad()
        fk = self.call_fk(cur)
        right_pos = list(fk.right_pose.pos_xyz)
        right_quat = list(fk.right_pose.quat_xyzw)
        q = self.call_ik(
            ik_pos, quat, right_pos, right_quat, cur,
            constraint_mode=constraint_mode)
        cur_deg = [math.degrees(v) for v in cur]
        q_deg = [math.degrees(v) for v in q]
        target = q_deg[:7] + cur_deg[7:14]  # 左手新解 + 右手保持当前
        self.move_arm_deg(arm_hold, target, lock_left=False)
        self.wait_converge(3.0)

    def left_grasp_to(self, arm_hold, ik_pos, quat):
        """左手最终下探优先位姿硬约束；无解时回退混合约束（对称 right_grasp_to）。"""
        try:
            self.left_arm_to(
                arm_hold, ik_pos, quat,
                constraint_mode=IK_MODE_POS_HARD_ORI_HARD)
        except RuntimeError as exc:
            self.log("scene2 left hard grasp IK failed, fallback to mixed IK: %s" % exc)
            self.left_arm_to(
                arm_hold, ik_pos, quat,
                constraint_mode=IK_MODE_THREE_POINT_MIXED)

    def handoff_to_left_and_box(self, arm_hold, name):
        """换手入箱：右手把包裹送到交接点 -> 左手接 -> 左手入箱。"""
        import rospy
        hq = quat_from_ypr([-0.839, -100.0, 0.0], [90.0, -20.0, 90.0])
        rq = quat_from_ypr([-0.839, -100.0, 0.0], [-90.0, -0.0, -90.0])
        bq = quat_from_ypr([-0.328, -100.935, 0.0], [-90.0, 0.0, 0.369])
        # 1. 右手送到交接点（先中转高度再下降）
        self.right_arm_to(arm_hold,
                          [RIGHT_HANDOFF_TO_LEFT_IK[0], RIGHT_HANDOFF_TO_LEFT_IK[1], RIGHT_HANDOFF_TRANSIT_IK_Z], hq)
        self.right_arm_to(arm_hold, RIGHT_HANDOFF_TO_LEFT_IK, hq)
        self.log("scene1 %s handoff: 右手到交接点", name)
        # 2. 左手到接应点
        self.left_arm_to(arm_hold, LEFT_HANDOFF_RECEIVE_IK, rq)
        self.log("scene1 %s handoff: 左手到接应点", name)
        # 3. 换手：左手夹住 -> 右手松开
        self.set_gripper(GRIP_CLOSE, GRIP_CLOSE)
        rospy.sleep(0.6)
        self.set_gripper(GRIP_CLOSE, GRIP_OPEN)
        rospy.sleep(0.8)
        self.log("scene1 %s handoff: 左手已接", name)
        try:
            self.save_head_snapshot("/tmp/handoff_%s.png" % name)
        except Exception:
            pass
        try:
            _fk = self.call_fk(self.read_current_arm_rad(20), 8.0)
            _lp = list(_fk.left_pose.pos_xyz); _rp = list(_fk.right_pose.pos_xyz)
            self.log("scene1 %s handoff后 左手FK=(%.3f,%.3f,%.3f) 右手FK=(%.3f,%.3f,%.3f)",
                     name, _lp[0], _lp[1], _lp[2], _rp[0], _rp[1], _rp[2])
        except Exception:
            pass
        # 4. 右手退让抬升
        self.right_arm_to(arm_hold,
                          [RIGHT_HANDOFF_TO_LEFT_IK[0], RIGHT_HANDOFF_TO_LEFT_IK[1], RIGHT_HANDOFF_TRANSIT_IK_Z], hq)
        # 5. 左手入箱（接近 -> 下放 -> 松开）
        self.left_arm_to(arm_hold,
                         [BOX_DROP_IK[0], BOX_DROP_IK[1], BOX_DROP_IK[2] + PLACE_APPROACH_Z_OFFSET], bq)
        self.left_arm_to(arm_hold, BOX_DROP_IK, bq)
        self.set_gripper(GRIP_OPEN, GRIP_OPEN)
        rospy.sleep(1.0)
        self.log("scene1 %s box drop done", name)
        # 入箱后拍快照（验证包裹是否进箱）
        try:
            self.save_head_snapshot("/tmp/box_drop_%s.png" % name)
        except Exception:
            pass
        # 6. 双臂回预设位（让出头部相机视野，否则下一轮检测不到包裹）
        try:
            self.move_arm_deg(arm_hold, PRESET_FINAL_DEG, move_time=2.0, settle=1.0)
            self.set_gripper(GRIP_OPEN, GRIP_OPEN)
            self.log("scene1 %s 手臂已归位", name)
        except Exception as _re:
            self.log("scene1 %s 手臂归位失败(忽略): %s", name, str(_re)[:60])

    def move_right_cartesian_to(self, arm_hold, target_ik, quat, n_points=6,
                                constraint_mode=IK_MODE_POS_HARD_ORI_HARD, seg_time=0.4):
        """笛卡尔直线分段运动（对齐官方 _move_right_cartesian_to）：末端沿直线分 n 段，逐点单手 IK，样条式连续下发。"""
        import rospy
        cur = self.read_current_arm_rad()
        fk = self.call_fk(cur, 5.0)
        start = list(fk.right_pose.pos_xyz)
        left_pos = list(fk.left_pose.pos_xyz)
        left_quat = list(fk.left_pose.quat_xyzw)
        end = [float(v) for v in target_ik]
        warm = cur
        left_deg = [math.degrees(v) for v in cur[:7]]
        joint_wps = [[math.degrees(v) for v in cur]]
        for k in range(1, int(n_points) + 1):
            a = float(k) / float(n_points)
            pt = [start[i] + (end[i] - start[i]) * a for i in range(3)]
            try:
                q = self.call_ik(left_pos, left_quat, pt, quat, warm, constraint_mode=constraint_mode)
            except RuntimeError:
                q = self.call_ik(left_pos, left_quat, pt, quat, warm, constraint_mode=IK_MODE_THREE_POINT_MIXED)
            q_deg = [math.degrees(v) for v in q]
            joint_wps.append(left_deg + q_deg[7:14])
            warm = q
        # 连续下发（段间线性插值，不停顿），对齐官方样条流式下发
        rate = rospy.Rate(ARM_TRAJ_HZ)
        for i in range(1, len(joint_wps)):
            steps = max(1, int(round(seg_time * ARM_TRAJ_HZ)))
            for s in range(steps + 1):
                a = float(s) / steps
                interp = [joint_wps[i-1][j] + (joint_wps[i][j] - joint_wps[i-1][j]) * a for j in range(14)]
                arm_hold.set_degrees(interp)
                if s < steps:
                    rate.sleep()
        rospy.sleep(0.8)  # 官方 SCENE1_ARM_SETTLE_TIME
        self.wait_converge(3.0)

    def world_to_ik(self, world):
        return [world[i] + WORLD_TO_IK_OFFSET[i] for i in range(3)]

    def perceive_parcel_ik_xy(self):
        # 第一版：返回基准位置（感知后续补），只取 x/y，z 用固定抓取高度
        return None

    def _grasp_offset(self, y):
        if y > RIGHT_PICK_NEAR_FAR_Y_THRESHOLD:
            return RIGHT_PICK_OFFSET_NEAR_ROW
        return RIGHT_PICK_OFFSET_FAR_ROW

    def wait_converge(self, timeout=3.0):
        import rospy
        prev = None
        stable = 0
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while rospy.Time.now() < deadline:
            try:
                q = self.read_current_arm_rad(2.0)
                fk = self.call_fk(q, 2.0)
                pos = list(fk.right_pose.pos_xyz)
            except Exception:
                rospy.sleep(0.1)
                continue
            if prev is not None:
                d = math.sqrt(sum((pos[i] - prev[i]) ** 2 for i in range(3)))
                if d < 0.002:
                    stable += 1
                else:
                    stable = 0
                if stable >= 3:
                    return True
            prev = pos
            rospy.sleep(0.1)
        return False

    def wrist_parcel_base(self):
        import rospy
        import cv2
        import numpy as np
        import tf
        from sensor_msgs.msg import CompressedImage
        try:
            m = rospy.wait_for_message('/cam_r/color/image_raw/compressed', CompressedImage, timeout=8)
        except Exception as e:
            self.log("wrist cam timeout, skip visual-servo: %s", e)
            return None
        img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
        h, _ = img.shape[:2]
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = (hsv[:, :, 2] > 165).astype(np.uint8) * 255
        k = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        n, _l, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
        best = None
        best_area = 0
        for i in range(1, n):
            area = stats[i][4]
            if area > best_area:
                best_area = area
                best = cents[i]
        if best is None:
            return None
        u, v = best
        lst = tf.TransformListener()
        try:
            lst.waitForTransform('base_link', CAMR_FRAME, rospy.Time(0), rospy.Duration(3))
            pos, quat = lst.lookupTransform('base_link', CAMR_FRAME, rospy.Time(0))
        except Exception as e:
            self.log("wrist TF fail: %s", e)
            return None
        dcam = [(u - CAMR_CX) / CAMR_FX, (v - CAMR_CY) / CAMR_FY, 1.0]
        dbase = _rot_vec(dcam, quat)
        t = (PARCEL_PLANE_Z_BASE - pos[2]) / dbase[2]
        bx = pos[0] + t * dbase[0]
        by = pos[1] + t * dbase[1]
        self.log("wrist parcel base=(%.3f, %.3f) uv=(%.0f, %.0f) area=%d", bx, by, u, v, best_area)
        return [bx, by]

    def run_parcel(self, arm_hold, name, ik_xy, z_top=None):
        offset = self._grasp_offset(ik_xy[1])
        px = ik_xy[0] + offset[0]
        py = ik_xy[1] + offset[1]
        if z_top is not None:
            # 用深度实测顶面高度，避免平面假设带来的 z 偏差
            pz = z_top - GRASP_BELOW_TOP + offset[2]
        else:
            pz = RIGHT_PICK_IK_Z + offset[2]
        # 预抓取（高位对齐）
        pre = [px, py, RIGHT_PICK_TRANSIT_IK_Z]
        self.right_arm_to(arm_hold, pre, self.right_pick_quat)
        # 诊断：第一个包裹拍头部图看手臂是否到位
        if not getattr(self, "_head_snap_done", False):
            self._head_snap_done = True
            self.save_head_snapshot("/tmp/pregrasp_head.png")
        # 视觉伺服：腕相机检测快递，校正抓取 x/y
        # 注意：深度定位准确时不再用腕相机（它容易误检到已搬走的包裹，反而把坐标带偏）
        wb = None if z_top is not None else self.wrist_parcel_base()
        if wb is not None:
            dx = wb[0] - px
            dy = wb[1] - py
            if abs(dx) > 0.03 or abs(dy) > 0.03:
                self.log("scene1 %s wrist detect outlier (dx=%.3f dy=%.3f), ignored", name, dx, dy)
                dx = 0.0
                dy = 0.0
            if abs(dx) > 0.005 or abs(dy) > 0.005:
                px += dx
                py += dy
                pre = [px, py, RIGHT_PICK_TRANSIT_IK_Z]
                self.right_arm_to(arm_hold, pre, self.right_pick_quat)
                self.log("scene1 %s visual-servo corrected to (%.3f, %.3f)", name, px, py)
        # 下降抓取
        grasp = [px, py, pz]
        self.right_grasp_to(arm_hold, grasp, self.right_pick_quat)
        # 抓取前记录夹爪状态（确认下探时是张开的）
        try:
            _g0 = self.read_gripper_state()
            self.log("scene1 %s PRE_GRASP gripper=%s", name, str(round(_g0, 4)) if _g0 is not None else "None")
        except Exception:
            pass
        # FK 闭环：实测末端位置与目标的偏差，阻尼迭代修正（只补 60% 防震荡）
        for _it in range(3):
            try:
                _fk = self.call_fk(self.read_current_arm_rad(20), 8.0)
                _p = list(_fk.right_pose.pos_xyz)
            except Exception as _e:
                self.log("scene1 %s FK check fail: %s", name, _e)
                break
            ex = grasp[0] - _p[0]; ey = grasp[1] - _p[1]; ez = grasp[2] - _p[2]
            _e3 = math.sqrt(ex * ex + ey * ey + ez * ez)
            self.log("scene1 %s FK iter%d actual=(%.3f,%.3f,%.3f) target=(%.3f,%.3f,%.3f) err=%.1fmm",
                     name, _it, _p[0], _p[1], _p[2], grasp[0], grasp[1], grasp[2], 1000.0 * _e3)
            if _e3 < 0.008:
                break
            # 阻尼修正：只补 60%，避免 overshoot 震荡
            self.right_grasp_to(arm_hold,
                                [grasp[0] + 0.6 * ex, grasp[1] + 0.6 * ey, grasp[2] + 0.6 * ez],
                                self.right_pick_quat)
        try:
            self.save_head_snapshot("/tmp/grasp_at_%s.png" % name)
        except Exception:
            pass
        self.set_gripper(GRIP_OPEN, GRIP_CLOSE)  # 右手闭
        import rospy; rospy.sleep(1.0)
        g = self.read_gripper_state()
        self.log("scene1 %s GRASP_CHECK right_gripper=%s (0.8 means empty grasp)" % (name, str(round(g, 4)) if g is not None else "None"))
        # 抓空自校正：抬高一点重试（应对下探过深、指尖滑到盒底以下）
        for _try in range(3):
            if g is None or g < 0.55:
                break
            self.set_gripper(GRIP_OPEN, GRIP_OPEN)
            import rospy as _rp; _rp.sleep(0.8)
            # 重新视觉检测：首次下探可能把盒子推走了（真正的视觉闭环）
            _best = None; _bd = 1e9
            try:
                for _c in detect_parcels(self.log):
                    _cz = _c[2] if len(_c) > 2 else None
                    # 过滤桌面/地面误检（z 明显低于包裹顶面，包裹顶面约 0~0.06m）
                    if _cz is not None and _cz < -0.03:
                        continue
                    _d = math.sqrt((_c[0] - grasp[0]) ** 2 + (_c[1] - grasp[1]) ** 2)
                    if _d < _bd:
                        _bd = _d; _best = _c
            except Exception as _de:
                self.log("scene1 %s re-detect fail: %s" % (name, _de))
            if _best is not None and _bd < 0.10 and _best[2] is not None:
                _retry = [_best[0], _best[1], _best[2] - GRASP_BELOW_TOP + 0.012]
                self.log("scene1 %s retry#%d box moved %.0fmm -> re-grasp (%.3f,%.3f)"
                         % (name, _try + 1, _bd * 1000.0, _retry[0], _retry[1]))
            else:
                _retry = [grasp[0], grasp[1], grasp[2] + 0.012]
                self.log("scene1 %s retry#%d no valid re-detect, z+0.012" % (name, _try + 1))
            try:
                self.right_arm_to(arm_hold, [_retry[0], _retry[1], RIGHT_PICK_TRANSIT_IK_Z], self.right_pick_quat)
                self.right_grasp_to(arm_hold, _retry, self.right_pick_quat)
                self.set_gripper(GRIP_OPEN, GRIP_CLOSE)
                _rp.sleep(1.0)
                g = self.read_gripper_state()
                self.log("scene1 %s retry#%d GRASP=%s" % (name, _try + 1, str(round(g, 4)) if g is not None else "None"))
            except Exception as _re:
                self.log("scene1 %s retry#%d fail: %s", name, _try + 1, _re)
                break
        # 抬升
        lift = [grasp[0], grasp[1], grasp[2] + LIFT_Z_OFFSET]
        self.right_arm_to(arm_hold, lift, self.right_pick_quat)
        # 深度图实测称重区 3D，动态补偿释放点（修正几何偏差）
        wc = self.measure_weigh_center_3d()
        if wc is not None:
            release = [wc[0], wc[1], WEIGH_RELEASE_IK[2]]
            self.log("scene1 %s depth-servo weigh center=(%.3f,%.3f)", name, wc[0], wc[1])
        else:
            release = list(WEIGH_RELEASE_IK)
            self.log("scene1 %s depth-servo failed, use geometry release", name)
        # 移到称重区上方
        transit = [release[0], release[1], WEIGH_TRANSIT_IK_Z]
        self.right_arm_to(arm_hold, transit, self.weigh_release_quat)
        # 下降释放
        self.right_arm_to(arm_hold, release, self.weigh_release_quat)
        rospy.sleep(1.5)
        self.set_gripper(GRIP_OPEN, GRIP_OPEN)  # 开爪称重
        rospy.sleep(1.0)
        self.save_head_snapshot("/tmp/weigh_" + name + ".png")
        # 二次抓取（称重后重新夹起）
        self.right_arm_to(arm_hold, WEIGH_REGRASP_IK, self.weigh_regrasp_quat)
        self.set_gripper(GRIP_OPEN, GRIP_CLOSE)
        rospy.sleep(1.0)
        g2 = self.read_gripper_state()
        self.log("scene1 %s regrasp GRASP=%s", name, str(round(g2, 4)) if g2 is not None else "None")
        # 换手入箱
        try:
            self.handoff_to_left_and_box(arm_hold, name)
        except Exception as e:
            self.log("scene1 %s 换手入箱失败: %s", name, e)
            try:
                lift2 = [WEIGH_REGRASP_IK[0], WEIGH_REGRASP_IK[1], WEIGH_REGRASP_IK[2] + LIFT_Z_OFFSET]
                self.right_arm_to(arm_hold, lift2, self.weigh_regrasp_quat)
                self.set_gripper(GRIP_OPEN, GRIP_OPEN)
            except Exception:
                pass
        self.log("scene1 %s done" % name)


def _load_launcher():
    import rospkg
    sim_utils = os.path.join(rospkg.RosPack().get_path("challenge_cup_simulator"), "utils")
    if sim_utils not in sys.path:
        sys.path.insert(0, sim_utils)
    from challenge_sim_launcher import ChallengeSimLauncher
    return ChallengeSimLauncher


def run_scene2(seed, time_limit=None, timer_gui=True):
    """scene2 零件分拣：检测 6 零件 -> 按类型抓取 -> 放对应箱。"""
    import rospy
    import math as _m
    stood = {"ok": False}
    def _cb(msg):
        stood["ok"] = True
    rospy.Subscriber("/bot_stand_up_complete", rospy.AnyMsg, _cb, queue_size=5)
    ChallengeSimLauncher = _load_launcher()
    launcher = ChallengeSimLauncher(scene="scene2", seed=seed, match_time_limit=time_limit or 0, timer_gui=timer_gui)
    launcher.start(node_name="challenge_task_scene2", timeout=120)
    rospy.loginfo("scene2 启动 seed=%s", seed)
    runner = Scene1Runner()
    t0 = rospy.Time.now()
    while not stood["ok"] and (rospy.Time.now() - t0).to_sec() < 150:
        rospy.sleep(0.5)
    rospy.loginfo("scene2 stand-up=%s", stood["ok"])
    rospy.sleep(4.0)
    runner.set_arm_mode(ARM_MODE_EXTERNAL_CONTROL, 20.0)
    arm_pub = runner.arm_pub
    cur = runner.read_current_arm_rad(90.0)
    arm_hold = ArmHold(arm_pub, [_m.degrees(v) for v in cur])
    arm_hold.start()
    rospy.sleep(0.5)
    runner.move_arm_deg(arm_hold, PRESET_FINAL_DEG, move_time=2.0, settle=1.2)
    runner.set_gripper(GRIP_OPEN, GRIP_OPEN)
    publish_head_down()  # 头部下看，避免近端零件被画面下边缘裁切（官方 HEAD_TARGET=[0,20]）

    parts, _bins = detect_scene2_parts(rospy.loginfo)
    if not parts:
        rospy.logwarn("scene2 未检测到零件")
        return
    for idx, part in enumerate(parts):
        t, x, y, z, yaw = part
        arm = SCENE2_BIN_TARGET_ARM.get(t, "right")
        if arm is None:
            # B 按零件 y 分左右手（y<0 右手，y>0 左手）
            arm = "right" if y < 0 else "left"
        name = "part_%s_%d" % (t, idx)
        # 抓取位姿：官方自适应 yaw（工件自身 yaw + yaw_adaptive + 窄边自动选）
        yaw_rad = yaw if yaw is not None else 0.0
        # C 用「红柄+银杆」组合 PCA yaw + 官方 yaw_adaptive=-45°；A/B 用固定位姿
        if t == "C":
            quat_grasp = scene2_grasp_quat(t, yaw_rad, arm, y)
        else:
            yaw_manual_deg = SCENE2_YAW_MANUAL_DEG if arm == "right" else -SCENE2_YAW_MANUAL_DEG
            quat_grasp = quat_from_ypr([0.0, SCENE2_GRASP_PITCH_DEG, 0.0], [yaw_manual_deg, 0.0, 0.0])
        rospy.loginfo("scene2 %s yaw=%.1fdeg arm=%s", name, _m.degrees(yaw_rad), arm)
        # 抓取点：深度定位中心 + 手指补偿（抓取姿态手指 +x 47mm）+ IK 欠冲补偿
        # diag FK 实测：右臂 x 欠冲 3.6cm、y 偏 -1.4cm；左臂 x 欠冲 2.9cm、y ~0
        # 远点（|y|>0.33）在工作空间边缘，IK 执行误差大，抓取点向中线内收 3cm 抓近边
        y_g = y
        if abs(y) > 0.33:
            y_g = y - 0.03 * (1 if y > 0 else -1)
        if arm == "right":
            gx, gy, gz = x - 0.047 + 0.036, y_g + 0.014, z + 0.01
        else:
            # 左臂 x/y 超调都随 y 增大（FK 实测 y=0.39 时手指 x 前伸 +3.8cm、y +3.7cm），线性补偿
            b = -0.20 * (y_g - 0.22) if y_g > 0.22 else 0.0
            gx, gy, gz = x - 0.047 + 0.029 + b, y_g + b, z + 0.01
        try:
            # 中转到位
            if arm == "right":
                runner.right_arm_to(arm_hold, [gx, gy, RIGHT_PICK_TRANSIT_IK_Z], quat_grasp)
            else:
                runner.left_arm_to(arm_hold, [gx, gy, RIGHT_PICK_TRANSIT_IK_Z], quat_grasp)
            # 下探 + FK 闭环校正：工作空间边缘 IK 执行误差大且不稳，读 FK 迭代校正到期望 EE
            _des = [x - 0.047, y_g, z + 0.01]   # 期望 EE：手指前伸 47mm 后恰在工件近边
            _cx, _cy, _cz = gx, gy, gz
            for _fc in range(3):
                if arm == "right":
                    runner.right_grasp_to(arm_hold, [_cx, _cy, _cz], quat_grasp)
                else:
                    runner.left_grasp_to(arm_hold, [_cx, _cy, _cz], quat_grasp)
                _fk = runner.call_fk(runner.read_current_arm_rad(10), 5.0)
                _e = list(_fk.left_pose.pos_xyz) if arm == "left" else list(_fk.right_pose.pos_xyz)
                _ex, _ey, _ez = _des[0] - _e[0], _des[1] - _e[1], _des[2] - _e[2]
                rospy.loginfo("scene2 %s FK校正%d fk=(%.3f,%.3f,%.3f) err=(%.3f,%.3f,%.3f)", name, _fc, _e[0], _e[1], _e[2], _ex, _ey, _ez)
                if max(abs(_ex), abs(_ey), abs(_ez)) < 0.012:
                    break
                _cx, _cy, _cz = _cx + _ex, _cy + _ey, _cz + _ez
            gx, gy, gz = _cx, _cy, _cz   # 抬升/后续用校正后的实际抓取点
            # 闭合
            if arm == "right":
                runner.set_gripper(GRIP_OPEN, GRIP_CLOSE)
            else:
                runner.set_gripper(GRIP_CLOSE, GRIP_OPEN)
            rospy.sleep(1.0)
            g = runner.read_gripper_state(arm)
            rospy.loginfo("scene2 %s GRASP=%s", name, str(round(g, 4)) if g is not None else "None")
            try:
                _fk = runner.call_fk(runner.read_current_arm_rad(10), 5.0)
                _e = list(_fk.left_pose.pos_xyz) if arm == "left" else list(_fk.right_pose.pos_xyz)
                rospy.loginfo("scene2 %s 抓取FK=(%.3f,%.3f,%.3f) det=(%.3f,%.3f,%.3f)", name, _e[0], _e[1], _e[2], x, y, z)
            except Exception:
                pass
            # 空抓重试：夹爪值 >=0.72 说明没夹到（细长螺丝刀 0.73=空，A/B 空也在 0.73 附近）。
            # 远点件抓取点偶有检测噪声/超调，向后(靠近机器人)+向内(靠近中线)偏移重试，最多 2 次。
            if g is not None and g >= 0.72:
                for _ri in range(2):
                    _rx = gx - 0.03 * (_ri + 1)
                    _ry = gy + (-0.02 * (_ri + 1) if arm == "left" else 0.02 * (_ri + 1))
                    _rz = gz - 0.008 * (_ri + 1)   # 每次再下探 8mm（补深度 z 噪声）
                    runner.set_gripper(GRIP_OPEN, GRIP_OPEN)
                    rospy.sleep(0.3)
                    if arm == "right":
                        runner.right_arm_to(arm_hold, [_rx, _ry, RIGHT_PICK_TRANSIT_IK_Z], quat_grasp)
                        runner.right_grasp_to(arm_hold, [_rx, _ry, _rz], quat_grasp)
                        runner.set_gripper(GRIP_OPEN, GRIP_CLOSE)
                    else:
                        runner.left_arm_to(arm_hold, [_rx, _ry, RIGHT_PICK_TRANSIT_IK_Z], quat_grasp)
                        runner.left_grasp_to(arm_hold, [_rx, _ry, _rz], quat_grasp)
                        runner.set_gripper(GRIP_CLOSE, GRIP_OPEN)
                    rospy.sleep(1.0)
                    _g2 = runner.read_gripper_state(arm)
                    rospy.loginfo("scene2 %s 空抓重试%d GRASP=%s", name, _ri + 1, str(round(_g2, 4)) if _g2 is not None else "None")
                    if _g2 is not None and _g2 < 0.72:
                        gx, gy, g, gz = _rx, _ry, _g2, _rz
                        break
                if g is not None and g >= 0.72:
                    rospy.logwarn("scene2 %s 空抓重试后仍空 GRASP=%s，继续流程", name, str(round(g, 4)))
            # 抬升
            lift = [gx, gy, gz + LIFT_Z_OFFSET]
            if arm == "right":
                runner.right_arm_to(arm_hold, lift, quat_grasp)
            else:
                runner.left_arm_to(arm_hold, lift, quat_grasp)
            runner.save_head_snapshot("/tmp/lift_%s.png" % name)
            # 放箱（IK 笛卡尔：抬升后保持抓取姿态移到箱上方→下放→松开）
            # 之前用关节空间摆动从抓取位甩到放置位，腕部会大角度扭转，松抓的螺丝刀易在甩动中滑脱
            bin_xy = SCENE2_BIN_XY.get(t)
            if bin_xy is not None:
                bx, by = bin_xy
                if arm == "right":
                    px, py = bx + 0.036, by + 0.014   # 右臂 IK 欠冲补偿
                else:
                    y_bias = -0.14 * (by - 0.22) if by > 0.22 else 0.0
                    px, py = bx + 0.029, by + y_bias  # 左臂 IK 欠冲 + y 超调补偿
                transit_z = 0.30   # 中转高度（高于箱，与抬升高度一致）
                release_z = 0.20   # 下放释放高度（箱上方）
                if arm == "right":
                    runner.right_arm_to(arm_hold, [px, py, transit_z], quat_grasp)
                    runner.right_grasp_to(arm_hold, [px, py, release_z], quat_grasp)
                else:
                    runner.left_arm_to(arm_hold, [px, py, transit_z], quat_grasp)
                    runner.left_grasp_to(arm_hold, [px, py, release_z], quat_grasp)
                runner.set_gripper(GRIP_OPEN, GRIP_OPEN)
                rospy.sleep(1.5)  # 开爪后驻留，等零件落入箱内再归位
                rospy.loginfo("scene2 %s 已放箱 %s", name, t)
            # 归位
            runner.move_arm_deg(arm_hold, PRESET_FINAL_DEG, move_time=2.0, settle=1.0)
            runner.set_gripper(GRIP_OPEN, GRIP_OPEN)
        except Exception as e:
            rospy.logerr("scene2 %s 失败: %s", name, e)
            try:
                runner.move_arm_deg(arm_hold, PRESET_FINAL_DEG, move_time=2.0, settle=1.0)
            except Exception:
                pass
    rospy.loginfo("scene2 试点结束")
    runner.save_head_snapshot("/tmp/scene2_final.png")
    rospy.sleep(2.0)
    if time_limit:
        rospy.signal_shutdown("done")


def run_scene(scene, seed, time_limit=None, timer_gui=True):
    import rospy
    if scene == "scene2":
        run_scene2(seed, time_limit=time_limit, timer_gui=timer_gui)
        return
    ChallengeSimLauncher = _load_launcher()
    launcher = ChallengeSimLauncher(scene=scene, seed=seed, match_time_limit=time_limit or 0, timer_gui=timer_gui)
    launcher.start(node_name="challenge_task_scene1", timeout=120)

    import rospy
    from sensor_msgs.msg import JointState
    rospy.loginfo("scene1 试点启动 seed=%s", seed)
    runner = Scene1Runner()
    rospy.sleep(2.0)

    # 等机器人站起 + 传感器就绪（过早调用 arm mode 服务会把节点打崩）
    try:
        rospy.wait_for_message("/bot_stand_up_complete", rospy.AnyMsg, timeout=150)
        rospy.loginfo("scene1: stand-up complete")
    except Exception as e:
        rospy.logwarn("scene1: stand-up wait: %s", e)

    arm_pub = runner.arm_pub
    cur = runner.read_current_arm_rad(90.0)
    rospy.loginfo("scene1: sensors ready")

    # 关键：切换到外部控制模式(2)，否则手臂不跟随 /kuavo_arm_traj
    for _attempt in range(5):
        try:
            runner.set_arm_mode(ARM_MODE_EXTERNAL_CONTROL, 20.0)
            rospy.loginfo("scene1: arm mode set OK")
            break
        except Exception as e:
            rospy.logwarn("scene1: set arm mode attempt %d failed: %s", _attempt + 1, e)
            rospy.sleep(3.0)
    arm_hold = ArmHold(arm_pub, [math.degrees(v) for v in cur])
    arm_hold.start()
    # 预设
    runner.move_arm_deg(arm_hold, PRESET_FINAL_DEG, move_time=2.0, settle=1.0)
    runner.set_gripper(GRIP_OPEN, GRIP_OPEN)

    # 每抓一个前重新检测（抓走后剩余包裹坐标实时更新，避免抓第一个后其他包裹被碰动导致坐标过时）
    for idx in range(4):
        parcels = detect_parcels(rospy.loginfo)
        if not parcels:
            rospy.loginfo("scene1 第 %d 轮未检测到包裹，结束", idx)
            break
        item = parcels[0]  # 取 x 最小（最靠左）的包裹
        ik_xy = (item[0], item[1])
        z_top = item[2] if len(item) > 2 else None
        name = "parcel_det_%d" % idx
        try:
            runner.run_parcel(arm_hold, name, ik_xy, z_top=z_top)
        except Exception as e:
            rospy.logerr("scene1 %s 失败: %s", name, e)

    rospy.loginfo("scene1 试点结束")
    rospy.sleep(2.0)
    if time_limit:
        rospy.signal_shutdown("done")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default="scene1")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--time-limit", type=float, default=None)
    parser.add_argument("--no-timer-gui", action="store_true")
    args = parser.parse_args()
    run_scene(args.scene, args.seed, time_limit=args.time_limit, timer_gui=not args.no_timer_gui)


if __name__ == "__main__":
    main()
