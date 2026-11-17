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
RIGHT_PICK_TRANSIT_IK_Z = 0.12
RIGHT_PICK_NEAR_FAR_Y_THRESHOLD = -0.20
RIGHT_PICK_OFFSET_FAR_ROW = [-0.03, 0.0, 0.0]
RIGHT_PICK_OFFSET_NEAR_ROW = [-0.03, 0.02, -0.02]
LIFT_Z_OFFSET = 0.30
WEIGH_TRANSIT_IK_Z = 0.326217
WEIGH_RELEASE_IK = [0.40, -0.574, 0.146217]
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


def _pixel_to_base(u, v, zplane):
    dcam = [(u-CAM_CX)/CAM_FX, (v-CAM_CY)/CAM_FY, 1.0]
    dbase = _rot_vec(dcam, CAM_QUAT)
    t = (zplane - CAM_POS[2]) / dbase[2]
    return [CAM_POS[0]+t*dbase[0], CAM_POS[1]+t*dbase[1]]


def detect_parcels(logfn):
    import rospy
    import cv2
    import numpy as np
    from sensor_msgs.msg import CompressedImage
    msg = None
    for _attempt in range(3):
        try:
            msg = rospy.wait_for_message('/cam_h/color/image_raw/compressed', CompressedImage, timeout=15)
            if msg is not None:
                break
        except Exception:
            rospy.sleep(1.0)
    if msg is None:
        return []
    arr = np.frombuffer(msg.data, np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
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
        bx, by = _pixel_to_base(u, v, PARCEL_PLANE_Z_BASE)
        bx += CAM_X_CORRECTION
        if bx < 0.0 or bx > 0.7:
            continue
        found.append((bx, by))
    found.sort()
    logfn("scene1 detect: %d parcels %s", len(found), [(round(a, 3), round(b, 3)) for a, b in found])
    return found


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

    def read_gripper_state(self):
        import rospy
        from sensor_msgs.msg import JointState
        try:
            msg = rospy.wait_for_message("/gripper/state", JointState, timeout=3.0)
            for i, n in enumerate(msg.name):
                if n == "right_gripper_joint" and i < len(msg.position):
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
                     constraint_mode=IK_MODE_THREE_POINT_MIXED, lock_left_preset=False):
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
                lock_left_preset=False)
        except RuntimeError as exc:
            self.log("scene1 hard grasp IK failed, fallback to mixed IK: %s" % exc)
            self.right_arm_to(
                arm_hold, ik_pos, quat,
                constraint_mode=IK_MODE_THREE_POINT_MIXED,
                lock_left_preset=False)

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

    def run_parcel(self, arm_hold, name, ik_xy):
        offset = self._grasp_offset(ik_xy[1])
        px = ik_xy[0] + offset[0]
        py = ik_xy[1] + offset[1]
        pz = RIGHT_PICK_IK_Z + offset[2]
        # 预抓取（高位对齐）
        pre = [px, py, RIGHT_PICK_TRANSIT_IK_Z]
        self.right_arm_to(arm_hold, pre, self.right_pick_quat)
        # 诊断：第一个包裹拍头部图看手臂是否到位
        if not getattr(self, "_head_snap_done", False):
            self._head_snap_done = True
            self.save_head_snapshot("/tmp/pregrasp_head.png")
        # 视觉伺服：腕相机检测快递，校正抓取 x/y
        wb = self.wrist_parcel_base()
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
        self.set_gripper(GRIP_OPEN, GRIP_CLOSE)  # 右手闭
        import rospy; rospy.sleep(1.0)
        g = self.read_gripper_state()
        self.log("scene1 %s GRASP_CHECK right_gripper=%s (0.8 means empty grasp)" % (name, str(round(g, 4)) if g is not None else "None"))
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
        # 二次抓取
        self.right_arm_to(arm_hold, WEIGH_REGRASP_IK, self.weigh_regrasp_quat)
        self.set_gripper(GRIP_OPEN, GRIP_CLOSE)
        rospy.sleep(1.0)
        # 抬升（第一版到此为止，第二阶段再接换手入箱）
        lift2 = [WEIGH_REGRASP_IK[0], WEIGH_REGRASP_IK[1], WEIGH_REGRASP_IK[2] + LIFT_Z_OFFSET]
        self.right_arm_to(arm_hold, lift2, self.weigh_regrasp_quat)
        self.set_gripper(GRIP_OPEN, GRIP_OPEN)
        self.log("scene1 %s done" % name)


def _load_launcher():
    import rospkg
    sim_utils = os.path.join(rospkg.RosPack().get_path("challenge_cup_simulator"), "utils")
    if sim_utils not in sys.path:
        sys.path.insert(0, sim_utils)
    from challenge_sim_launcher import ChallengeSimLauncher
    return ChallengeSimLauncher


def run_scene(scene, seed, time_limit=None, timer_gui=True):
    import rospy
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

    parcels = detect_parcels(rospy.loginfo)
    for idx, ik_xy in enumerate(parcels):
        name = "parcel_det_%d" % idx
        try:
            runner.run_parcel(arm_hold, name, ik_xy)
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
