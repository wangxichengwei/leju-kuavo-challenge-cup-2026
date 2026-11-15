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
WEIGH_RELEASE_IK = [0.396, -0.574, 0.146217]
WEIGH_REGRASP_IK = [0.396, -0.574, -0.04]
IK_MODE_THREE_POINT_MIXED = 0x06
RIGHT_PICK_FIRST_YPR = [0, -90, 0.0]
RIGHT_PICK_SECOND_YPR = [90, 0.0, 0.0]
WEIGH_RELEASE_FIRST_YPR = [0, -100, 0.0]
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
    msg = rospy.wait_for_message('/cam_h/color/image_raw/compressed', CompressedImage, timeout=15)
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
        _x, _y, _bw, _bh, area = stats[i]
        if area < 800 or area > 6000:
            continue
        u, v = cents[i]
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


class Scene1Runner:
    def __init__(self):
        import rospy
        from sensor_msgs.msg import JointState
        self.arm_pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)
        self.gripper_pub = rospy.Publisher("/gripper/command", JointState, queue_size=10)
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

    def move_arm_deg(self, arm_hold, target_deg, move_time=MOVE_TIME, settle=SETTLE):
        import rospy
        cur_rad = self.read_current_arm_rad()
        start = [math.degrees(v) for v in cur_rad]
        target = [float(v) for v in target_deg]
        steps = max(1, int(round(move_time * ARM_TRAJ_HZ)))
        rate = rospy.Rate(ARM_TRAJ_HZ)
        for i in range(steps + 1):
            a = float(i) / steps
            arm_hold.set_degrees([start[k] + (target[k] - start[k]) * a for k in range(14)])
            if i < steps:
                rate.sleep()
        rospy.sleep(settle)

    def set_gripper(self, left, right):
        import rospy
        from sensor_msgs.msg import JointState
        m = JointState(); m.header.stamp = rospy.Time.now()
        m.name = ["left_gripper_joint", "right_gripper_joint"]; m.position = [float(left), float(right)]
        for _ in range(5):
            self.gripper_pub.publish(m); rospy.sleep(0.05)

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

    def save_head_snapshot(self, path):
        import rospy
        import cv2
        import numpy as np
        from sensor_msgs.msg import CompressedImage
        try:
            m = rospy.wait_for_message("/cam_h/color/image_raw/compressed", CompressedImage, timeout=5.0)
            a = np.frombuffer(m.data, np.uint8)
            img = cv2.imdecode(a, cv2.IMREAD_COLOR)
            if img is not None:
                cv2.imwrite(path, img)
                self.log("scene1 snapshot saved %s" % path)
        except Exception as e:
            self.log("scene1 snapshot failed %s" % e)

    def call_ik(self, left_pos, left_quat, right_pos, right_quat, current_rad, timeout=5.0):
        import rospy
        from kuavo_msgs.msg import twoArmHandPoseCmd, ikSolveParam
        from kuavo_msgs.srv import twoArmHandPoseCmdSrv
        req = twoArmHandPoseCmd()
        p = ikSolveParam()
        p.major_optimality_tol = 1e-3; p.major_feasibility_tol = 1e-3; p.minor_feasibility_tol = 1e-3
        p.major_iterations_limit = 500; p.oritation_constraint_tol = 1e-3; p.pos_constraint_tol = 1e-3
        p.pos_cost_weight = 0.0; p.constraint_mode = IK_MODE_THREE_POINT_MIXED
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

    # 右手到目标位姿（左手锁当前 FK）
    def right_arm_to(self, arm_hold, ik_pos, quat):
        cur = self.read_current_arm_rad()
        fk = self.call_fk(cur, 10.0)
        left_pos = list(fk.left_pose.pos_xyz)
        left_quat = list(fk.left_pose.quat_xyzw)
        q = self.call_ik(left_pos, left_quat, ik_pos, quat, cur)
        # 左手保持当前
        cur_deg = [math.degrees(v) for v in cur]
        q_deg = [math.degrees(v) for v in q]
        target = cur_deg[:7] + q_deg[7:14]
        self.move_arm_deg(arm_hold, target)
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

    def run_parcel(self, arm_hold, name, ik_xy):
        offset = self._grasp_offset(ik_xy[1])
        px = ik_xy[0] + offset[0]
        py = ik_xy[1] + offset[1]
        pz = RIGHT_PICK_IK_Z + offset[2]
        # 预抓取（高位对齐）
        pre = [px, py, RIGHT_PICK_TRANSIT_IK_Z]
        self.right_arm_to(arm_hold, pre, self.right_pick_quat)
        # 下降抓取
        grasp = [px, py, pz]
        self.right_arm_to(arm_hold, grasp, self.right_pick_quat)
        self.set_gripper(GRIP_OPEN, GRIP_CLOSE)  # 右手闭
        import rospy; rospy.sleep(1.0)
        g = self.read_gripper_state()
        self.log("scene1 %s GRASP_CHECK right_gripper=%s (0.8 means empty grasp)" % (name, str(round(g, 4)) if g is not None else "None"))
        # 抬升
        lift = [grasp[0], grasp[1], grasp[2] + LIFT_Z_OFFSET]
        self.right_arm_to(arm_hold, lift, self.right_pick_quat)
        # 移到称重区上方
        transit = [WEIGH_RELEASE_IK[0], WEIGH_RELEASE_IK[1], WEIGH_TRANSIT_IK_Z]
        self.right_arm_to(arm_hold, transit, self.weigh_release_quat)
        # 下降释放
        self.right_arm_to(arm_hold, WEIGH_RELEASE_IK, self.weigh_release_quat)
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

    arm_pub = runner.arm_pub
    cur = runner.read_current_arm_rad(10.0)
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
