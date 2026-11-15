#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scene1 诊断：验证传感器上线 + 手臂运动 + FK/IK。"""
import math, os, sys, threading

ARM_JOINT_NAMES = ["arm_joint_%d" % i for i in range(1, 15)]
ARM_TRAJ_HZ = 50.0
IK_MODE_THREE_POINT_MIXED = 0x06
PRESET_FINAL_DEG = [29.89, 30.67, 29.889, -139.1, -59.33, 0, 0, 29.89, -30.67, -29.889, -139.1, 59.33, 0, 0]
RIGHT_PICK_FIRST_YPR = [0, -90, 0.0]
RIGHT_PICK_SECOND_YPR = [90, 0.0, 0.0]


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
        self._pub = pub; self._lock = threading.Lock()
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


def read_current_arm_rad(timeout):
    import rospy
    from kuavo_msgs.msg import sensorsData
    msg = rospy.wait_for_message("/sensors_data_raw", sensorsData, timeout=timeout)
    q = list(msg.joint_data.joint_q)
    if len(q) >= 27:
        return q[13:27]
    if len(q) >= 26:
        return q[12:26]
    raise RuntimeError("joint_q len %d" % len(q))


def move_arm_deg(hold, target_deg, move_time=2.0, settle=1.0):
    import rospy
    cur = read_current_arm_rad(30)
    start = [math.degrees(v) for v in cur]
    target = [float(v) for v in target_deg]
    steps = max(1, int(round(move_time * ARM_TRAJ_HZ)))
    rate = rospy.Rate(ARM_TRAJ_HZ)
    for i in range(steps + 1):
        a = float(i) / steps
        hold.set_degrees([start[k] + (target[k] - start[k]) * a for k in range(14)])
        if i < steps:
            rate.sleep()
    rospy.sleep(settle)


def call_fk(joint_rad, timeout=10.0):
    import rospy
    from kuavo_msgs.srv import fkSrv
    rospy.wait_for_service("/ik/fk_srv", timeout=timeout)
    resp = rospy.ServiceProxy("/ik/fk_srv", fkSrv)(list(joint_rad))
    if not resp.success:
        raise RuntimeError("fk failed")
    return resp.hand_poses


def call_ik(right_pos, right_quat, current_rad, timeout=10.0):
    import rospy
    from kuavo_msgs.msg import twoArmHandPoseCmd, ikSolveParam
    from kuavo_msgs.srv import twoArmHandPoseCmdSrv
    req = twoArmHandPoseCmd()
    p = ikSolveParam()
    p.major_optimality_tol = 1e-3; p.major_feasibility_tol = 1e-3; p.minor_feasibility_tol = 1e-3
    p.major_iterations_limit = 500; p.oritation_constraint_tol = 1e-3; p.pos_constraint_tol = 1e-3
    p.pos_cost_weight = 0.0; p.constraint_mode = IK_MODE_THREE_POINT_MIXED
    req.ik_param = p; req.use_custom_ik_param = True; req.joint_angles_as_q0 = True
    req.hand_poses.left_pose.joint_angles = list(current_rad[:7])
    req.hand_poses.right_pose.joint_angles = list(current_rad[7:])
    req.hand_poses.left_pose.elbow_pos_xyz = [0.0, 0.0, 0.0]
    req.hand_poses.right_pose.elbow_pos_xyz = [0.0, 0.0, 0.0]
    # 左手锁当前，右手到目标
    fk = call_fk(current_rad, timeout)
    req.hand_poses.left_pose.pos_xyz = list(fk.left_pose.pos_xyz)
    req.hand_poses.left_pose.quat_xyzw = list(fk.left_pose.quat_xyzw)
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
    return q[:14]


def _load_launcher():
    import rospkg
    sim_utils = os.path.join(rospkg.RosPack().get_path("challenge_cup_simulator"), "utils")
    if sim_utils not in sys.path:
        sys.path.insert(0, sim_utils)
    from challenge_sim_launcher import ChallengeSimLauncher
    return ChallengeSimLauncher


def main():
    import rospy
    ChallengeSimLauncher = _load_launcher()
    launcher = ChallengeSimLauncher(scene="scene1", seed=0, match_time_limit=0, timer_gui=False)
    launcher.start(node_name="diag_arm", timeout=180)
    rospy.loginfo("DIAG: launch done, waiting stand-up...")
    try:
        rospy.wait_for_message("/bot_stand_up_complete", rospy.AnyMsg, timeout=90)
        rospy.loginfo("DIAG: stand-up complete")
    except Exception as e:
        rospy.logwarn("DIAG: stand-up timeout: %s", e)
    try:
        cur = read_current_arm_rad(60)
        rospy.loginfo("DIAG: sensors OK, arm_joints_rad=%s", [round(v, 4) for v in cur])
    except Exception as e:
        rospy.logerr("DIAG: sensors FAILED: %s", e)
        rospy.signal_shutdown("sensor_fail")
        return
    from sensor_msgs.msg import JointState
    pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)
    hold = ArmHold(pub, [math.degrees(v) for v in cur])
    hold.start()
    rospy.sleep(0.5)
    rospy.loginfo("DIAG: moving to preset...")
    move_arm_deg(hold, PRESET_FINAL_DEG, move_time=2.0, settle=1.5)
    after = read_current_arm_rad(30)
    rospy.loginfo("DIAG: preset done, arm_joints_rad=%s", [round(v, 4) for v in after])
    try:
        fk = call_fk(after, 10)
        rospy.loginfo("DIAG: FK OK, right_pos=%s", [round(v, 4) for v in fk.right_pose.pos_xyz])
    except Exception as e:
        rospy.logwarn("DIAG: FK failed: %s", e)
    try:
        q = call_ik([0.35, -0.20, 0.15], quat_from_ypr(RIGHT_PICK_FIRST_YPR, RIGHT_PICK_SECOND_YPR), after)
        rospy.loginfo("DIAG: IK OK, q_deg=%s", [round(math.degrees(v), 2) for v in q])
    except Exception as e:
        rospy.logerr("DIAG: IK failed: %s", e)
    rospy.sleep(1.0)
    rospy.loginfo("DIAG: DONE")
    rospy.signal_shutdown("diag_done")


if __name__ == "__main__":
    main()
