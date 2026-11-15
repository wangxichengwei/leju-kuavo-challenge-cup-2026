#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""到预抓取位后拍右腕相机，看快递在不在视野。"""
import math, os, sys, threading
import numpy as np

ARM_JOINT_NAMES = ["arm_joint_%d" % i for i in range(1, 15)]
ARM_TRAJ_HZ = 50.0
IK_MODE = 0x06
WORLD_TO_IK_OFFSET = [0.565966, -0.013886, -0.923783]
PRESET = [29.89, 30.67, 29.889, -139.1, -59.33, 0, 0, 29.89, -30.67, -29.889, -139.1, 59.33, 0, 0]
RIGHT_PICK_TRANSIT_IK_Z = 0.12


def _mm3(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _rot2q(m):
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
    n = math.sqrt(x*x + y*y + z*z + w*w)
    return [x/n, y/n, z/n, w/n]


def q_from_ypr(first, second=None):
    yaw, pitch, _ = [math.radians(v) for v in first]
    cy, sy = math.cos(yaw), math.sin(yaw); cp, sp = math.cos(pitch), math.sin(pitch)
    m = [[cy*cp, -sy, cy*sp], [sy*cp, cy, sy*sp], [-sp, 0.0, cp]]
    if second is not None:
        my, mp, mr = [math.radians(v) for v in second]
        mm = [[1,0,0],[0,1,0],[0,0,1]]
        if abs(my) > 0.01:
            c,s = math.cos(my), math.sin(my); mm = _mm3([[c,-s,0],[s,c,0],[0,0,1]], mm)
        if abs(mp) > 0.01:
            c,s = math.cos(mp), math.sin(mp); mm = _mm3([[c,0,s],[0,1,0],[-s,0,c]], mm)
        if abs(mr) > 0.01:
            c,s = math.cos(mr), math.sin(mr); mm = _mm3([[1,0,0],[0,c,-s],[0,s,c]], mm)
        m = _mm3(m, mm)
    return _rot2q(m)


class Hold:
    def __init__(self, pub, deg):
        self._pub = pub; self._lock = threading.Lock()
        self._deg = [float(v) for v in deg]
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)
    def start(self): self._t.start()
    def set(self, d):
        with self._lock: self._deg = [float(v) for v in d]
    def _run(self):
        import rospy
        from sensor_msgs.msg import JointState
        r = rospy.Rate(ARM_TRAJ_HZ)
        while not self._stop.is_set() and not rospy.is_shutdown():
            with self._lock: d = list(self._deg)
            m = JointState(); m.header.stamp = rospy.Time.now(); m.name = ARM_JOINT_NAMES; m.position = d
            try:
                self._pub.publish(m); r.sleep()
            except rospy.ROSException:
                break


def read_arm(t=30.0):
    import rospy
    from kuavo_msgs.msg import sensorsData
    m = rospy.wait_for_message("/sensors_data_raw", sensorsData, timeout=t)
    q = list(m.joint_data.joint_q)
    return q[13:27] if len(q) >= 27 else q[12:26]


def call_fk(q, t=10.0):
    import rospy
    from kuavo_msgs.srv import fkSrv
    rospy.wait_for_service("/ik/fk_srv", timeout=t)
    r = rospy.ServiceProxy("/ik/fk_srv", fkSrv)(list(q))
    if not r.success: raise RuntimeError("fk fail")
    return r.hand_poses


def call_ik(lpos, lquat, rpos, rquat, cur, t=10.0):
    import rospy
    from kuavo_msgs.msg import twoArmHandPoseCmd, ikSolveParam
    from kuavo_msgs.srv import twoArmHandPoseCmdSrv
    req = twoArmHandPoseCmd()
    p = ikSolveParam()
    p.major_optimality_tol = 1e-3; p.major_feasibility_tol = 1e-3; p.minor_feasibility_tol = 1e-3
    p.major_iterations_limit = 500; p.oritation_constraint_tol = 1e-3; p.pos_constraint_tol = 1e-3
    p.pos_cost_weight = 0.0; p.constraint_mode = IK_MODE
    req.ik_param = p; req.use_custom_ik_param = True; req.joint_angles_as_q0 = True
    req.hand_poses.left_pose.joint_angles = list(cur[:7])
    req.hand_poses.right_pose.joint_angles = list(cur[7:])
    req.hand_poses.left_pose.elbow_pos_xyz = [0.0,0.0,0.0]
    req.hand_poses.right_pose.elbow_pos_xyz = [0.0,0.0,0.0]
    req.hand_poses.left_pose.pos_xyz = list(lpos)
    req.hand_poses.left_pose.quat_xyzw = list(lquat)
    req.hand_poses.right_pose.pos_xyz = list(rpos)
    req.hand_poses.right_pose.quat_xyzw = list(rquat)
    rospy.wait_for_service("/ik/two_arm_hand_pose_cmd_srv", timeout=t)
    resp = rospy.ServiceProxy("/ik/two_arm_hand_pose_cmd_srv", twoArmHandPoseCmdSrv)(req)
    if not resp.success: raise RuntimeError("ik fail: %s" % getattr(resp, "error_reason", ""))
    q = list(resp.q_arm)
    if len(q) < 14:
        l = list(resp.hand_poses.left_pose.joint_angles); r2 = list(resp.hand_poses.right_pose.joint_angles)
        if len(l) == 7 and len(r2) == 7: q = l + r2
    return q[:14]


def move_hold(hold, target_deg, mt=1.8, settle=1.0):
    import rospy
    cur = [math.degrees(v) for v in read_arm(30)]
    steps = max(1, int(round(mt * ARM_TRAJ_HZ)))
    rate = rospy.Rate(ARM_TRAJ_HZ)
    for i in range(steps + 1):
        a = float(i) / steps
        hold.set([cur[k] + (target_deg[k] - cur[k]) * a for k in range(14)])
        if i < steps: rate.sleep()
    rospy.sleep(settle)


def right_to(hold, ikpos, quat):
    cur = read_arm(30)
    fk = call_fk(cur, 10)
    q = call_ik(list(fk.left_pose.pos_xyz), list(fk.left_pose.quat_xyzw), ikpos, quat, cur, 10)
    cur_deg = [math.degrees(v) for v in cur]
    q_deg = [math.degrees(v) for v in q]
    move_hold(hold, cur_deg[:7] + q_deg[7:14])


def snap(tag):
    import rospy, cv2
    from sensor_msgs.msg import CompressedImage
    try:
        m = rospy.wait_for_message("/cam_r/color/image_raw/compressed", CompressedImage, timeout=8)
        img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
        p = "/tmp/probe_%s.png" % tag
        cv2.imwrite(p, img)
        rospy.loginfo("PROBE saved %s", p)
    except Exception as e:
        rospy.logwarn("PROBE snap fail %s", e)


def main():
    import rospy
    import rospkg
    sim_utils = os.path.join(rospkg.RosPack().get_path("challenge_cup_simulator"), "utils")
    if sim_utils not in sys.path: sys.path.insert(0, sim_utils)
    from challenge_sim_launcher import ChallengeSimLauncher
    launcher = ChallengeSimLauncher(scene="scene1", seed=0, match_time_limit=0, timer_gui=False)
    launcher.start(node_name="probe_wrist", timeout=180)
    rospy.loginfo("PROBE: waiting standup")
    try:
        rospy.wait_for_message("/bot_stand_up_complete", rospy.AnyMsg, timeout=120)
    except Exception as e:
        rospy.logwarn("PROBE standup wait: %s", e)
    from sensor_msgs.msg import JointState
    pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)
    hold = Hold(pub, [math.degrees(v) for v in read_arm(60)])
    hold.start()
    rospy.sleep(0.5)
    rospy.loginfo("PROBE: moving preset")
    move_hold(hold, PRESET, 2.0, 1.2)
    quat = q_from_ypr([0, -90, 0.0], [90.0, 0.0, 0.0])
    # 预抓取位（parcel_2 位置）
    px, py = 0.332, -0.116
    rospy.loginfo("PROBE: moving to pre-grasp z=%.3f", RIGHT_PICK_TRANSIT_IK_Z)
    right_to(hold, [px, py, RIGHT_PICK_TRANSIT_IK_Z], quat)
    rospy.sleep(1.5)
    snap("pregrasp")
    # 再降一点看
    rospy.loginfo("PROBE: moving to z=0.02")
    right_to(hold, [px, py, 0.02], quat)
    rospy.sleep(1.5)
    snap("low")
    rospy.loginfo("PROBE_DONE")
    rospy.signal_shutdown("done")


if __name__ == "__main__":
    main()
