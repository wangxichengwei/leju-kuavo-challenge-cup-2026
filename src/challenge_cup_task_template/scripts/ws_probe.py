#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""扫描 IK 可达工作空间。"""
import math, os, sys, threading, itertools

PRESET = [29.89, 30.67, 29.889, -139.1, -59.33, 0, 0, 29.89, -30.67, -29.889, -139.1, 59.33, 0, 0]
ARM_JOINT_NAMES = ["arm_joint_%d" % i for i in range(1, 15)]
IK_MODE = 0x06
LEFT_PRESET_2_IK = [0.313, 0.239, 0.282]


def _mm3(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _rot2q(m):
    tr = m[0][0]+m[1][1]+m[2][2]
    if tr > 0:
        s = math.sqrt(tr+1.0)*2.0
        w=0.25*s; x=(m[2][1]-m[1][2])/s; y=(m[0][2]-m[2][0])/s; z=(m[1][0]-m[0][1])/s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0+m[0][0]-m[1][1]-m[2][2])*2.0
        w=(m[2][1]-m[1][2])/s; x=0.25*s; y=(m[0][1]+m[1][0])/s; z=(m[0][2]+m[2][0])/s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0+m[1][1]-m[0][0]-m[2][2])*2.0
        w=(m[0][2]-m[2][0])/s; x=(m[0][1]+m[1][0])/s; y=0.25*s; z=(m[1][2]+m[2][1])/s
    else:
        s = math.sqrt(1.0+m[2][2]-m[0][0]-m[1][1])*2.0
        w=(m[1][0]-m[0][1])/s; x=(m[0][2]+m[2][0])/s; y=(m[1][2]+m[2][1])/s; z=0.25*s
    n = math.sqrt(x*x+y*y+z*z+w*w)
    return [x/n, y/n, z/n, w/n]


def q_ypr(first, second=None):
    yaw, pitch, _ = [math.radians(v) for v in first]
    cy, sy = math.cos(yaw), math.sin(yaw); cp, sp = math.cos(pitch), math.sin(pitch)
    m = [[cy*cp, -sy, cy*sp], [sy*cp, cy, sy*sp], [-sp, 0.0, cp]]
    if second is not None:
        my, mp, mr = [math.radians(v) for v in second]
        mm = [[1,0,0],[0,1,0],[0,0,1]]
        if abs(my) > 0.01:
            c,s=math.cos(my),math.sin(my); mm=_mm3([[c,-s,0],[s,c,0],[0,0,1]],mm)
        if abs(mp) > 0.01:
            c,s=math.cos(mp),math.sin(mp); mm=_mm3([[c,0,s],[0,1,0],[-s,0,c]],mm)
        if abs(mr) > 0.01:
            c,s=math.cos(mr),math.sin(mr); mm=_mm3([[1,0,0],[0,c,-s],[0,s,c]],mm)
        m = _mm3(m, mm)
    return _rot2q(m)


def read_arm(t=30.0):
    import rospy
    from kuavo_msgs.msg import sensorsData
    m = rospy.wait_for_message("/sensors_data_raw", sensorsData, timeout=t)
    q = list(m.joint_data.joint_q)
    return q[13:27] if len(q) >= 27 else q[12:26]


class Hold:
    def __init__(self, pub, deg):
        self._pub=pub; self._lock=threading.Lock(); self._deg=[float(v) for v in deg]
        self._stop=threading.Event(); self._t=threading.Thread(target=self._run,daemon=True)
    def start(self): self._t.start()
    def set(self,d):
        with self._lock: self._deg=[float(v) for v in d]
    def _run(self):
        import rospy
        from sensor_msgs.msg import JointState
        r=rospy.Rate(50.0)
        while not self._stop.is_set() and not rospy.is_shutdown():
            with self._lock: d=list(self._deg)
            m=JointState(); m.header.stamp=rospy.Time.now(); m.name=ARM_JOINT_NAMES; m.position=d
            try:
                self._pub.publish(m); r.sleep()
            except rospy.ROSException:
                break


def try_ik(left_pos, left_quat, right_pos, right_quat, cur, t=8.0):
    import rospy
    from kuavo_msgs.msg import twoArmHandPoseCmd, ikSolveParam
    from kuavo_msgs.srv import twoArmHandPoseCmdSrv
    req = twoArmHandPoseCmd()
    p = ikSolveParam()
    p.major_optimality_tol=1e-3; p.major_feasibility_tol=1e-3; p.minor_feasibility_tol=1e-3
    p.major_iterations_limit=500; p.oritation_constraint_tol=1e-3; p.pos_constraint_tol=1e-3
    p.pos_cost_weight=0.0; p.constraint_mode=IK_MODE
    req.ik_param=p; req.use_custom_ik_param=True; req.joint_angles_as_q0=True
    req.hand_poses.left_pose.joint_angles=list(cur[:7])
    req.hand_poses.right_pose.joint_angles=list(cur[7:])
    req.hand_poses.left_pose.elbow_pos_xyz=[0.0,0.0,0.0]
    req.hand_poses.right_pose.elbow_pos_xyz=[0.0,0.0,0.0]
    req.hand_poses.left_pose.pos_xyz=list(left_pos)
    req.hand_poses.left_pose.quat_xyzw=list(left_quat)
    req.hand_poses.right_pose.pos_xyz=list(right_pos)
    req.hand_poses.right_pose.quat_xyzw=list(right_quat)
    rospy.wait_for_service("/ik/two_arm_hand_pose_cmd_srv", timeout=t)
    try:
        resp = rospy.ServiceProxy("/ik/two_arm_hand_pose_cmd_srv", twoArmHandPoseCmdSrv)(req)
    except Exception as e:
        return False, str(e)
    if not resp.success:
        return False, getattr(resp, "error_reason", "")
    return True, ""


def main():
    import rospy, rospkg
    su = os.path.join(rospkg.RosPack().get_path("challenge_cup_simulator"), "utils")
    if su not in sys.path: sys.path.insert(0, su)
    from challenge_sim_launcher import ChallengeSimLauncher
    L = ChallengeSimLauncher(scene="scene1", seed=0, match_time_limit=0, timer_gui=False)
    L.start(node_name="ws_probe", timeout=180)
    rospy.loginfo("WS waiting standup")
    try:
        rospy.wait_for_message("/bot_stand_up_complete", rospy.AnyMsg, timeout=120)
    except Exception as e:
        rospy.logwarn("WS standup %s", e)
    from sensor_msgs.msg import JointState
    from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest
    try:
        rospy.wait_for_service("/arm_traj_change_mode", timeout=10)
        pr = rospy.ServiceProxy("/arm_traj_change_mode", changeArmCtrlMode)
        rq = changeArmCtrlModeRequest(); rq.control_mode = 2
        rr = pr(rq)
        rospy.loginfo("WS mode -> %s (%s)", rr.result, rr.message)
    except Exception as e:
        rospy.logwarn("WS mode fail: %s", e)
    pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)
    hold = Hold(pub, [math.degrees(v) for v in read_arm(60)])
    hold.start()
    rospy.sleep(0.5)
    # 移动到 preset
    cur = [math.degrees(v) for v in read_arm(30)]
    steps = 100
    rate = rospy.Rate(50)
    for i in range(steps+1):
        a = float(i)/steps
        hold.set([cur[k]+(PRESET[k]-cur[k])*a for k in range(14)])
        if i < steps: rate.sleep()
    rospy.sleep(1.5)
    cur_rad = read_arm(30)
    rospy.loginfo("WS cur_rad=%s", [round(v,3) for v in cur_rad])

    lq = q_ypr([-146.440, 4.966, 0.0], [0.0, 0.0, 96.580])
    rq2 = q_ypr([0, -90, 0.0], [90.0, 0.0, 0.0])
    rospy.loginfo("WS right_quat=%s", [round(v,3) for v in rq2])

    ok_count = 0
    for z in [0.25, 0.20, 0.12, 0.05, 0.0, -0.03]:
        for x in [0.10, 0.18, 0.25, 0.31, 0.38, 0.45]:
            for y in [-0.35, -0.25, -0.15, -0.05]:
                ok, msg = try_ik(LEFT_PRESET_2_IK, lq, [x, y, z], rq2, cur_rad)
                if ok:
                    ok_count += 1
                    rospy.loginfo("WS OK x=%.2f y=%.2f z=%.2f", x, y, z)
    rospy.loginfo("WS_TOTAL_OK=%d", ok_count)
    rospy.loginfo("WS_DONE")
    rospy.signal_shutdown("done")


if __name__ == "__main__":
    main()
