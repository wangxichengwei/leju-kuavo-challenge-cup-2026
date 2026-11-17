#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""z 扫描诊断：固定 x/y，扫 z，记录 0x03 可解性与执行后 FK 实测 z。"""
import importlib.util, pathlib, os, sys, math

PKG = pathlib.Path("/root/kuavo_ws/src/challenge_cup_task_template")
SCRIPT = PKG / "scripts" / "challenge_task.py"
spec = importlib.util.spec_from_file_location("challenge_task", SCRIPT)
ct = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ct)


def _load_launcher():
    import rospkg
    su = os.path.join(rospkg.RosPack().get_path("challenge_cup_simulator"), "utils")
    if su not in sys.path:
        sys.path.insert(0, su)
    from challenge_sim_launcher import ChallengeSimLauncher
    return ChallengeSimLauncher


def main():
    import rospy
    from sensor_msgs.msg import JointState

    Launcher = _load_launcher()
    launcher = Launcher(scene="scene1", seed=0, match_time_limit=0, timer_gui=False)
    launcher.start(node_name="zsweep_ik", timeout=180)
    rospy.loginfo("ZSW: waiting standup")
    try:
        rospy.wait_for_message("/bot_stand_up_complete", rospy.AnyMsg, timeout=150)
    except Exception as e:
        rospy.logwarn("ZSW: standup wait: %s", e)

    runner = ct.Scene1Runner()
    try:
        runner.set_arm_mode(ct.ARM_MODE_EXTERNAL_CONTROL, 20.0)
    except Exception as e:
        rospy.logerr("ZSW: arm mode fail: %s", e)

    arm_pub = runner.arm_pub
    cur = runner.read_current_arm_rad(90)
    arm_hold = ct.ArmHold(arm_pub, [math.degrees(v) for v in cur])
    arm_hold.start()
    rospy.sleep(0.5)
    runner.move_arm_deg(arm_hold, ct.PRESET_FINAL_DEG, move_time=2.0, settle=1.2)
    runner.set_gripper(ct.GRIP_OPEN, ct.GRIP_OPEN)
    rospy.loginfo("ZSW: preset done")

    parcels = ct.detect_parcels(rospy.loginfo)
    if not parcels:
        rospy.logerr("ZSW: no parcels")
        rospy.signal_shutdown("no_parcel")
        return
    px, py = parcels[0]
    offset = runner._grasp_offset(py)
    gx = px + offset[0]
    gy = py + offset[1]
    quat = runner.right_pick_quat
    rospy.loginfo("ZSW: target parcel=(%.3f,%.3f) grasp xy=(%.3f,%.3f)", px, py, gx, gy)

    ZS = [-0.01, -0.02, -0.03, -0.04, -0.05, -0.06]
    for z in ZS:
        # 先回 transit 安全高度
        try:
            runner.right_arm_to(arm_hold, [gx, gy, ct.RIGHT_PICK_TRANSIT_IK_Z], quat)
            rospy.sleep(0.5)
        except Exception as e:
            rospy.logwarn("ZSW z=%.3f transit fail: %s", z, e)
            continue
        # 0x03 硬约束下降到 z
        ok03 = False
        try:
            runner.right_arm_to(arm_hold, [gx, gy, z], quat,
                                constraint_mode=ct.IK_MODE_POS_HARD_ORI_HARD,
                                lock_left_preset=False)
            ok03 = True
        except Exception as e:
            rospy.logwarn("ZSW z=%.3f 0x03 IK FAIL: %s", z, e)
        # 读 FK 实测（无论成功失败）
        try:
            cur_rad = runner.read_current_arm_rad(10)
            fk = runner.call_fk(cur_rad, 5.0)
            rp = list(fk.right_pose.pos_xyz)
            rospy.loginfo("ZSW_RESULT z_cmd=%.3f ok03=%s z_fk=%.3f err_z=%.3f (x=%.3f y=%.3f)",
                          z, ok03, rp[2], rp[2] - z, rp[0], rp[1])
        except Exception as e:
            rospy.logwarn("ZSW z=%.3f FK read fail: %s", z, e)

    rospy.loginfo("ZSW_DONE")
    rospy.signal_shutdown("done")


if __name__ == "__main__":
    main()
