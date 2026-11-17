#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""闭爪前后 FK 对比：确认闭爪是否把末端 z 顶起。"""
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


def fk_pos(runner):
    cur_rad = runner.read_current_arm_rad(10)
    fk = runner.call_fk(cur_rad, 5.0)
    return list(fk.right_pose.pos_xyz)


def main():
    import rospy
    Launcher = _load_launcher()
    launcher = Launcher(scene="scene1", seed=0, match_time_limit=0, timer_gui=False)
    launcher.start(node_name="close_lift", timeout=180)
    rospy.loginfo("CL: waiting standup")
    try:
        rospy.wait_for_message("/bot_stand_up_complete", rospy.AnyMsg, timeout=150)
    except Exception as e:
        rospy.logwarn("CL: standup: %s", e)

    runner = ct.Scene1Runner()
    try:
        runner.set_arm_mode(ct.ARM_MODE_EXTERNAL_CONTROL, 20.0)
    except Exception as e:
        rospy.logerr("CL: arm mode fail: %s", e)

    arm_pub = runner.arm_pub
    cur = runner.read_current_arm_rad(90)
    arm_hold = ct.ArmHold(arm_pub, [math.degrees(v) for v in cur])
    arm_hold.start()
    rospy.sleep(0.5)
    runner.move_arm_deg(arm_hold, ct.PRESET_FINAL_DEG, move_time=2.0, settle=1.2)
    runner.set_gripper(ct.GRIP_OPEN, ct.GRIP_OPEN)
    rospy.loginfo("CL: preset done")

    parcels = ct.detect_parcels(rospy.loginfo)
    if not parcels:
        rospy.logerr("CL: no parcels")
        rospy.signal_shutdown("no_parcel")
        return
    px, py = parcels[0]
    offset = runner._grasp_offset(py)
    gx = px + offset[0]; gy = py + offset[1]
    gz = ct.RIGHT_PICK_IK_Z + offset[2]
    quat = runner.right_pick_quat
    rospy.loginfo("CL: grasp xy=(%.3f,%.3f) z=%.3f", gx, gy, gz)

    # 到 transit -> grasp
    runner.right_arm_to(arm_hold, [gx, gy, ct.RIGHT_PICK_TRANSIT_IK_Z], quat)
    runner.right_grasp_to(arm_hold, [gx, gy, gz], quat)
    before = fk_pos(runner)
    rospy.loginfo("CL_BEFORE_CLOSE z=%.3f (full %s)", before[2], [round(v,3) for v in before])

    # 闭爪
    runner.set_gripper(ct.GRIP_OPEN, ct.GRIP_CLOSE)
    rospy.sleep(1.0)
    mid = fk_pos(runner)
    rospy.loginfo("CL_AFTER_CLOSE_1s z=%.3f", mid[2])
    rospy.sleep(2.0)
    after = fk_pos(runner)
    rospy.loginfo("CL_AFTER_CLOSE_3s z=%.3f (full %s)", after[2], [round(v,3) for v in after])
    rospy.loginfo("CL_LIFT z: before=%.3f -> after=%.3f (delta=%.3f)", before[2], after[2], after[2]-before[2])

    rospy.loginfo("CL_DONE")
    rospy.signal_shutdown("done")


if __name__ == "__main__":
    main()
