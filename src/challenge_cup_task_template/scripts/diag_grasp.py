#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓取诊断 v2：闭爪后/抬升后各拍一张头部相机图（闭爪拍照带重试）。"""
import importlib.util, pathlib, os, sys, math

PKG = pathlib.Path("/root/kuavo_ws/src/challenge_cup_task_template")
SCRIPT = PKG / "scripts" / "challenge_task.py"
spec = importlib.util.spec_from_file_location("challenge_task", SCRIPT)
ct = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ct)


def snap_with_retry(runner, path, retries=3):
    import rospy
    for i in range(retries):
        runner.save_head_snapshot(path)
        if os.path.exists(path):
            return True
        rospy.logwarn("snap %s attempt %d failed, retry after 2s", path, i + 1)
        rospy.sleep(2.0)
    return False


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
    launcher.start(node_name="diag_grasp", timeout=180)
    rospy.loginfo("DIAG: waiting standup")
    try:
        rospy.wait_for_message("/bot_stand_up_complete", rospy.AnyMsg, timeout=150)
    except Exception as e:
        rospy.logwarn("DIAG: standup wait: %s", e)

    runner = ct.Scene1Runner()
    try:
        runner.set_arm_mode(ct.ARM_MODE_EXTERNAL_CONTROL, 20.0)
        rospy.loginfo("DIAG: arm mode OK")
    except Exception as e:
        rospy.logerr("DIAG: arm mode fail: %s", e)

    arm_pub = runner.arm_pub
    cur = runner.read_current_arm_rad(90)
    rospy.loginfo("DIAG: sensors OK")
    arm_hold = ct.ArmHold(arm_pub, [math.degrees(v) for v in cur])
    arm_hold.start()
    rospy.sleep(0.5)
    runner.move_arm_deg(arm_hold, ct.PRESET_FINAL_DEG, move_time=2.0, settle=1.2)
    runner.set_gripper(ct.GRIP_OPEN, ct.GRIP_OPEN)
    rospy.loginfo("DIAG: preset done")

    parcels = ct.detect_parcels(rospy.loginfo)
    if not parcels:
        rospy.logerr("DIAG: no parcels detected")
        rospy.signal_shutdown("no_parcel")
        return
    px, py = parcels[0]
    rospy.loginfo("DIAG: target parcel=(%.3f, %.3f)", px, py)

    offset = runner._grasp_offset(py)
    gx = px + offset[0]
    gy = py + offset[1]
    gz = ct.RIGHT_PICK_IK_Z + offset[2]
    quat = runner.right_pick_quat

    runner.right_arm_to(arm_hold, [gx, gy, ct.RIGHT_PICK_TRANSIT_IK_Z], quat)
    rospy.loginfo("DIAG: pre-grasp reached")
    runner.right_grasp_to(arm_hold, [gx, gy, gz], quat)
    rospy.loginfo("DIAG: grasp pose reached, closing gripper")

    runner.set_gripper(ct.GRIP_OPEN, ct.GRIP_CLOSE)
    rospy.sleep(2.5)
    g = runner.read_gripper_state()
    rospy.loginfo("DIAG: after-close right_gripper=%s", g)
    ok_closed = snap_with_retry(runner, "/tmp/diag_closed.png")
    rospy.loginfo("DIAG: closed snapshot ok=%s", ok_closed)

    runner.right_arm_to(arm_hold, [gx, gy, gz + ct.LIFT_Z_OFFSET], quat)
    rospy.sleep(2.0)
    ok_lifted = snap_with_retry(runner, "/tmp/diag_lifted.png")
    rospy.loginfo("DIAG: lifted snapshot ok=%s", ok_lifted)

    rospy.loginfo("DIAG_DONE")
    rospy.signal_shutdown("done")


if __name__ == "__main__":
    main()
