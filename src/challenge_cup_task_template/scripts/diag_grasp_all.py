#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""逐包裹闭爪拍照：对每个检测到的包裹，走 pre-grasp -> grasp -> 闭爪，拍一张头部相机图。"""
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
    launcher.start(node_name="diag_grasp_all", timeout=180)
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
    rospy.loginfo("DIAG: detected %d parcels: %s", len(parcels), [(round(a,3), round(b,3)) for a, b in parcels])
    if not parcels:
        rospy.logerr("DIAG: no parcels")
        rospy.signal_shutdown("no_parcel")
        return

    quat = runner.right_pick_quat

    for idx, (px, py) in enumerate(parcels):
        try:
            offset = runner._grasp_offset(py)
            gx = px + offset[0]
            gy = py + offset[1]
            gz = ct.RIGHT_PICK_IK_Z + offset[2]
            rospy.loginfo("DIAG parcel %d: detect=(%.3f,%.3f) grasp=(%.3f,%.3f,%.3f)", idx, px, py, gx, gy, gz)
            # pre-grasp
            runner.right_arm_to(arm_hold, [gx, gy, ct.RIGHT_PICK_TRANSIT_IK_Z], quat)
            rospy.sleep(0.5)
            # grasp
            runner.right_grasp_to(arm_hold, [gx, gy, gz], quat)
            rospy.sleep(0.5)
            # 闭爪
            runner.set_gripper(ct.GRIP_OPEN, ct.GRIP_CLOSE)
            rospy.sleep(2.5)
            g = runner.read_gripper_state()
            rospy.loginfo("DIAG parcel %d after-close right_gripper=%s", idx, g)
            # FK 实测末端位置，对比指令：分离"检测误差"vs"IK/执行误差"
            try:
                cur_rad = runner.read_current_arm_rad(10)
                fk = runner.call_fk(cur_rad, 5.0)
                rp = list(fk.right_pose.pos_xyz)
                rospy.loginfo("DIAG parcel %d FK_right=(%.3f,%.3f,%.3f) err_vs_grasp=(%.3f,%.3f,%.3f)",
                              idx, rp[0], rp[1], rp[2], rp[0]-gx, rp[1]-gy, rp[2]-gz)
            except Exception as e:
                rospy.logwarn("DIAG parcel %d FK read failed: %s", idx, e)
            ok = snap_with_retry(runner, "/tmp/diag_closed_parcel_%d.png" % idx)
            rospy.loginfo("DIAG parcel %d snapshot ok=%s", idx, ok)
        except Exception as e:
            rospy.logwarn("DIAG parcel %d FAILED: %s", idx, e)
        finally:
            # 松开 + 回 preset，准备下一个
            runner.set_gripper(ct.GRIP_OPEN, ct.GRIP_OPEN)
            rospy.sleep(0.8)
            try:
                runner.move_arm_deg(arm_hold, ct.PRESET_FINAL_DEG, move_time=1.5, settle=0.8)
            except Exception as e:
                rospy.logwarn("DIAG parcel %d reset failed: %s", idx, e)

    rospy.loginfo("DIAG_ALL_DONE")
    rospy.signal_shutdown("done")


if __name__ == "__main__":
    main()
