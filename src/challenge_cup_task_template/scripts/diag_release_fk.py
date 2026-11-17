#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""释放点 FK 诊断：手臂到 WEIGH_RELEASE_IK 后读 FK，量化偏差。"""
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
    Launcher = _load_launcher()
    launcher = Launcher(scene="scene1", seed=0, match_time_limit=0, timer_gui=False)
    launcher.start(node_name="release_fk", timeout=180)
    rospy.loginfo("RFK: waiting standup")
    try:
        rospy.wait_for_message("/bot_stand_up_complete", rospy.AnyMsg, timeout=150)
    except Exception as e:
        rospy.logwarn("RFK: standup: %s", e)

    runner = ct.Scene1Runner()
    try:
        runner.set_arm_mode(ct.ARM_MODE_EXTERNAL_CONTROL, 20.0)
    except Exception as e:
        rospy.logerr("RFK: arm mode fail: %s", e)

    arm_pub = runner.arm_pub
    cur = runner.read_current_arm_rad(90)
    arm_hold = ct.ArmHold(arm_pub, [math.degrees(v) for v in cur])
    arm_hold.start()
    rospy.sleep(0.5)
    runner.move_arm_deg(arm_hold, ct.PRESET_FINAL_DEG, move_time=2.0, settle=1.2)
    rospy.loginfo("RFK: preset done")

    release = ct.WEIGH_RELEASE_IK
    transit = [release[0], release[1], ct.WEIGH_TRANSIT_IK_Z]
    quat = runner.weigh_release_quat
    rospy.loginfo("RFK: release_ik=%s transit_z=%.3f", release, ct.WEIGH_TRANSIT_IK_Z)

    for k in range(3):
        try:
            runner.right_arm_to(arm_hold, transit, quat)
            rospy.sleep(0.5)
            runner.right_arm_to(arm_hold, release, quat)
            rospy.sleep(1.0)
            cur_rad = runner.read_current_arm_rad(10)
            fk = runner.call_fk(cur_rad, 5.0)
            rp = list(fk.right_pose.pos_xyz)
            rospy.loginfo("RFK_RESULT trial=%d cmd=(%.3f,%.3f,%.3f) fk=(%.3f,%.3f,%.3f) err=(%.3f,%.3f,%.3f)",
                          k, release[0], release[1], release[2], rp[0], rp[1], rp[2],
                          rp[0]-release[0], rp[1]-release[1], rp[2]-release[2])
        except Exception as e:
            rospy.logwarn("RFK trial=%d fail: %s", k, e)

    rospy.loginfo("RFK_DONE")
    rospy.signal_shutdown("done")


if __name__ == "__main__":
    main()
