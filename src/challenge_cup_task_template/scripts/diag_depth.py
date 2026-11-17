#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, sys
def _load_launcher():
    import rospkg
    su = os.path.join(rospkg.RosPack().get_path("challenge_cup_simulator"), "utils")
    if su not in sys.path:
        sys.path.insert(0, su)
    from challenge_sim_launcher import ChallengeSimLauncher
    return ChallengeSimLauncher
import rospy
Launcher = _load_launcher()
launcher = Launcher(scene="scene1", seed=0, match_time_limit=0, timer_gui=False)
launcher.start(node_name="depth_probe", timeout=180)
rospy.wait_for_message("/bot_stand_up_complete", rospy.AnyMsg, timeout=150)
import cv2, numpy as np
from sensor_msgs.msg import CompressedImage
msg = rospy.wait_for_message('/cam_h/depth/image_raw/compressedDepth', CompressedImage, timeout=15)
png = np.frombuffer(msg.data[12:], np.uint8)
depth = cv2.imdecode(png, cv2.IMREAD_UNCHANGED)
rospy.loginfo("DP: shape=%s dtype=%s", depth.shape, depth.dtype)
depth = depth.astype(np.float32)
valid = depth[depth > 0]
rospy.loginfo("DP: nonzero %d/%d", len(valid), depth.size)
for p in [1, 5, 10, 25, 50, 75, 90, 99]:
    rospy.loginfo("DP: p%d=%.0f", p, np.percentile(valid, p))
rospy.loginfo("DP: max=%.0f", depth.max())
vis = np.clip(depth, 0, 3000) / 3000.0 * 255
cv2.imwrite("/tmp/depth_vis2.png", vis.astype(np.uint8))
rospy.loginfo("DP_DONE")
rospy.signal_shutdown("done")
