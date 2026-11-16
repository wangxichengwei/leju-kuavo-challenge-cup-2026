#!/usr/bin/env python3
"""Pure geometry checks for the scene1 right-gripper pick pose."""

import importlib.util
import pathlib
import unittest


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "challenge_task.py"
SPEC = importlib.util.spec_from_file_location("challenge_task", SCRIPT)
challenge_task = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(challenge_task)


class RightPickOrientationTest(unittest.TestCase):
    def test_pick_pose_descends_vertically_with_fingers_across_parcel_width(self):
        """Catch poses that approach sideways or span the parcel's 12 cm edge."""
        quat = challenge_task.quat_from_ypr(
            challenge_task.RIGHT_PICK_FIRST_YPR,
            challenge_task.RIGHT_PICK_SECOND_YPR,
        )

        # From the S52 gripper mount: EEF +X is the finger opening axis and
        # EEF -Z is the direction from the palm toward the fingertips.
        opening_world = challenge_task._rot_vec([1.0, 0.0, 0.0], quat)
        approach_world = challenge_task._rot_vec([0.0, 0.0, -1.0], quat)

        # Scene1 parcels are yawed +90 degrees, so their 4.5 cm edge is world Y.
        self.assertVectorAlmostEqual(opening_world, [0.0, 1.0, 0.0])
        self.assertVectorAlmostEqual(approach_world, [0.0, 0.0, -1.0])

    def test_rows_use_same_grasp_height(self):
        """A camera-row choice must not drive one row 2 cm deeper into the table."""
        self.assertAlmostEqual(
            challenge_task.RIGHT_PICK_OFFSET_NEAR_ROW[2],
            challenge_task.RIGHT_PICK_OFFSET_FAR_ROW[2],
            places=6,
        )

    def assertVectorAlmostEqual(self, actual, expected):
        for got, want in zip(actual, expected):
            self.assertAlmostEqual(got, want, places=6)


if __name__ == "__main__":
    unittest.main()
