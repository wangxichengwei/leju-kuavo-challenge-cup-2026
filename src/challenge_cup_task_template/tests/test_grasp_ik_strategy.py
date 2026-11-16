#!/usr/bin/env python3
"""Behavior checks for the final-contact IK strategy."""

import importlib.util
import pathlib
import unittest


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "challenge_task.py"
SPEC = importlib.util.spec_from_file_location("challenge_task", SCRIPT)
challenge_task = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(challenge_task)


class _FakeRunner:
    def __init__(self):
        self.calls = []

    def right_arm_to(self, arm_hold, ik_pos, quat, constraint_mode, lock_left_preset):
        self.calls.append((constraint_mode, lock_left_preset))
        if len(self.calls) == 1:
            raise RuntimeError("hard IK unavailable")

    def log(self, *args):
        pass


class RightGraspIkStrategyTest(unittest.TestCase):
    def test_hard_single_hand_grasp_falls_back_to_existing_mixed_mode(self):
        """Catch regressions that let mixed IK silently accept a large pose error."""
        runner = _FakeRunner()
        try:
            grasp = challenge_task.Scene1Runner.right_grasp_to
        except AttributeError:
            self.fail("Scene1Runner.right_grasp_to is missing")

        grasp(runner, object(), [0.3, -0.1, -0.05], [0.0, 0.0, 0.707, 0.707])

        self.assertEqual(
            runner.calls,
            [
                (challenge_task.IK_MODE_POS_HARD_ORI_HARD, False),
                (challenge_task.IK_MODE_THREE_POINT_MIXED, True),
            ],
        )


if __name__ == "__main__":
    unittest.main()
