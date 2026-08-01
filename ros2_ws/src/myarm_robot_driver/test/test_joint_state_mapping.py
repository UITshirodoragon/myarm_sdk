"""Unit tests for the ROS-independent command-boundary mapping."""

import unittest

from myarm_robot_driver.joint_state_mapping import (
    canonical_joint_positions_from_names,
)

JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_flex_joint",
    "forearm_roll_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
)


class JointStateMappingTest(unittest.TestCase):
    def test_named_state_maps_to_canonical_order_and_ignores_extra_joint(self):
        result = canonical_joint_positions_from_names(
            names=(
                "wrist_roll_joint",
                "left_gripper_joint",
                "elbow_flex_joint",
                "shoulder_pan_joint",
                "wrist_flex_joint",
                "forearm_roll_joint",
                "shoulder_lift_joint",
            ),
            positions=(0.6, 9.0, 0.3, 0.1, 0.5, 0.4, 0.2),
            canonical_joint_names=JOINT_NAMES,
        )
        self.assertEqual(result.values, (0.1, 0.2, 0.3, 0.4, 0.5, 0.6))

    def test_duplicate_name_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "duplicate"):
            canonical_joint_positions_from_names(
                names=("shoulder_pan_joint",) * 6,
                positions=(0.0,) * 6,
                canonical_joint_names=JOINT_NAMES,
            )

    def test_missing_canonical_joint_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "missing"):
            canonical_joint_positions_from_names(
                names=JOINT_NAMES[:-1],
                positions=(0.0,) * 5,
                canonical_joint_names=JOINT_NAMES,
            )

    def test_unnamed_state_requires_exact_canonical_length(self):
        with self.assertRaisesRegex(ValueError, "exactly 6"):
            canonical_joint_positions_from_names(
                names=(),
                positions=(0.0,) * 5,
                canonical_joint_names=JOINT_NAMES,
            )


if __name__ == "__main__":
    unittest.main()
