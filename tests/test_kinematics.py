"""FR5 model and the checks of recorded poses against recorded joints."""

import unittest

import numpy as np

from handeye.geometry import fr5_from_matrix, inverse, parameters_to_matrix, rotation_angle_degrees
from handeye.kinematics import JOINT_LOWER_DEG, JOINT_UPPER_DEG, check_convention, fk, ik, row_consistency


def recorded(order: str, seed: int = 7, poses: int = 8):
    rng = np.random.default_rng(seed)
    # Unknown but constant base and flange frame offsets must not matter.
    base = parameters_to_matrix(np.array([0.0, 0.0, 0.02, 0.0, 0.0, 0.001]))
    flange = parameters_to_matrix(np.array([0.0, 0.0, np.pi / 2, 0.0, 0.0, 0.0]))
    joints = [rng.uniform(JOINT_LOWER_DEG, JOINT_UPPER_DEG).round(3).tolist() for _ in range(poses)]
    return joints, [fr5_from_matrix(base @ fk(q) @ flange, order) for q in joints]


class KinematicsTest(unittest.TestCase):
    def test_forward_kinematics_zero_pose(self):
        # UR-style stretched arm: x = a2 + a3, y = -(d4 + d6), z = d1 - d5.
        np.testing.assert_allclose(fk([0] * 6)[:3, 3], [-0.820, -0.202, 0.050], atol=1e-9)

    def test_inverse_kinematics_round_trip(self):
        rng = np.random.default_rng(3)
        target = fk([20, -80, 100, -110, -80, 40])
        solution = ik(target, rng)
        self.assertIsNotNone(solution)
        delta = inverse(target) @ fk(solution)
        # ik() stops once every component is below 0.1 um / 0.1 urad
        self.assertLess(np.linalg.norm(delta[:3, 3]), 1e-6)
        self.assertLess(rotation_angle_degrees(delta[:3, :3]), 1e-5)

    def test_handeye_convention_passes(self):
        result = check_convention(*recorded("zyx"))
        self.assertEqual(result["status"], "pass", result["reasons"])
        self.assertLess(result["leave_one_out"]["max_mm"], 0.01)

    def test_wrong_convention_fails(self):
        result = check_convention(*recorded("xyz"))
        self.assertEqual((result["status"], result["best_convention"]), ("fail", "Rx@Ry@Rz"))

    def test_repeated_or_too_few_poses_are_insufficient_not_pass(self):
        joints, poses = recorded("zyx", poses=1)
        self.assertEqual(check_convention(joints * 6, poses * 6)["status"], "insufficient_data")
        self.assertEqual(check_convention(*recorded("zyx", poses=4))["status"], "insufficient_data")

    def test_row_consistency_flags_a_mistyped_pose(self):
        joints, poses = recorded("zyx", poses=10)
        poses[6][1] += 10.0  # y_mm typed 10 mm off
        suspect = row_consistency(list(range(1, 11)), joints, poses)["suspect_rows"]
        self.assertEqual([row["row"] for row in suspect], [7])


if __name__ == "__main__":
    unittest.main()
