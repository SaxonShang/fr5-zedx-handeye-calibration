"""The FR5 convention check must accept handeye.py's convention and flag others."""

import unittest

import numpy as np

from handeye import parameters_to_matrix
from sim import fr5_check
from sim.synthetic import fr5_from_matrix


def records(order: str, seed: int = 7) -> list[dict]:
    rng = np.random.default_rng(seed)
    # Unknown but constant base and flange frame offsets must not matter.
    base = parameters_to_matrix(np.array([0.0, 0.0, 0.02, 0.0, 0.0, 0.001]))
    flange = parameters_to_matrix(np.array([0.0, 0.0, np.pi / 2, 0.0, 0.0, 0.0]))
    result = []
    for _ in range(8):
        joints = rng.uniform(-120.0, 120.0, 6).round(3).tolist()
        pose = fr5_from_matrix(base @ fr5_check.fk(joints) @ flange, order)
        result.append({"joints_deg": joints, "flange_pose_mm_deg": pose})
    return result


class Fr5ConventionCheckTest(unittest.TestCase):
    def test_forward_kinematics_zero_pose(self):
        # All joints at zero: UR-style stretched arm, flange at
        # x = a2 + a3, y = -(d4 + d6), z = d1 - d5.
        np.testing.assert_allclose(fr5_check.fk([0] * 6)[:3, 3], [-0.820, -0.202, 0.050], atol=1e-9)

    def test_handeye_convention_passes(self):
        result = fr5_check.analyse(records("zyx"))
        self.assertTrue(result["handeye_convention_ok"])
        self.assertLess(result["fits"][fr5_check.HANDEYE_CONVENTION]["rms_mm"], 0.01)

    def test_wrong_convention_is_flagged(self):
        result = fr5_check.analyse(records("xyz"))
        self.assertFalse(result["handeye_convention_ok"])
        self.assertEqual(result["best_convention"], "Rx@Ry@Rz")


if __name__ == "__main__":
    unittest.main()
