"""关节裕度与 Jacobian 指标的确定性测试。"""

import unittest

import numpy as np

try:
    from kinematic_constraints import evaluate_kinematics, sample_path_edges
except ModuleNotFoundError as exc:
    raise unittest.SkipTest(f"机器人环境未安装离线规划依赖：{exc}") from exc


class _Robot:
    n = 6

    def jacob0(self, q, end=None):
        return np.diag([2.0, 1.0, 1.0, 1.0, 1.0, 0.5])


class _SingularRobot(_Robot):
    def jacob0(self, q, end=None):
        return np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 0.0])


class KinematicConstraintsTest(unittest.TestCase):
    def test_metrics_locate_limit_and_singularity_candidates(self):
        q = np.zeros((2, 6))
        q[1, 0] = 0.9
        qlim = np.vstack([-np.ones(6), np.ones(6)])
        result = evaluate_kinematics(_Robot(), q, qlim, characteristic_length_m=2.0)

        self.assertEqual(result["worst_margin_index"], 1)
        self.assertAlmostEqual(result["margin_deg"][1], np.rad2deg(0.1))
        self.assertAlmostEqual(result["sigma_min"][0], 0.5)
        self.assertAlmostEqual(result["condition"][0], 2.0)
        self.assertAlmostEqual(result["manipulability"][0], 0.125)

        q[1, 0] = 1.1
        result = evaluate_kinematics(_Robot(), q, qlim)
        self.assertLess(result["margin_deg"][1], 0)

    def test_exact_singularity_has_infinite_condition_number(self):
        qlim = np.vstack([-np.ones(6), np.ones(6)])
        result = evaluate_kinematics(_SingularRobot(), np.zeros((1, 6)), qlim)
        self.assertEqual(result["sigma_min"][0], 0.0)
        self.assertTrue(np.isinf(result["condition"][0]))
        self.assertEqual(result["manipulability"][0], 0.0)

    def test_path_sampling_includes_edge_interiors(self):
        path = np.zeros((2, 6))
        path[1, 0] = np.deg2rad(9)
        samples, edges = sample_path_edges(path)

        self.assertEqual(len(samples), 4)
        np.testing.assert_allclose(samples[:, 0], np.deg2rad([0, 3, 6, 9]))
        np.testing.assert_array_equal(edges, [-1, 0, 0, 0])

    def test_invalid_limits_are_rejected(self):
        qlim = np.vstack([-np.ones(6), np.ones(6)])
        qlim[1, 0] = -1.0
        with self.assertRaisesRegex(ValueError, "qlim"):
            evaluate_kinematics(_Robot(), np.zeros((1, 6)), qlim)


if __name__ == "__main__":
    unittest.main()
