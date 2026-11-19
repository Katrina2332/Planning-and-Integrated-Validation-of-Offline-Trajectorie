"""AR2010 默认任务的无 GUI 离线规划回归。"""

import random
import unittest

import numpy as np

try:
    from collision import edge_valid
    from config import (
        RANDOM_SEED,
        RUCKIG_MAX_ACC,
        RUCKIG_MAX_JERK,
        RUCKIG_MAX_VEL,
    )
    from manual_path import build_fallback_manual_path, build_manual_path
    from resource_dictionary import get_scene_info
    from robot_scene import (
        create_attached_bodies,
        create_scene,
        get_joint_limits,
        get_start_goal,
        load_robot,
    )
    from rrt_planner import birrt_connect
    from ruckig_planner import (
        approximate_max_jerk,
        ruckig_path,
        sampled_trajectory_valid,
    )
    from shortcut import shortcut_path
except ModuleNotFoundError as exc:
    raise unittest.SkipTest(f"机器人环境未安装离线规划依赖：{exc}") from exc


class OfflinePlanningRegression(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = load_robot()
        cls.q_start, cls.q_goal = get_start_goal()

    def test_default_ar2010_pipeline_remains_valid(self):
        random.seed(RANDOM_SEED)
        np.random.seed(RANDOM_SEED)
        robot = self.robot
        q_start, q_goal = self.q_start, self.q_goal
        qlim = get_joint_limits(robot)
        _, obstacles = create_scene(robot, q_start, q_goal)
        attached_bodies = create_attached_bodies()

        manual_path = build_manual_path(
            robot, q_start, q_goal, obstacles, attached_bodies
        )
        rrt_path, iterations = birrt_connect(
            robot, q_start, q_goal, obstacles, qlim, attached_bodies
        )
        shortcut_result = shortcut_path(
            robot, rrt_path, obstacles, attached_bodies
        )
        if manual_path is None:
            manual_path = build_fallback_manual_path(rrt_path)
        result = ruckig_path(shortcut_result)

        self.assertGreaterEqual(iterations, 0)
        self.assertGreaterEqual(len(manual_path), 2)
        self.assertEqual(robot.n, 6)
        self.assertEqual(result["q"].shape[1], robot.n)
        self.assertEqual(result["qd"].shape, result["q"].shape)
        self.assertEqual(result["qdd"].shape, result["q"].shape)
        self.assertTrue(np.all(np.isfinite(result["q"])))
        np.testing.assert_allclose(result["q"][0], q_start, atol=1e-8)
        np.testing.assert_allclose(result["q"][-1], q_goal, atol=1e-8)
        self.assertTrue(sampled_trajectory_valid(
            robot, result, obstacles, attached_bodies
        ))
        self.assertTrue(np.all(
            np.max(np.abs(result["qd"]), axis=0) <= RUCKIG_MAX_VEL + 1e-8
        ))
        self.assertTrue(np.all(
            np.max(np.abs(result["qdd"]), axis=0) <= RUCKIG_MAX_ACC + 1e-8
        ))
        jerk = approximate_max_jerk(result["time"], result["qdd"])
        self.assertTrue(np.all(jerk <= RUCKIG_MAX_JERK + 1e-6))

    def test_selected_scene_builds_current_collision_objects(self):
        robot = self.robot
        q_start, q_goal = self.q_start, self.q_goal
        empty = get_scene_info("empty_scene")
        obstacle = get_scene_info("obstacle_scene")
        empty_specs, empty_bodies = create_scene(
            robot, q_start, q_goal, scene_config=empty
        )
        obstacle_specs, obstacle_bodies = create_scene(
            robot, q_start, q_goal, scene_config=obstacle
        )
        self.assertEqual(len(empty_specs), 3)
        self.assertEqual(len(obstacle_specs), 3)
        self.assertEqual(len(empty_bodies), 0)
        self.assertEqual(len(obstacle_bodies), 1)
        self.assertEqual(
            obstacle_specs[2]["name"], "central_safety_block"
        )
        attached_bodies = create_attached_bodies()
        self.assertTrue(edge_valid(
            robot, q_start, q_goal, empty_bodies, attached_bodies
        ))
        self.assertFalse(edge_valid(
            robot, q_start, q_goal, obstacle_bodies, attached_bodies
        ))


if __name__ == "__main__":
    unittest.main()
