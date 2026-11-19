"""AR2010 分阶段抓放离线演示。"""

import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

try:
    import spatialgeometry as sg
    from collision import path_valid
    from config import (
        ConfigError, RUCKIG_MAX_ACC, RUCKIG_MAX_JERK, RUCKIG_MAX_VEL,
        _validate_and_normalize, load_config,
    )
    from robot_scene import get_attached_body_pose, get_joint_limits
    from ruckig_planner import approximate_max_jerk, sampled_trajectory_valid
except ModuleNotFoundError as exc:
    raise unittest.SkipTest(f"机器人环境未安装离线规划依赖：{exc}") from exc

import main_pick_place as demo
from main_pick_place import (
    build_pick_place_playback, contact_segment_valid, plan_pick_place,
    viewer_frame_state,
)


class PickPlaceConfig(unittest.TestCase):
    def test_demo_targets_are_independent_between_loads(self):
        first = load_config()
        second = load_config()
        first["task"]["pick_place_demo"]["pre_pick_joint_deg"][0] += 1.0
        self.assertNotEqual(
            first["task"]["pick_place_demo"]["pre_pick_joint_deg"][0],
            second["task"]["pick_place_demo"]["pre_pick_joint_deg"][0],
        )

    def test_demo_joint_targets_are_validated(self):
        config = load_config()
        self.assertEqual(config["task"]["pick_place_demo"]["pre_pick_joint_deg"].shape, (6,))
        config["task"]["pick_place_demo"]["pre_pick_joint_deg"] = [0.0]
        with self.assertRaises(ConfigError) as context:
            _validate_and_normalize(config, config["config_path"])
        self.assertEqual(context.exception.code, "INVALID_TASK_CONFIG")
        self.assertEqual(context.exception.field, "task.pick_place_demo.pre_pick_joint_deg")
        self.assertEqual(context.exception.path.name, "tasks.yaml")


class PickPlacePipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result = plan_pick_place()

    def test_stage_states_and_continuity(self):
        phases = self.result["phases"]
        self.assertEqual(list(phases), ["approach", "transport", "retreat"])
        self.assertEqual(
            [body["name"] for body in phases["approach"]["attached_bodies"]],
            ["gripper"],
        )
        self.assertEqual(
            [body["name"] for body in phases["transport"]["attached_bodies"]],
            ["gripper", "workpiece"],
        )
        self.assertEqual(
            [body["name"] for body in phases["retreat"]["attached_bodies"]],
            ["gripper"],
        )
        for first, second in (("approach", "transport"), ("transport", "retreat")):
            np.testing.assert_allclose(
                phases[first]["trajectory"]["q"][-1],
                phases[second]["trajectory"]["q"][0], atol=1e-8,
            )

    def test_all_phases_are_finite_within_limits_and_collision_checked(self):
        robot = self.result["robot"]
        obstacles = self.result["obstacles"]
        qlim = get_joint_limits(robot)
        for name, phase in self.result["phases"].items():
            trajectory = phase["trajectory"]
            q, qd, qdd = (trajectory[key] for key in ("q", "qd", "qdd"))
            self.assertTrue(np.all(np.isfinite(q)))
            self.assertTrue(np.all(np.isfinite(qd)))
            self.assertTrue(np.all(np.isfinite(qdd)))
            self.assertTrue(np.all(q >= qlim[0] - 1e-8))
            self.assertTrue(np.all(q <= qlim[1] + 1e-8))
            self.assertTrue(np.all(np.max(np.abs(qd), axis=0) <= RUCKIG_MAX_VEL + 1e-8))
            self.assertTrue(np.all(np.max(np.abs(qdd), axis=0) <= RUCKIG_MAX_ACC + 1e-8))
            self.assertTrue(np.all(
                approximate_max_jerk(trajectory["time"], qdd) <= RUCKIG_MAX_JERK + 1e-6
            ))
            np.testing.assert_allclose(q[0], phase["path"][0], atol=1e-8)
            np.testing.assert_allclose(q[-1], phase["path"][-1], atol=1e-8)
            if name == "transport":
                self.assertTrue(path_valid(
                    robot, phase["path"], obstacles, phase["attached_bodies"]
                ))
                self.assertTrue(sampled_trajectory_valid(
                    robot, trajectory, obstacles, phase["attached_bodies"]
                ))
            else:
                self.assertTrue(contact_segment_valid(
                    robot, phase["path"], trajectory, obstacles,
                    phase["attached_bodies"][0], phase["world_workpiece"],
                ))

    def test_only_gripper_contact_is_allowed(self):
        robot = self.result["robot"]
        phase = self.result["phases"]["approach"]
        gripper = phase["attached_bodies"][0]
        q_pick = phase["path"][-1]
        gripper_shape = sg.Cuboid(
            gripper["size"], pose=get_attached_body_pose(robot, q_pick, gripper)
        )
        self.assertTrue(gripper_shape.iscollided(phase["world_workpiece"]))
        self.assertTrue(contact_segment_valid(
            robot, phase["path"], phase["trajectory"],
            self.result["obstacles"], gripper, phase["world_workpiece"],
        ))
        blocker = sg.Cuboid([3.0, 3.0, 3.0], pose=get_attached_body_pose(
            robot, q_pick, gripper
        ))
        self.assertFalse(contact_segment_valid(
            robot, phase["path"], phase["trajectory"],
            self.result["obstacles"] + [blocker], gripper,
            phase["world_workpiece"],
        ))
        with patch.object(
            robot, "iscollided",
            side_effect=lambda q, obstacle: obstacle is phase["world_workpiece"],
        ):
            self.assertFalse(contact_segment_valid(
                robot, phase["path"], phase["trajectory"],
                self.result["obstacles"], gripper, phase["world_workpiece"],
            ))

    def test_viewer_shows_static_attached_static_workpiece(self):
        phases = self.result["phases"]
        playback = build_pick_place_playback(self.result)
        transport_start = len(phases["approach"]["trajectory"]["q"])
        retreat_start = transport_start + len(phases["transport"]["trajectory"]["q"])
        self.assertTrue(np.all(np.diff(playback["time"]) > 0.0))
        self.assertEqual(playback["phase"][transport_start], "transport")
        self.assertEqual(playback["phase"][retreat_start], "retreat")
        for index in (transport_start, retreat_start):
            np.testing.assert_allclose(
                playback["q"][index - 1], playback["q"][index], atol=1e-8,
            )
            before = viewer_frame_state(self.result, playback, index - 1)
            after = viewer_frame_state(self.result, playback, index)
            np.testing.assert_allclose(
                before["workpiece_pose"].A, after["workpiece_pose"].A,
                atol=1e-8,
            )
        before_pick = viewer_frame_state(self.result, playback, 0)
        after_pick = viewer_frame_state(self.result, playback, transport_start)
        during_transport = viewer_frame_state(
            self.result, playback, retreat_start - 1
        )
        after_place = viewer_frame_state(self.result, playback, retreat_start)
        after_retreat = viewer_frame_state(
            self.result, playback, len(playback["q"]) - 1
        )
        self.assertFalse(before_pick["workpiece_attached"])
        self.assertTrue(after_pick["workpiece_attached"])
        self.assertTrue(during_transport["workpiece_attached"])
        self.assertFalse(after_place["workpiece_attached"])
        np.testing.assert_allclose(
            before_pick["workpiece_pose"].A,
            viewer_frame_state(self.result, playback, transport_start - 1)[
                "workpiece_pose"
            ].A,
        )
        np.testing.assert_allclose(
            after_place["workpiece_pose"].A,
            after_retreat["workpiece_pose"].A,
        )
        self.assertFalse(np.allclose(
            after_pick["workpiece_pose"].A,
            during_transport["workpiece_pose"].A,
        ))


class PickPlaceExportGate(unittest.TestCase):
    def test_failed_phase_does_not_publish_stage_trajectories(self):
        config = load_config()
        config["output"]["directory"] = Path("output")
        with patch.object(demo, "load_config", return_value=config), \
                patch.object(demo, "plan_pick_place",
                             side_effect=RuntimeError("接近阶段复核失败。")), \
                patch.object(Path, "mkdir"), \
                patch.object(Path, "unlink", autospec=True) as unlink, \
                patch.object(demo, "export_ruckig_trajectory") as export:
            with self.assertRaisesRegex(RuntimeError, "接近阶段复核失败"):
                demo.main()
        self.assertEqual(unlink.call_count, 3)
        export.assert_not_called()


if __name__ == "__main__":
    unittest.main()
