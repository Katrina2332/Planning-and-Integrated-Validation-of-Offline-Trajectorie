# -*- coding: utf-8 -*-
"""AR2010 假设抓取姿态的离线接近、搬运和撤离流程演示。"""

import argparse
import random

import numpy as np
import spatialgeometry as sg

from collision import path_valid
from config import (
    RUCKIG_MAX_ACC, RUCKIG_MAX_JERK, RUCKIG_MAX_VEL,
    SHORTCUT_ENABLED, load_config,
)
from exporter import export_ruckig_trajectory
from main import validate_inputs
from robot_scene import (
    create_attached_bodies, create_scene, get_attached_body_pose,
    get_joint_limits, load_robot,
)
from rrt_planner import birrt_connect
from ruckig_planner import (
    approximate_max_jerk, ruckig_path, sampled_trajectory_valid,
)
from shortcut import shortcut_path


def contact_segment_valid(
    robot, path, trajectory, obstacles, gripper, world_workpiece
):
    """只允许夹具与静止工件接触，其余环境碰撞照常检查。"""
    return (
        path_valid(robot, path, obstacles + [world_workpiece])
        and path_valid(robot, path, obstacles, [gripper])
        and sampled_trajectory_valid(
            robot, trajectory, obstacles + [world_workpiece]
        )
        and sampled_trajectory_valid(robot, trajectory, obstacles, [gripper])
    )


def _trajectory_within_limits(trajectory, path, qlim):
    q, qd, qdd = (trajectory[key] for key in ("q", "qd", "qdd"))
    time = trajectory["time"]
    return bool(
        len(time) >= 2 and np.all(np.isfinite(time))
        and np.all(np.diff(time) > 0.0)
        and all(np.all(np.isfinite(values)) for values in (q, qd, qdd))
        and np.allclose(q[0], path[0], atol=1e-8)
        and np.allclose(q[-1], path[-1], atol=1e-8)
        and np.all(q >= qlim[0] - 1e-8)
        and np.all(q <= qlim[1] + 1e-8)
        and np.all(np.max(np.abs(qd), axis=0) <= RUCKIG_MAX_VEL + 1e-8)
        and np.all(np.max(np.abs(qdd), axis=0) <= RUCKIG_MAX_ACC + 1e-8)
        and np.all(
            approximate_max_jerk(time, qdd) <= RUCKIG_MAX_JERK + 1e-6
        )
    )


def plan_pick_place(config=None):
    """复用当前运输段，在两端增加带工件状态切换的接近和撤离。"""
    config = load_config() if config is None else config
    demo = config["task"].get("pick_place_demo")
    if demo is None:
        raise ValueError("当前任务未定义 pick_place_demo 演示姿态。")
    seed = config["planner"]["random_seed"]
    random.seed(seed)
    np.random.seed(seed)

    robot = load_robot()
    qlim = get_joint_limits(robot)
    q_pick = np.deg2rad(config["task"]["start_joint_deg"])
    q_place = np.deg2rad(config["task"]["goal_joint_deg"])
    q_pre_pick = np.deg2rad(demo["pre_pick_joint_deg"])
    q_post_place = np.deg2rad(demo["post_place_joint_deg"])
    validate_inputs(robot, q_pick, q_place, qlim, config)
    validate_inputs(robot, q_pre_pick, q_post_place, qlim, config)
    scene_specs, obstacles = create_scene(
        robot, q_pick, q_place, scene_config=config["scene"]
    )
    bodies = {body["name"]: body for body in create_attached_bodies()}
    gripper, workpiece = bodies["gripper"], bodies["workpiece"]
    world_pick = sg.Cuboid(
        workpiece["size"], pose=get_attached_body_pose(robot, q_pick, workpiece)
    )
    world_place = sg.Cuboid(
        workpiece["size"], pose=get_attached_body_pose(robot, q_place, workpiece)
    )

    approach_path = np.vstack([q_pre_pick, q_pick])
    approach_trajectory = ruckig_path(approach_path)
    if not (_trajectory_within_limits(approach_trajectory, approach_path, qlim)
            and contact_segment_valid(
                robot, approach_path, approach_trajectory, obstacles,
                gripper, world_pick,
            )):
        raise RuntimeError("接近阶段复核失败。")

    raw_path, _ = birrt_connect(
        robot, q_pick, q_place, obstacles, qlim, [gripper, workpiece]
    )
    transport_path = shortcut_path(
        robot, raw_path, obstacles, [gripper, workpiece]
    ) if SHORTCUT_ENABLED else raw_path
    transport_trajectory = ruckig_path(transport_path)
    if not (_trajectory_within_limits(transport_trajectory, transport_path, qlim)
            and path_valid(robot, transport_path, obstacles, [gripper, workpiece])
            and sampled_trajectory_valid(
                robot, transport_trajectory, obstacles, [gripper, workpiece]
            )):
        raise RuntimeError("搬运阶段复核失败。")

    retreat_path = np.vstack([q_place, q_post_place])
    retreat_trajectory = ruckig_path(retreat_path)
    if not (_trajectory_within_limits(retreat_trajectory, retreat_path, qlim)
            and contact_segment_valid(
                robot, retreat_path, retreat_trajectory, obstacles,
                gripper, world_place,
            )):
        raise RuntimeError("撤离阶段复核失败。")

    return {
        "robot": robot,
        "scene_specs": scene_specs,
        "obstacles": obstacles,
        "phases": {
            "approach": {
                "path": approach_path, "trajectory": approach_trajectory,
                "attached_bodies": [gripper], "world_workpiece": world_pick,
            },
            "transport": {
                "path": transport_path, "trajectory": transport_trajectory,
                "attached_bodies": [gripper, workpiece],
                "world_workpiece": None,
            },
            "retreat": {
                "path": retreat_path, "trajectory": retreat_trajectory,
                "attached_bodies": [gripper], "world_workpiece": world_place,
            },
        },
    }


def build_pick_place_playback(result, pause_s=0.5):
    """拼接三段采样，保留抓取与释放的边界帧。"""
    q_blocks, time_blocks, phase_names = [], [], []
    offset = 0.0
    for index, (name, phase) in enumerate(result["phases"].items()):
        trajectory = phase["trajectory"]
        if index:
            offset += pause_s
        q_blocks.append(trajectory["q"])
        time_blocks.append(trajectory["time"] + offset)
        phase_names.extend([name] * len(trajectory["q"]))
        offset = float(time_blocks[-1][-1])
    return {
        "q": np.vstack(q_blocks),
        "time": np.concatenate(time_blocks),
        "phase": phase_names,
    }


def viewer_frame_state(result, playback, index):
    """返回当前帧中夹具与工件的姿态及工件附着状态。"""
    robot = result["robot"]
    phases = result["phases"]
    phase = playback["phase"][index]
    q = playback["q"][index]
    gripper = phases["approach"]["attached_bodies"][0]
    workpiece = phases["transport"]["attached_bodies"][1]
    if phase == "approach":
        q_workpiece = phases["approach"]["path"][-1]
    elif phase == "retreat":
        q_workpiece = phases["retreat"]["path"][0]
    else:
        q_workpiece = q
    return {
        "q": q,
        "phase": phase,
        "gripper_pose": get_attached_body_pose(robot, q, gripper),
        "workpiece_pose": get_attached_body_pose(robot, q_workpiece, workpiece),
        "workpiece_attached": phase == "transport",
    }


def main(show_viewer=False, port=8080):
    config = load_config()
    output_dir = config["output"]["directory"] / "pick_place_demo"
    if config["output"]["save_csv"]:
        output_dir.mkdir(parents=True, exist_ok=True)
    filenames = {
        name: output_dir / f"{name}_ruckig_trajectory.csv"
        for name in ("approach", "transport", "retreat")
    }
    for path in filenames.values():
        path.unlink(missing_ok=True)

    result = plan_pick_place(config)
    print("接近 → 夹持工件 → 搬运 → 释放工件 → 撤离：全部离线复核通过。")
    for name, phase in result["phases"].items():
        print(f"  {name}: {phase['trajectory']['duration']:.3f} s")
        if config["output"]["save_csv"]:
            export_ruckig_trajectory(phase["trajectory"], filenames[name])
    if config["output"]["save_csv"]:
        print("阶段轨迹目录：", output_dir)
    if show_viewer:
        from native_viewer import show_pick_place_viewer

        show_pick_place_viewer(config, result, port=port)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AR2010 分阶段抓放离线演示")
    parser.add_argument("--viewer", action="store_true", help="用 Viser 回放抓放过程")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    main(show_viewer=args.viewer, port=args.port)
