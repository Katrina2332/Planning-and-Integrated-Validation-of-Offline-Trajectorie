# -*- coding: utf-8 -*-
"""
Yaskawa Motoman AR2010 机械臂 Demo 主流程：
Manual Teach -> RRT-Connect -> Shortcut -> Ruckig -> Collision Check
-> Metrics -> Controller Program CSV -> Visualization
"""

import argparse
import random
import time

import numpy as np

from collision import edge_valid, path_collision_free
from config import (
    ENABLE_GRIPPER_COLLISION,
    ENABLE_ORIENTATION_CONSTRAINT,
    ENABLE_WORKPIECE_COLLISION,
    MAX_EDGE_TCP_ROTATION_DEG,
    MAX_RUCKIG_SEGMENT_TCP_ROTATION_DEG,
    RANDOM_SEED,
    ROBOT_DISPLAY_NAME,
    SHORTCUT_ENABLED,
    WORKPIECE_SIZE,
    load_config,
)
from exporter import (
    export_controller_program,
    export_keypoints,
    export_metrics_comparison,
    export_ruckig_trajectory,
)
from kinematic_constraints import evaluate_kinematics, sample_path_edges
from manual_path import build_fallback_manual_path, build_manual_path
from metrics import (
    keypoint_metrics,
    max_segment_tcp_rotation_deg,
    print_dynamic_metrics_table,
    print_kinematic_metrics,
    print_metrics_table,
)
from robot_scene import (
    create_attached_bodies,
    create_scene,
    fk_tcp,
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
from visualization import plot_kinematic_safety, plot_path_comparison, plot_ruckig_profiles


def validate_inputs(robot, q_start, q_goal, qlim, config):
    """在规划前检查机器人自由度、任务关节值和关节限位。"""
    expected_dof = config["robot"]["expected_dof"]
    if robot.n != expected_dof:
        raise ValueError(
            f"配置期望 {expected_dof} 自由度，但 URDF 模型提供 {robot.n} 自由度。"
        )
    if q_start.shape != (robot.n,) or q_goal.shape != (robot.n,):
        raise ValueError("起点和终点的关节数量必须与机器人自由度一致。")
    if not np.all(np.isfinite(q_start)) or not np.all(np.isfinite(q_goal)):
        raise ValueError("起点和终点不能包含 NaN 或无穷值。")
    for name, q in (("起点", q_start), ("终点", q_goal)):
        if np.any(q < qlim[0]) or np.any(q > qlim[1]):
            raise ValueError(f"{name}超出 URDF 关节限位。")


def main(show_viewer=False, viewer_port=8080):
    """按课题 Demo 的阶段顺序执行完整流程。"""
    config = load_config()
    output = config["output"]
    output_dir = output["directory"]
    output_files = output["files"]

    # 1. 初始化随机种子与输出目录
    random.seed(RANDOM_SEED)
    np.random.seed(RANDOM_SEED)
    if output["save_csv"] or output["save_figures"]:
        output_dir.mkdir(parents=True, exist_ok=True)
    for key in ("controller_program", "ruckig_trajectory"):
        (output_dir / output_files[key]).unlink(missing_ok=True)

    print("=" * 88)
    print(f"{ROBOT_DISPLAY_NAME}：传统示教 vs Bi-RRT Connect + Shortcut + Ruckig")
    print("=" * 88)

    # 2. 加载机器人和场景
    robot = load_robot()
    q_start, q_goal = get_start_goal()
    qlim = get_joint_limits(robot)
    validate_inputs(robot, q_start, q_goal, qlim, config)
    obstacle_specs, obstacles = create_scene(
        robot, q_start, q_goal, scene_config=config["scene"]
    )
    attached_bodies = create_attached_bodies()

    print(
        "配置："
        f"robot_id={config['ids']['robot_id']}, "
        f"scene_id={config['ids']['scene_id']}, "
        f"task_id={config['ids']['task_id']}"
    )
    print(f"机器人：{ROBOT_DISPLAY_NAME}")
    print(f"自由度：{robot.n}")
    print("\n起始 TCP：", np.round(fk_tcp(robot, q_start).t, 4))
    print("目标 TCP：", np.round(fk_tcp(robot, q_goal).t, 4))
    print(f"Gripper collision enabled: {ENABLE_GRIPPER_COLLISION}")
    print(f"Workpiece collision enabled: {ENABLE_WORKPIECE_COLLISION}")
    print(f"Workpiece size: {WORKPIECE_SIZE} m")
    print(f"Orientation constraint enabled: {ENABLE_ORIENTATION_CONSTRAINT}")
    print(f"Max edge TCP rotation: {MAX_EDGE_TCP_ROTATION_DEG:g} deg")
    print(
        "Max dense Ruckig segment TCP rotation: "
        f"{MAX_RUCKIG_SEGMENT_TCP_ROTATION_DEG:g} deg"
    )

    print("\n场景物体：")
    for spec in obstacle_specs:
        print(
            f"  {spec['name']} | center={np.round(spec['center'], 3)} m | "
            f"size={spec['size']} m | collision={spec['collision_enabled']}"
        )

    # 3. Manual Teach：构建人工示教参考路径
    manual_path = build_manual_path(
        robot, q_start, q_goal, obstacles, attached_bodies
    )
    if manual_path is not None:
        print(f"\n常规人工示教 MoveJ 路径：{len(manual_path)} 个关键点")

    # 4. Bi-RRT Connect：搜索关节空间无碰撞路径
    print(
        "\n起点 -> 终点直接运动是否安全：",
        edge_valid(robot, q_start, q_goal, obstacles, attached_bodies),
    )

    t0 = time.perf_counter()
    rrt_path, iterations = birrt_connect(
        robot, q_start, q_goal, obstacles, qlim, attached_bodies
    )
    rrt_time = time.perf_counter() - t0
    print(
        f"\nBi-RRT 找到路径：{len(rrt_path)} 个节点 | "
        f"{iterations} 次迭代 | {rrt_time * 1000:.1f} ms"
    )
    print(
        "Raw RRT 最大单段 TCP 姿态变化："
        f"{max_segment_tcp_rotation_deg(robot, rrt_path):.2f} deg"
    )

    # 5. Shortcut：删除可被安全跨过的中间节点
    if SHORTCUT_ENABLED:
        shortcut_result = shortcut_path(
            robot, rrt_path, obstacles, attached_bodies
        )
    else:
        shortcut_result = rrt_path.copy()
        print("Shortcut 已通过配置关闭，沿用 Raw RRT 路径。")
    print(f"Shortcut 后：{len(shortcut_result)} 个关键点")
    print(
        "Shortcut 最大单段 TCP 姿态变化："
        f"{max_segment_tcp_rotation_deg(robot, shortcut_result):.2f} deg"
    )
    print(
        "Shortcut 路径无碰撞：",
        path_collision_free(
            robot, shortcut_result, obstacles, attached_bodies
        ),
    )

    if manual_path is None:
        manual_path = build_fallback_manual_path(rrt_path)

    # 6. Ruckig：对 Shortcut 相邻关键点逐段生成离线轨迹
    print("\n" + "=" * 88)
    print("第四阶段：Ruckig jerk-limited 离线轨迹生成")
    print("=" * 88)

    t0 = time.perf_counter()
    ruckig_result = ruckig_path(shortcut_result)
    ruckig_calc_time = time.perf_counter() - t0

    print(f"Ruckig 最终轨迹：{len(ruckig_result['q'])} 个采样点")
    print(f"理论运行时间：{ruckig_result['duration']:.3f} s")
    print(f"离线计算耗时：{ruckig_calc_time * 1000:.1f} ms")

    # 7. Collision Check：复核最终 Ruckig 稠密轨迹
    ruckig_safe = sampled_trajectory_valid(
        robot, ruckig_result, obstacles, attached_bodies
    )
    print("Ruckig 稠密轨迹附件碰撞与逐段姿态复核：", ruckig_safe)

    if not ruckig_safe:
        raise RuntimeError(
            "Ruckig 最终轨迹复核失败：出现碰撞或姿态约束不满足；"
            "未发布最终 CSV。"
        )

    # 独立运动学检测：几何路径覆盖插值边，最终轨迹覆盖全部时间采样点。
    for name, path in (("Raw RRT", rrt_path), ("Shortcut", shortcut_result)):
        q_samples, edge_indices = sample_path_edges(path)
        kinematics = evaluate_kinematics(robot, q_samples, qlim)
        print_kinematic_metrics(name, q_samples, kinematics, edge_indices=edge_indices)
    final_kinematics = evaluate_kinematics(robot, ruckig_result["q"], qlim)
    print_kinematic_metrics(
        "Final Ruckig", ruckig_result["q"], final_kinematics,
        times=ruckig_result["time"],
    )

    # 8. Metrics：常规示教与优化结果使用相同 Ruckig 约束公平比较
    rows = [
        keypoint_metrics(
            robot,
            "常规人工示教 MoveJ",
            manual_path,
            obstacles,
            attached_bodies,
        ),
        keypoint_metrics(
            robot,
            "优化 Bi-RRT + Shortcut",
            shortcut_result,
            obstacles,
            attached_bodies,
        ),
    ]
    print_metrics_table(rows)
    print_dynamic_metrics_table(rows)

    # 9. Ruckig 六轴峰值（jerk 为采样数据的数值估计）
    peak_vel = np.max(np.abs(ruckig_result["qd"]), axis=0)
    peak_acc = np.max(np.abs(ruckig_result["qdd"]), axis=0)
    peak_jerk = approximate_max_jerk(
        ruckig_result["time"], ruckig_result["qdd"]
    )

    print("\nRuckig 六轴峰值：")
    for i in range(robot.n):
        print(
            f"  J{i + 1}: "
            f"v={np.rad2deg(peak_vel[i]):7.2f} deg/s | "
            f"a={np.rad2deg(peak_acc[i]):8.2f} deg/s^2 | "
            f"jerk≈{np.rad2deg(peak_jerk[i]):9.2f} deg/s^3"
        )

    # 10. Controller Program CSV 与轨迹、关键点导出
    if output["save_csv"]:
        export_controller_program(
            shortcut_result,
            ruckig_result["segments"],
            output_dir / output_files["controller_program"],
        )
        export_ruckig_trajectory(
            ruckig_result,
            output_dir / output_files["ruckig_trajectory"],
        )
        export_keypoints(
            manual_path, output_dir / output_files["manual_keypoints"]
        )
        export_keypoints(
            rrt_path, output_dir / output_files["rrt_keypoints"]
        )
        export_keypoints(
            shortcut_result,
            output_dir / output_files["shortcut_keypoints"],
        )
        export_metrics_comparison(
            rows, output_dir / output_files["metrics"]
        )

    # 11. Visualization：路径对比与 Ruckig 六轴曲线
    show_matplotlib = output["show_visualization"] and not show_viewer
    if output["save_figures"] or show_matplotlib:
        plot_path_comparison(
            robot,
            manual_path,
            rrt_path,
            shortcut_result,
            ruckig_result,
            obstacle_specs,
            output_dir=output_dir,
            save_figures=output["save_figures"],
            show_visualization=show_matplotlib,
            figure_dpi=output["figure_dpi"],
            filename=output_files["path_figure"],
        )
        plot_ruckig_profiles(
            ruckig_result,
            output_dir=output_dir,
            save_figures=output["save_figures"],
            show_visualization=show_matplotlib,
            figure_dpi=output["figure_dpi"],
            filenames=output_files,
        )
        plot_kinematic_safety(
            ruckig_result["time"], final_kinematics,
            output_dir=output_dir,
            save_figures=output["save_figures"],
            show_visualization=show_matplotlib,
            figure_dpi=output["figure_dpi"],
        )

    print("\n完成。输出目录：", output_dir)
    if output["save_csv"]:
        print("核心工业中间输出：")
        print(f"  {output_files['controller_program']}")
        print(f"  {output_files['ruckig_trajectory']}")

    if show_viewer:
        from native_viewer import show_native_viewer

        show_native_viewer(
            config, robot, obstacle_specs, attached_bodies,
            manual_path, rrt_path, shortcut_result, ruckig_result,
            port=viewer_port,
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AR2010 离线轨迹规划")
    parser.add_argument("--viewer", action="store_true", help="使用 Viser 查看并回放轨迹")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    main(show_viewer=args.viewer, viewer_port=args.port)
 
