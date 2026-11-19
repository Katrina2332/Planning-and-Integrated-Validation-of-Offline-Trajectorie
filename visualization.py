# -*- coding: utf-8 -*-
"""路径对比与 Ruckig 六轴位置、速度、加速度可视化。"""

import matplotlib.pyplot as plt
import numpy as np

from config import (
    FIGURE_DPI,
    OUTPUT_DIR,
    OUTPUT_FILES,
    SAVE_FIGURES,
    SHOW_VISUALIZATION,
)
from metrics import tcp_points


def draw_box_wireframe(ax, center, size, color=None, pose=None):
    """在三维坐标轴中绘制长方体障碍物线框。"""
    hx, hy, hz = np.asarray(size, dtype=float) / 2.0
    corners = np.array([
        [-hx, -hy, -hz],
        [-hx, -hy, +hz],
        [-hx, +hy, -hz],
        [-hx, +hy, +hz],
        [+hx, -hy, -hz],
        [+hx, -hy, +hz],
        [+hx, +hy, -hz],
        [+hx, +hy, +hz],
    ])
    if pose is None:
        corners += np.asarray(center)
    else:
        transform = np.asarray(pose.A, dtype=float)
        corners = corners @ transform[:3, :3].T + transform[:3, 3]

    edges = [
        (0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
        (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7),
    ]
    for i, j in edges:
        ax.plot(
            [corners[i, 0], corners[j, 0]],
            [corners[i, 1], corners[j, 1]],
            [corners[i, 2], corners[j, 2]],
            linewidth=1,
            color=color,
        )


def plot_path_comparison(
    robot,
    manual_path,
    rrt_path,
    shortcut_result,
    ruckig_result,
    obstacle_specs,
    output_dir=OUTPUT_DIR,
    save_figures=SAVE_FIGURES,
    show_visualization=SHOW_VISUALIZATION,
    figure_dpi=FIGURE_DPI,
    filename=None,
):
    """绘制四种路径的 TCP 轨迹与障碍物。"""
    manual_xyz = tcp_points(robot, manual_path)
    rrt_xyz = tcp_points(robot, rrt_path)
    shortcut_xyz = tcp_points(robot, shortcut_result)
    ruckig_xyz = tcp_points(robot, ruckig_result["q"])

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(*manual_xyz.T, "--o", label="Conventional taught MoveJ")
    '''ax.plot(*rrt_xyz.T, ":.", label="Raw Bi-RRT")
    ax.plot(*shortcut_xyz.T, "-o", linewidth=2.0, label="RRT + Shortcut")'''
    ax.plot(*ruckig_xyz.T, "-", linewidth=2.8, label="Final Ruckig trajectory")

    for spec in obstacle_specs:
        draw_box_wireframe(
            ax,
            spec["center"],
            spec["size"],
            color=spec.get("color"),
            pose=spec.get("pose"),
        )

    ax.set_xlabel("X / m")
    ax.set_ylabel("Y / m")
    ax.set_zlabel("Z / m")
    ax.set_title("Yaskawa AR2010: Conventional vs Optimized Transport")
    ax.legend()
    plt.tight_layout()
    if save_figures:
        plt.savefig(
            output_dir / (filename or OUTPUT_FILES["path_figure"]),
            dpi=figure_dpi,
        )
    if show_visualization:
        plt.show()
    plt.close(fig)


def plot_ruckig_profiles(
    result,
    output_dir=OUTPUT_DIR,
    save_figures=SAVE_FIGURES,
    show_visualization=SHOW_VISUALIZATION,
    figure_dpi=FIGURE_DPI,
    filenames=None,
):
    """分别绘制六轴位置、速度与加速度曲线。"""
    t = result["time"]
    names = filenames or OUTPUT_FILES
    series = [
        (
            np.rad2deg(result["q"]),
            "Joint angle / deg",
            "Ruckig Joint Position",
            names["joint_position_figure"],
        ),
        (
            np.rad2deg(result["qd"]),
            "Joint velocity / deg/s",
            "Ruckig Joint Velocity",
            names["joint_velocity_figure"],
        ),
        (
            np.rad2deg(result["qdd"]),
            "Joint acceleration / deg/s²",
            "Ruckig Joint Acceleration",
            names["joint_acceleration_figure"],
        ),
    ]

    for values, ylabel, title, filename in series:
        fig = plt.figure(figsize=(10, 6))
        ax = fig.add_subplot(111)
        for j in range(values.shape[1]):
            ax.plot(t, values[:, j], label=f"J{j + 1}")
        ax.set_xlabel("Time / s")
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(True)
        ax.legend(ncol=3)
        plt.tight_layout()
        if save_figures:
            plt.savefig(output_dir / filename, dpi=figure_dpi)
        if show_visualization:
            plt.show()
        plt.close(fig)


def plot_kinematic_safety(
    times, result, output_dir=OUTPUT_DIR, save_figures=SAVE_FIGURES,
    show_visualization=SHOW_VISUALIZATION, figure_dpi=FIGURE_DPI,
):
    """绘制最终 Ruckig 轨迹的限位裕度和缩放 Jacobian 最小奇异值。"""
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for ax, key, worst, ylabel in (
        (axes[0], "margin_deg", "worst_margin_index", "Minimum joint margin / deg"),
        (axes[1], "sigma_min", "worst_sigma_index", "Scaled Jacobian minimum singular value"),
    ):
        ax.plot(times, result[key])
        ax.plot(times[result[worst]], result[key][result[worst]], "ro")
        ax.set_ylabel(ylabel)
        ax.grid(True)
    axes[0].axhline(0.0, color="red", linestyle="--", linewidth=0.8)
    axes[1].set_xlabel("Time / s")
    fig.suptitle(
        "Ruckig kinematic safety (Jacobian translation scale: "
        f"{result['characteristic_length_m']:g} m)"
    )
    plt.tight_layout()
    if save_figures:
        fig.savefig(output_dir / "kinematic_safety.png", dpi=figure_dpi)
    if show_visualization:
        plt.show()
    plt.close(fig)
