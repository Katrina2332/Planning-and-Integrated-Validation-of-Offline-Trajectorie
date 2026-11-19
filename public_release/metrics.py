# -*- coding: utf-8 -*-
"""Manual、Raw Bi-RRT 与 Shortcut 路径的统一指标计算。"""

import numpy as np

from collision import path_collision_free, rotation_distance_deg
from robot_scene import fk_tcp
from ruckig_planner import ruckig_path


def tcp_points(robot, path):
    """计算路径所有关节关键点对应的 TCP 坐标。"""
    return np.asarray([fk_tcp(robot, q).t for q in path])


def tcp_path_length(robot, path):
    xyz = tcp_points(robot, path)
    if len(xyz) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(xyz, axis=0), axis=1)))


def joint_space_length(path):
    if len(path) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))


def total_joint_motion_deg(path):
    if len(path) < 2:
        return 0.0
    return float(np.rad2deg(np.sum(np.abs(np.diff(path, axis=0)))))


def _tcp_rotations(robot, path):
    """返回关键点对应的 TCP 旋转矩阵。"""
    return [np.asarray(fk_tcp(robot, q).R, dtype=float) for q in path]


def total_tcp_rotation_deg(robot, path):
    """计算整条关键点路径的累计 TCP 姿态变化。"""
    rotations = _tcp_rotations(robot, path)
    return float(sum(
        rotation_distance_deg(rotations[i], rotations[i + 1])
        for i in range(len(rotations) - 1)
    ))


def max_segment_tcp_rotation_deg(robot, path):
    """计算相邻关键点之间最大的 TCP 姿态变化。"""
    rotations = _tcp_rotations(robot, path)
    if len(rotations) < 2:
        return 0.0
    return float(max(
        rotation_distance_deg(rotations[i], rotations[i + 1])
        for i in range(len(rotations) - 1)
    ))


def ruckig_acceleration_metrics(path):
    """计算相同 Ruckig 约束下的平均与累计关节加速度负荷。"""
    result = ruckig_path(path)
    time_s = np.asarray(result["time"], dtype=float)
    acceleration = np.asarray(result["qdd"], dtype=float)
    duration = float(result["duration"])

    summed_acceleration_squared = np.sum(acceleration ** 2, axis=1)
    acceleration_squared_integral = float(np.trapezoid(
        summed_acceleration_squared, time_s
    ))
    acceleration_rms_rad_s2 = float(np.sqrt(
        acceleration_squared_integral / (duration * acceleration.shape[1])
    ))
    return {
        "ruckig_time_s": duration,
        "accel_rms_deg_s2": float(np.rad2deg(acceleration_rms_rad_s2)),
        "accel_ias_rad2_s3": acceleration_squared_integral,
    }


def keypoint_metrics(
    robot, name, path, obstacles, attached_bodies=None
):
    """按统一口径计算一条关键点路径的对比指标。"""
    acceleration_metrics = ruckig_acceleration_metrics(path)
    return {
        "name": name,
        "nodes": len(path),
        "tcp_m": tcp_path_length(robot, path),
        "joint_rad": joint_space_length(path),
        "joint_motion_deg": total_joint_motion_deg(path),
        "tcp_rotation_deg": total_tcp_rotation_deg(robot, path),
        "max_segment_rotation_deg": max_segment_tcp_rotation_deg(
            robot, path
        ),
        **acceleration_metrics,
        "collision_free": path_collision_free(
            robot, path, obstacles, attached_bodies
        ),
    }


def print_kinematic_metrics(name, q_samples, result, edge_indices=None, times=None):
    """输出关节裕度和奇异性最差采样点的位置。"""
    margin_index = result["worst_margin_index"]
    sigma_index = result["worst_sigma_index"]

    def location(index):
        if times is not None:
            return f"样本 {index}，t={times[index]:.3f} s"
        if edge_indices is None:
            return f"样本 {index}"
        edge = edge_indices[index]
        return f"样本 {index}，{'起点' if edge < 0 else f'边 {edge + 1}'}"

    joint = int(np.argmin(result["joint_margin_deg"][margin_index])) + 1
    print(
        f"\n{name} 运动学检测（{len(q_samples)} 个采样点，"
        f"Jacobian 平移参考长度 {result['characteristic_length_m']:g} m）："
    )
    print(
        f"  最小关节裕度：{result['margin_deg'][margin_index]:.3f} deg，"
        f"J{joint}，{location(margin_index)}"
    )
    print(
        f"  最小奇异值：{result['sigma_min'][sigma_index]:.6g}，"
        f"条件数：{result['condition'][sigma_index]:.6g}，"
        f"可操作度：{result['manipulability'][sigma_index]:.6g}，"
        f"{location(sigma_index)}"
    )
    print(f"  最差奇异构型 / deg：{np.round(np.rad2deg(q_samples[sigma_index]), 3)}")
    if np.any(result["margin_deg"] < 0):
        print(f"  [警告] {np.count_nonzero(result['margin_deg'] < 0)} 个采样点超出关节限位。")


def print_metrics_table(rows):
    """打印领导展示所需的四项核心指标。"""
    print("\n" + "-" * 103)
    print(
        f"{'方案':<28}{'TCP路径长度(m)':>18}{'关节空间长度(rad)':>22}"
        f"{'六轴总运动量(deg)':>22}{'理论时间(s)':>15}"
    )
    print("-" * 103)

    for m in rows:
        print(
            f"{m['name']:<28}{m['tcp_m']:>18.4f}"
            f"{m['joint_rad']:>22.4f}{m['joint_motion_deg']:>22.1f}"
            f"{m['ruckig_time_s']:>15.3f}"
        )

    print("-" * 103)

    if len(rows) == 2:
        baseline, optimized = rows
        fields = [
            ("TCP路径长度", "tcp_m"),
            ("关节空间长度", "joint_rad"),
            ("六轴总运动量", "joint_motion_deg"),
            ("理论时间", "ruckig_time_s"),
        ]
        print("优化方案相对常规示教的改善：")
        for label, field in fields:
            base_value = baseline[field]
            improvement = (
                0.0
                if base_value <= 1e-12
                else (1.0 - optimized[field] / base_value) * 100.0
            )
            print(f"  {label}: {improvement:+.1f}%")


def print_dynamic_metrics_table(rows):
    """打印体现平均动态载荷和累计加速度负担的两个指标。"""
    print("\n" + "-" * 82)
    print(
        f"{'方案':<28}{'六轴等效加速度RMS(deg/s^2)':>30}"
        f"{'累计加速度平方积分IAS(rad^2/s^3)':>24}"
    )
    print("-" * 82)
    for metrics in rows:
        print(
            f"{metrics['name']:<28}"
            f"{metrics['accel_rms_deg_s2']:>30.2f}"
            f"{metrics['accel_ias_rad2_s3']:>24.3f}"
        )
    print("-" * 82)

    if len(rows) == 2:
        baseline, optimized = rows
        print("优化方案的动态负荷改善：")
        for label, field in (
            ("六轴等效加速度 RMS", "accel_rms_deg_s2"),
            ("累计加速度平方积分 IAS", "accel_ias_rad2_s3"),
        ):
            improvement = (
                1.0 - optimized[field] / baseline[field]
            ) * 100.0
            print(f"  {label}: +{improvement:.1f}%")
