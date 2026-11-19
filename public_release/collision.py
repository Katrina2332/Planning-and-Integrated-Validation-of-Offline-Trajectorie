# -*- coding: utf-8 -*-
"""机器人构型、边和路径的离散碰撞检测。"""

import numpy as np
import spatialgeometry as sg

from config import (
    EDGE_CHECK_STEP,
    ENABLE_ORIENTATION_CONSTRAINT,
    MAX_EDGE_TCP_ROTATION_DEG,
)
from robot_scene import fk_tcp, get_attached_body_pose


def _edge_samples(q1, q2):
    """按统一关节空间步长生成一条边的离散样本。"""
    q1 = np.asarray(q1, dtype=float)
    q2 = np.asarray(q2, dtype=float)
    distance = np.linalg.norm(q2 - q1)
    n = max(2, int(np.ceil(distance / EDGE_CHECK_STEP)) + 1)
    return q1 + np.linspace(0.0, 1.0, n)[:, None] * (q2 - q1)


def configuration_in_collision(robot, q, obstacles, attached_bodies=None):
    """检查 Robot / Gripper / Workpiece 与环境障碍物的碰撞。"""
    if any(robot.iscollided(q, obstacle) for obstacle in obstacles):
        return True

    for body_spec in attached_bodies or []:
        if not body_spec.get("collision_enabled", True):
            continue

        # spatialgeometry/Coal 使用完整姿态进行旋转几何体碰撞，并非 AABB 近似。
        body = sg.Cuboid(
            body_spec["size"],
            pose=get_attached_body_pose(robot, q, body_spec),
        )
        if any(body.iscollided(obstacle) for obstacle in obstacles):
            return True

    # 当前版本按需求优先检查 Attached Body vs Environment，未增加附件自碰撞。
    return False


def rotation_distance_deg(R1, R2):
    """计算两个旋转矩阵的标准 SO(3) 角距离（degree）。"""
    relative = np.asarray(R1, dtype=float).T @ np.asarray(R2, dtype=float)
    cosine = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.rad2deg(np.arccos(cosine)))


def trajectory_tcp_rotation_deg(robot, q_samples):
    """累计一组相邻关节样本对应的 TCP 姿态旋转量。"""
    q_samples = np.asarray(q_samples, dtype=float)
    if len(q_samples) < 2:
        return 0.0

    rotations = [np.asarray(fk_tcp(robot, q).R, dtype=float) for q in q_samples]
    return float(sum(
        rotation_distance_deg(rotations[i], rotations[i + 1])
        for i in range(len(rotations) - 1)
    ))


def edge_orientation_valid(
    robot,
    q1,
    q2,
    max_rotation_deg=MAX_EDGE_TCP_ROTATION_DEG,
    q_samples=None,
):
    """检查一条边内离散相邻 TCP 姿态变化的累计量。"""
    if not ENABLE_ORIENTATION_CONSTRAINT:
        return True
    samples = _edge_samples(q1, q2) if q_samples is None else q_samples
    return trajectory_tcp_rotation_deg(robot, samples) <= max_rotation_deg + 1e-9


def edge_collision_free(robot, q1, q2, obstacles, attached_bodies=None):
    """按固定关节空间步长离散检查一条边。"""
    for q in _edge_samples(q1, q2):
        if configuration_in_collision(robot, q, obstacles, attached_bodies):
            return False
    return True


def edge_valid(robot, q1, q2, obstacles, attached_bodies=None):
    """统一检查一条边的碰撞安全性和可选 TCP 姿态约束。"""
    samples = _edge_samples(q1, q2)
    if any(
        configuration_in_collision(robot, q, obstacles, attached_bodies)
        for q in samples
    ):
        return False
    return edge_orientation_valid(robot, q1, q2, q_samples=samples)


def path_collision_free(robot, path, obstacles, attached_bodies=None):
    """检查路径的所有关键点与相邻边是否无碰撞。"""
    path = np.asarray(path, dtype=float)
    if any(
        configuration_in_collision(robot, q, obstacles, attached_bodies)
        for q in path
    ):
        return False
    return all(
        edge_collision_free(
            robot, path[i], path[i + 1], obstacles, attached_bodies
        )
        for i in range(len(path) - 1)
    )


def path_valid(robot, path, obstacles, attached_bodies=None):
    """检查路径每一段均满足碰撞与可选 TCP 姿态约束。"""
    path = np.asarray(path, dtype=float)
    return all(
        edge_valid(robot, path[i], path[i + 1], obstacles, attached_bodies)
        for i in range(len(path) - 1)
    )
