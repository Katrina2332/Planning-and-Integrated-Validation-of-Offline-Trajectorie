# -*- coding: utf-8 -*-
"""人工示教参考路径及其回退路径构造。"""

import numpy as np

from collision import path_collision_free
from config import MANUAL_CANDIDATES_DEG


def build_manual_path(
    robot, q_start, q_goal, obstacles, attached_bodies=None
):
    """按原顺序返回第一条无碰撞的人工示教候选路径。"""
    for candidate in MANUAL_CANDIDATES_DEG:
        path = np.vstack([
            q_start,
            np.deg2rad(np.asarray(candidate, dtype=float)),
            q_goal,
        ])
        if path_collision_free(robot, path, obstacles, attached_bodies):
            return path
    return None


def build_fallback_manual_path(rrt_path):
    """没有有效人工候选时，在 RRT 相邻节点间插入中点。"""
    expanded = [rrt_path[0]]
    for i in range(len(rrt_path) - 1):
        q0, q1 = rrt_path[i], rrt_path[i + 1]
        expanded.append(0.5 * (q0 + q1))
        expanded.append(q1)
    return np.asarray(expanded)
