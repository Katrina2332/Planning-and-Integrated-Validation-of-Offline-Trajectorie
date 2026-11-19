# -*- coding: utf-8 -*-
"""关节限位裕度与 Jacobian 奇异性离线评估。"""

import numpy as np

from collision import _edge_samples
from config import TCP_LINK


def sample_path_edges(path):
    """沿现有碰撞检查的步长采样全部关节空间直线边。"""
    path = np.asarray(path, dtype=float)
    samples = [path[0:1]]
    edge_indices = [-1]
    for i in range(len(path) - 1):
        edge = _edge_samples(path[i], path[i + 1])[1:]
        samples.append(edge)
        edge_indices.extend([i] * len(edge))
    return np.vstack(samples), np.asarray(edge_indices, dtype=int)


def evaluate_kinematics(robot, q_samples, qlim, characteristic_length_m=1.0):
    """返回逐点限位裕度和按特征长度缩放的 Jacobian 指标。"""
    q_samples = np.asarray(q_samples, dtype=float)
    qlim = np.asarray(qlim, dtype=float)
    if (q_samples.ndim != 2 or q_samples.shape[1] != robot.n
            or len(q_samples) == 0 or not np.all(np.isfinite(q_samples))):
        raise ValueError("q_samples 必须是非空的有限关节构型数组。")
    if (qlim.shape != (2, robot.n) or not np.all(np.isfinite(qlim))
            or np.any(qlim[1] <= qlim[0])):
        raise ValueError("qlim 必须是有效的两行关节限位数组。")
    if not np.isfinite(characteristic_length_m) or characteristic_length_m <= 0:
        raise ValueError("characteristic_length_m 必须是正的有限米数。")

    margins = np.minimum(q_samples - qlim[0], qlim[1] - q_samples)
    sigma_min, condition, manipulability = [], [], []
    for q in q_samples:
        jacobian = np.asarray(robot.jacob0(q, end=TCP_LINK), dtype=float).copy()
        if jacobian.shape != (6, robot.n) or not np.all(np.isfinite(jacobian)):
            raise ValueError("Jacobian 必须是有限的 6×n 矩阵。")
        jacobian[:3] /= characteristic_length_m
        singular_values = np.linalg.svd(jacobian, compute_uv=False)
        smallest = float(singular_values[-1])
        sigma_min.append(smallest)
        condition.append(float(singular_values[0] / smallest) if smallest > 0 else np.inf)
        manipulability.append(float(np.prod(singular_values)))

    return {
        "characteristic_length_m": float(characteristic_length_m),
        "margin_deg": np.rad2deg(np.min(margins, axis=1)),
        "joint_margin_deg": np.rad2deg(margins),
        "sigma_min": np.asarray(sigma_min),
        "condition": np.asarray(condition),
        "manipulability": np.asarray(manipulability),
        "worst_margin_index": int(np.argmin(np.min(margins, axis=1))),
        "worst_sigma_index": int(np.argmin(sigma_min)),
    }
