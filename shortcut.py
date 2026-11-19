# -*- coding: utf-8 -*-
"""RRT 路径的随机 Shortcut 简化。"""

import random

import numpy as np

from collision import edge_valid
from config import SHORTCUT_ATTEMPTS


def shortcut_path(robot, path, obstacles, attached_bodies=None):
    """随机连接非相邻节点，并删除可被安全跨过的中间节点。"""
    result = [q.copy() for q in np.asarray(path, dtype=float)]

    for _ in range(SHORTCUT_ATTEMPTS):
        if len(result) <= 2:
            break
        i = random.randint(0, len(result) - 3)
        j = random.randint(i + 2, len(result) - 1)

        if edge_valid(
            robot, result[i], result[j], obstacles, attached_bodies
        ):
            result = result[:i + 1] + result[j:]

    return np.asarray(result)
