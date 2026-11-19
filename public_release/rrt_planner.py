# -*- coding: utf-8 -*-
"""六维关节空间中的 Bi-RRT Connect 路径规划。"""

import random

import numpy as np

from collision import configuration_in_collision, edge_valid
from config import (
    GOAL_BIAS,
    MAX_ITERATIONS,
    RRT_CONNECT_MAX_EXTENSIONS,
    RRT_STEP,
)


class Node:
    """RRT 树节点，保存关节构型与父节点索引。"""

    def __init__(self, q, parent=None):
        self.q = np.asarray(q, dtype=float)
        self.parent = parent


def sample_q(goal_q, qlim):
    if random.random() < GOAL_BIAS:
        return goal_q.copy()
    return np.random.uniform(qlim[0], qlim[1])


def nearest_index(tree, q):
    all_q = np.vstack([node.q for node in tree])
    return int(np.argmin(np.linalg.norm(all_q - q, axis=1)))


def steer(q_from, q_to):
    delta = q_to - q_from
    distance = np.linalg.norm(delta)
    if distance <= RRT_STEP:
        return q_to.copy()
    return q_from + delta / distance * RRT_STEP


def within_limits(q, qlim):
    return bool(np.all(q >= qlim[0]) and np.all(q <= qlim[1]))


def extend_once(
    tree, q_target, robot, obstacles, qlim, attached_bodies=None
):
    i_near = nearest_index(tree, q_target)
    q_near = tree[i_near].q
    q_new = steer(q_near, q_target)

    if not within_limits(q_new, qlim):
        return None
    if not edge_valid(
        robot, q_near, q_new, obstacles, attached_bodies
    ):
        return None

    tree.append(Node(q_new, parent=i_near))
    return len(tree) - 1


def connect(tree, q_target, robot, obstacles, qlim, attached_bodies=None):
    last_index = None
    for _ in range(RRT_CONNECT_MAX_EXTENSIONS):
        new_index = extend_once(
            tree, q_target, robot, obstacles, qlim, attached_bodies
        )
        if new_index is None:
            return last_index, False
        last_index = new_index
        if np.linalg.norm(tree[new_index].q - q_target) < 1e-8:
            return new_index, True
    return last_index, False


def trace_to_root(tree, index):
    path = []
    while index is not None:
        path.append(tree[index].q.copy())
        index = tree[index].parent
    return path


def birrt_connect(
    robot, q_start, q_goal, obstacles, qlim, attached_bodies=None
):
    """使用与原 Demo 相同的双向 RRT-Connect 算法搜索路径。"""
    if configuration_in_collision(
        robot, q_start, obstacles, attached_bodies
    ):
        raise RuntimeError("起始姿态碰撞。")
    if configuration_in_collision(
        robot, q_goal, obstacles, attached_bodies
    ):
        raise RuntimeError("目标姿态碰撞。")

    if edge_valid(robot, q_start, q_goal, obstacles, attached_bodies):
        print("[提示] 起终点可以直接连接，当前场景没有形成有效 RRT 问题。")
        return np.vstack([q_start, q_goal]), 0

    tree_a = [Node(q_start)]
    tree_b = [Node(q_goal)]
    root_a = "start"

    for iteration in range(1, MAX_ITERATIONS + 1):
        q_rand = sample_q(tree_b[0].q, qlim)
        index_a = extend_once(
            tree_a, q_rand, robot, obstacles, qlim, attached_bodies
        )

        if index_a is not None:
            index_b, reached = connect(
                tree_b,
                tree_a[index_a].q,
                robot,
                obstacles,
                qlim,
                attached_bodies,
            )
            if reached:
                path_a = trace_to_root(tree_a, index_a)
                path_b = trace_to_root(tree_b, index_b)

                if root_a == "start":
                    path = list(reversed(path_a)) + path_b[1:]
                else:
                    path = list(reversed(path_b)) + path_a[1:]

                path = np.asarray(path)
                if np.linalg.norm(path[0] - q_start) > np.linalg.norm(
                    path[-1] - q_start
                ):
                    path = path[::-1]
                return path, iteration

        tree_a, tree_b = tree_b, tree_a
        root_a = "goal" if root_a == "start" else "start"

    raise RuntimeError("Bi-RRT Connect 未找到路径。")
