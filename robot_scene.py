# -*- coding: utf-8 -*-
"""Yaskawa Motoman AR2010 机器人加载、运动学基础信息与场景创建。"""

import numpy as np
import spatialgeometry as sg
from roboticstoolbox.models.URDF.URDFRobot import URDFRobot
from spatialmath import SE3

from config import (
    ATTACHED_BODY_CONFIGS,
    COLLISION_SAFETY_MARGIN,
    FALLBACK_JOINT_LIMITS_DEG,
    Q_GOAL_DEG,
    Q_START_DEG,
    ROBOT_BASE_RPY_DEG,
    ROBOT_BASE_TRANSLATION,
    SCENE_OBJECT_CONFIGS,
    TCP_LINK,
    URDF_PATH,
)


def load_robot():
    """加载资源字典选定的机器人 URDF 模型。"""
    robot = URDFRobot(str(URDF_PATH))
    robot.base = SE3.Trans(ROBOT_BASE_TRANSLATION) * SE3.RPY(
        ROBOT_BASE_RPY_DEG, unit="deg", order="xyz"
    )
    return robot


def fk_tcp(robot, q):
    """计算指定关节姿态下 tool0 的正运动学。"""
    return robot.fkine(q, end=TCP_LINK)


def get_joint_limits(robot):
    """读取关节限位，并对无效限位使用配置中的回退范围。"""
    limits = np.asarray(robot.qlim, dtype=float)
    if limits.shape != (2, robot.n):
        return np.deg2rad(FALLBACK_JOINT_LIMITS_DEG.copy())

    lower = limits[0].copy()
    upper = limits[1].copy()
    invalid = ~np.isfinite(lower) | ~np.isfinite(upper) | (upper <= lower)
    fallback = np.deg2rad(FALLBACK_JOINT_LIMITS_DEG)
    lower[invalid] = fallback[0, invalid]
    upper[invalid] = fallback[1, invalid]
    return np.vstack([lower, upper])


def get_start_goal():
    """把配置中的角度制起点、终点转换为弧度。"""
    q_start = np.deg2rad(Q_START_DEG)
    q_goal = np.deg2rad(Q_GOAL_DEG)
    return q_start, q_goal


def _tcp_relative_pose(offset, rpy_deg):
    """由 TCP 坐标系下的 xyz + RPY(xyz) 构造固定 SE(3) 变换。"""
    return SE3.Trans(np.asarray(offset, dtype=float)) * SE3.RPY(
        np.asarray(rpy_deg, dtype=float), unit="deg", order="xyz"
    )


def create_attached_bodies():
    """创建随 TCP 刚性运动的夹具和工件规格。"""
    # 这里只描述基础几何和 TCP 相对位姿；碰撞判断统一放在 collision.py。
    return [
        {
            "name": body["name"],
            "size": body["size_m"].copy(),
            "T_tcp_body": _tcp_relative_pose(
                body["tcp_offset_m"], body["tcp_rpy_deg"]
            ),
            "collision_enabled": body["collision_enabled"],
        }
        for body in ATTACHED_BODY_CONFIGS
    ]


def get_attached_body_pose(robot, q, body_spec):
    """计算 T_base_body(q) = T_base_tcp(q) @ T_tcp_body。"""
    return fk_tcp(robot, q) * body_spec["T_tcp_body"]


def create_scene(robot, q_start, q_goal, scene_config=None):
    """根据选中场景创建物体及启用的碰撞障碍物。"""
    if scene_config is None:
        scene_objects = SCENE_OBJECT_CONFIGS
        safety_margin = COLLISION_SAFETY_MARGIN
    else:
        scene_objects = scene_config["objects"]
        safety_margin = scene_config["collision_safety_margin_m"]
    scene_specs = []
    for item in scene_objects:
        pose = SE3.Trans(item["center_m"]) * SE3.RPY(
            item["rpy_deg"], unit="deg", order="xyz"
        )
        scene_specs.append({
            "name": item["name"],
            "size": item["size_m"].copy(),
            "center": item["center_m"].copy(),
            "rpy_deg": item["rpy_deg"].copy(),
            "pose": pose,
            "collision_enabled": item["collision_enabled"],
            "color": item.get("color"),
        })

    obstacles = [
        sg.Cuboid(
            np.asarray(spec["size"], dtype=float) + 2.0 * safety_margin,
            pose=spec["pose"],
        )
        for spec in scene_specs
        if spec["collision_enabled"]
    ]
    return scene_specs, obstacles
