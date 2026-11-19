# -*- coding: utf-8 -*-
"""Shortcut 相邻关键点的逐段 Ruckig 离线轨迹生成。"""

import numpy as np
from ruckig import InputParameter, Result, Ruckig, Trajectory

from collision import configuration_in_collision, trajectory_tcp_rotation_deg
from config import (
    ENABLE_ORIENTATION_CONSTRAINT,
    MAX_RUCKIG_SEGMENT_TCP_ROTATION_DEG,
    RUCKIG_MAX_ACC,
    RUCKIG_MAX_JERK,
    RUCKIG_MAX_VEL,
    RUCKIG_SAMPLE_DT,
)


def ruckig_segment(q_start, q_goal, sample_dt=RUCKIG_SAMPLE_DT):
    """生成一段起止速度、加速度均为零的离线轨迹。"""
    dof = len(q_start)
    inp = InputParameter(dof)

    inp.current_position = q_start.tolist()
    inp.current_velocity = np.zeros(dof).tolist()
    inp.current_acceleration = np.zeros(dof).tolist()

    inp.target_position = q_goal.tolist()
    inp.target_velocity = np.zeros(dof).tolist()
    inp.target_acceleration = np.zeros(dof).tolist()

    inp.max_velocity = RUCKIG_MAX_VEL.tolist()
    inp.max_acceleration = RUCKIG_MAX_ACC.tolist()
    inp.max_jerk = RUCKIG_MAX_JERK.tolist()

    otg = Ruckig(dof)
    trajectory = Trajectory(dof)
    result = otg.calculate(inp, trajectory)

    if result == Result.ErrorInvalidInput or "Error" in str(result):
        raise RuntimeError(f"Ruckig 计算失败：{result}")

    duration = float(trajectory.duration)
    n = max(2, int(np.ceil(duration / sample_dt)) + 1)
    times = np.linspace(0.0, duration, n)

    q_list, qd_list, qdd_list = [], [], []
    for t in times:
        q, qd, qdd = trajectory.at_time(float(t))
        q_list.append(q)
        qd_list.append(qd)
        qdd_list.append(qdd)

    return {
        "duration": duration,
        "time": np.asarray(times, dtype=float),
        "q": np.asarray(q_list, dtype=float),
        "qd": np.asarray(qd_list, dtype=float),
        "qdd": np.asarray(qdd_list, dtype=float),
    }


def ruckig_path(keypoints, sample_dt=RUCKIG_SAMPLE_DT):
    """对相邻关键点逐段规划并拼接 time、q、qd、qdd。"""
    keypoints = np.asarray(keypoints, dtype=float)

    all_t, all_q, all_qd, all_qdd = [], [], [], []
    segment_info = []
    time_offset = 0.0

    for i in range(len(keypoints) - 1):
        seg = ruckig_segment(keypoints[i], keypoints[i + 1], sample_dt)

        t = seg["time"] + time_offset
        q = seg["q"]
        qd = seg["qd"]
        qdd = seg["qdd"]

        if i == 0:
            sample_start_index = 0
        else:
            # 后续段的起点就是已拼接轨迹的最后一个样本。
            sample_start_index = sum(len(values) for values in all_q) - 1

        if i > 0:
            t, q, qd, qdd = t[1:], q[1:], qd[1:], qdd[1:]

        sample_end_index = sample_start_index + len(seg["q"]) - 1

        speed_ratio = np.max(np.abs(seg["qd"]) / RUCKIG_MAX_VEL)
        speed_proxy_pct = 100.0 * float(np.clip(speed_ratio, 0.0, 1.0))

        segment_info.append({
            "segment": i + 1,
            "from_index": i,
            "to_index": i + 1,
            "duration_s": float(seg["duration"]),
            "speed_proxy_pct": speed_proxy_pct,
            "sample_start_index": sample_start_index,
            "sample_end_index": sample_end_index,
        })

        all_t.append(t)
        all_q.append(q)
        all_qd.append(qd)
        all_qdd.append(qdd)
        time_offset += seg["duration"]

    return {
        "time": np.concatenate(all_t),
        "q": np.vstack(all_q),
        "qd": np.vstack(all_qd),
        "qdd": np.vstack(all_qdd),
        "duration": float(time_offset),
        "segments": segment_info,
    }


def ruckig_path_duration(path):
    """用同一套 Ruckig 约束累计各段理论运行时间。"""
    total = 0.0
    for i in range(len(path) - 1):
        # 只取 duration，采样间隔可以较粗，不影响 duration。
        total += ruckig_segment(path[i], path[i + 1], sample_dt=0.1)["duration"]
    return float(total)


def sampled_trajectory_collision_free(
    robot, q_trajectory, obstacles, attached_bodies=None
):
    """对最终 Ruckig 稠密轨迹重新进行构型碰撞检查。"""
    # Ruckig 的多轴同步轨迹不一定等于关节线性插值，因此必须重新检查。
    return not any(
        configuration_in_collision(robot, q, obstacles, attached_bodies)
        for q in q_trajectory
    )


def sampled_trajectory_orientation_valid(robot, ruckig_result):
    """逐关键点段复核 Ruckig 稠密轨迹的累计 TCP 姿态变化。"""
    if not ENABLE_ORIENTATION_CONSTRAINT:
        return True

    q_trajectory = ruckig_result["q"]
    for segment in ruckig_result["segments"]:
        start = segment["sample_start_index"]
        end = segment["sample_end_index"]
        if trajectory_tcp_rotation_deg(
            robot, q_trajectory[start:end + 1]
        ) > MAX_RUCKIG_SEGMENT_TCP_ROTATION_DEG + 1e-9:
            return False
    return True


def sampled_trajectory_valid(
    robot, ruckig_result, obstacles, attached_bodies=None
):
    """复核最终 Ruckig 轨迹的附件碰撞与逐段 TCP 姿态约束。"""
    return sampled_trajectory_collision_free(
        robot, ruckig_result["q"], obstacles, attached_bodies
    ) and sampled_trajectory_orientation_valid(robot, ruckig_result)


def approximate_max_jerk(time_array, qdd):
    """用 diff(qdd) / diff(time) 数值估计各轴最大 jerk。"""
    if len(time_array) < 2:
        return np.zeros(qdd.shape[1])

    dt = np.diff(time_array)
    da = np.diff(qdd, axis=0)
    valid = dt > 1e-9
    if not np.any(valid):
        return np.zeros(qdd.shape[1])

    jerk = da[valid] / dt[valid, None]
    return np.max(np.abs(jerk), axis=0)
