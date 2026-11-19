# -*- coding: utf-8 -*-
"""控制器中间程序、Ruckig 稠密轨迹和关键点 CSV 导出。"""

import csv

import numpy as np


def export_controller_program(keypoints, segment_info, output_file):
    """
    导出 point + J1~J6 + motion_mode + 速度代理值 + 预计段时间。

    speed_proxy_pct 只是 Ruckig 峰值关节速度 / Demo 最大关节速度得到的代理值，
    不是 ABB、Yaskawa、新时达等任何厂商控制器的真实速度百分比。
    """
    keypoints_deg = np.rad2deg(keypoints)
    joint_headers = [
        f"J{index}_deg" for index in range(1, keypoints_deg.shape[1] + 1)
    ]

    with open(output_file, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        writer.writerow([
            "point",
            *joint_headers,
            "motion_mode",
            "speed_proxy_pct",
            "segment_duration_s",
        ])

        writer.writerow(["P0", *keypoints_deg[0], "START", "", ""])

        for i in range(1, len(keypoints)):
            seg = segment_info[i - 1]
            writer.writerow([
                f"P{i}",
                *keypoints_deg[i],
                "MOVJ",
                f"{seg['speed_proxy_pct']:.1f}",
                f"{seg['duration_s']:.4f}",
            ])


def export_ruckig_trajectory(result, output_file):
    """导出完整 Ruckig time、q、qd、qdd 采样数据。"""
    data = np.column_stack([
        result["time"], result["q"], result["qd"], result["qdd"]
    ])
    dof = result["q"].shape[1]
    header = ",".join(
        ["time_s"]
        + [f"J{i}_rad" for i in range(1, dof + 1)]
        + [f"J{i}_vel" for i in range(1, dof + 1)]
        + [f"J{i}_acc" for i in range(1, dof + 1)]
    )
    np.savetxt(
        output_file,
        data,
        delimiter=",",
        header=header,
        comments="",
        fmt="%.10f",
    )


def export_keypoints(path, output_file):
    """以原格式导出六轴弧度制路径关键点。"""
    dof = np.asarray(path).shape[1]
    np.savetxt(
        output_file,
        path,
        delimiter=",",
        header=",".join(f"J{i}" for i in range(1, dof + 1)),
        comments="",
    )


def export_metrics_comparison(rows, output_file):
    """导出领导展示使用的路径、时间及动态负荷指标。"""
    fields = [
        "name",
        "tcp_m",
        "joint_rad",
        "joint_motion_deg",
        "ruckig_time_s",
        "accel_rms_deg_s2",
        "accel_ias_rad2_s3",
    ]
    with open(output_file, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
