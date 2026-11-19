# -*- coding: utf-8 -*-
"""用 Viser 回放本项目已有的关节轨迹。"""

import time
import webbrowser
from pathlib import Path

import numpy as np
import pinocchio as pin
import viser
from pinocchio.visualize import ViserVisualizer

from kinematic_constraints import sample_path_edges
from robot_scene import fk_tcp, get_attached_body_pose


def show_native_viewer(config, robot, scene_specs, attached_bodies,
                       manual_path, rrt_path, shortcut_path, trajectory, port=8080):
    """显示机器人、场景、各阶段 TCP 路径和最终 Ruckig 回放。"""
    urdf = str(config["robot"]["urdf"])
    mesh_dirs = [str(Path(urdf).parent)]
    model = pin.buildModelFromUrdf(urdf)
    if model.nq != robot.n:
        raise ValueError("Viser 模型与规划模型的关节维度不一致。")
    collision_model = pin.buildGeomFromUrdf(
        model, urdf, pin.GeometryType.COLLISION, package_dirs=mesh_dirs
    )
    visual_model = pin.buildGeomFromUrdf(
        model, urdf, pin.GeometryType.VISUAL, package_dirs=mesh_dirs
    )
    server = viser.ViserServer(host="localhost", port=port)
    viz = ViserVisualizer(model, collision_model, visual_model)
    viz.initViewer(viewer=server, loadModel=True)
    base_inverse = np.linalg.inv(robot.base.A)

    for spec in scene_specs:
        pose = base_inverse @ spec["pose"].A
        quaternion = pin.Quaternion(pose[:3, :3]).coeffs()
        box_options = {
            "position": pose[:3, 3],
            "wxyz": (quaternion[3], quaternion[0], quaternion[1], quaternion[2]),
        }
        server.scene.add_box(
            f"/environment/{spec['name']}",
            dimensions=tuple(spec["size"]),
            color=(180, 55, 55) if spec["collision_enabled"] else (80, 130, 170),
            opacity=0.45, **box_options,
        )
        if spec["collision_enabled"]:
            margin = config["scene"]["collision_safety_margin_m"]
            server.scene.add_box(
                f"/environment/{spec['name']}_margin",
                dimensions=tuple(spec["size"] + 2 * margin),
                color=(220, 40, 40), wireframe=True, **box_options,
            )

    def tcp_points(path):
        return np.asarray([
            (base_inverse @ fk_tcp(robot, q).A)[:3, 3]
            for q in path
        ])

    path_handles = {}
    for name, path, color in (
        ("人工示教", manual_path, (40, 170, 80)),
        ("Raw RRT", rrt_path, (40, 100, 220)),
        ("Shortcut", shortcut_path, (230, 130, 20)),
        ("最终 Ruckig", trajectory["q"], (155, 65, 190)),
    ):
        samples = path if name == "最终 Ruckig" else sample_path_edges(path)[0]
        xyz = tcp_points(samples)
        path_handles[name] = server.scene.add_line_segments(
            f"/paths/{name}", np.stack([xyz[:-1], xyz[1:]], axis=1),
            colors=color, thickness=3.0, thickness_units="screen",
        )

    body_handles = []
    for body in attached_bodies:
        if body["collision_enabled"]:
            handle = server.scene.add_box(
                f"/attachments/{body['name']}",
                dimensions=tuple(body["size"]),
                color=(40, 170, 80), opacity=0.5,
            )
            body_handles.append((body, handle))

    q = np.asarray(trajectory["q"])
    times = np.asarray(trajectory["time"])

    def display(index):
        viz.display(q[index])
        for body, handle in body_handles:
            pose = base_inverse @ get_attached_body_pose(robot, q[index], body).A
            quaternion = pin.Quaternion(pose[:3, :3]).coeffs()
            handle.position = pose[:3, 3]
            handle.wxyz = (
                quaternion[3], quaternion[0], quaternion[1], quaternion[2]
            )

    server.gui.add_markdown(
        "**本项目离线规划回放**\n\n"
        "绿：人工示教；蓝：Raw RRT；橙：Shortcut；紫：最终 Ruckig。"
        "播放仅用于网页展示，不代表控制器执行。"
    )
    selected = server.gui.add_dropdown(
        "显示路径", ["最终 Ruckig", "Shortcut", "Raw RRT", "人工示教", "全部"],
        initial_value="最终 Ruckig",
    )
    slider = server.gui.add_slider(
        "Ruckig 采样点", min=0, max=len(q) - 1, step=1, initial_value=0
    )
    play = server.gui.add_button("播放")
    stop = server.gui.add_button("停止")
    state = {"play": False, "stop": False}

    @selected.on_update
    def _(_event):
        for name, handle in path_handles.items():
            handle.visible = selected.value == "全部" or selected.value == name

    @slider.on_update
    def _(_event):
        display(int(slider.value))

    @play.on_click
    def _(_event):
        state["play"] = True
        state["stop"] = False

    @stop.on_click
    def _(_event):
        state["stop"] = True

    for name, handle in path_handles.items():
        handle.visible = name == selected.value
    display(0)
    url = f"http://localhost:{server.get_port()}"
    print(f"Viser：{url}", flush=True)
    webbrowser.open(url)
    while True:
        if not state["play"]:
            time.sleep(0.05)
            continue
        state["play"] = False
        for index in range(len(q)):
            if state["stop"]:
                break
            slider.value = index
            time.sleep(max(0.001, float(times[index] - times[index - 1])) if index else 0.001)


def show_pick_place_viewer(config, result, port=8080):
    """回放接近、抓取后搬运、释放后撤离的工件状态。"""
    from main_pick_place import build_pick_place_playback, viewer_frame_state

    robot = result["robot"]
    urdf = str(config["robot"]["urdf"])
    mesh_dirs = [str(Path(urdf).parent)]
    model = pin.buildModelFromUrdf(urdf)
    if model.nq != robot.n:
        raise ValueError("Viser 模型与规划模型的关节维度不一致。")
    collision_model = pin.buildGeomFromUrdf(
        model, urdf, pin.GeometryType.COLLISION, package_dirs=mesh_dirs
    )
    visual_model = pin.buildGeomFromUrdf(
        model, urdf, pin.GeometryType.VISUAL, package_dirs=mesh_dirs
    )
    server = viser.ViserServer(host="localhost", port=port)
    viz = ViserVisualizer(model, collision_model, visual_model)
    viz.initViewer(viewer=server, loadModel=True)
    base_inverse = np.linalg.inv(robot.base.A)

    def set_pose(handle, pose):
        local = base_inverse @ pose.A
        quaternion = pin.Quaternion(local[:3, :3]).coeffs()
        handle.position = local[:3, 3]
        handle.wxyz = (
            quaternion[3], quaternion[0], quaternion[1], quaternion[2]
        )

    for spec in result["scene_specs"]:
        local = base_inverse @ spec["pose"].A
        quaternion = pin.Quaternion(local[:3, :3]).coeffs()
        server.scene.add_box(
            f"/environment/{spec['name']}",
            dimensions=tuple(spec["size"]), position=local[:3, 3],
            wxyz=(quaternion[3], quaternion[0], quaternion[1], quaternion[2]),
            color=(180, 55, 55) if spec["collision_enabled"] else (80, 130, 170),
            opacity=0.45,
        )

    for name, color in (
        ("approach", (40, 100, 220)),
        ("transport", (155, 65, 190)),
        ("retreat", (40, 170, 80)),
    ):
        path = result["phases"][name]["trajectory"]["q"]
        xyz = np.asarray([
            (base_inverse @ fk_tcp(robot, q).A)[:3, 3] for q in path
        ])
        server.scene.add_line_segments(
            f"/paths/{name}", np.stack([xyz[:-1], xyz[1:]], axis=1),
            colors=color, thickness=3.0, thickness_units="screen",
        )

    gripper = result["phases"]["approach"]["attached_bodies"][0]
    workpiece = result["phases"]["transport"]["attached_bodies"][1]
    gripper_handle = server.scene.add_box(
        "/attachments/gripper", dimensions=tuple(gripper["size"]),
        color=(40, 170, 80), opacity=0.5,
    )
    world_handle = server.scene.add_box(
        "/workpiece/world", dimensions=tuple(workpiece["size"]),
        color=(230, 145, 35), opacity=0.7,
    )
    attached_handle = server.scene.add_box(
        "/workpiece/attached", dimensions=tuple(workpiece["size"]),
        color=(40, 170, 80), opacity=0.7,
    )

    playback = build_pick_place_playback(result)
    names = {"approach": "接近", "transport": "搬运", "retreat": "撤离"}
    server.gui.add_markdown(
        "**AR2010 假设抓取流程**\n\n"
        "蓝：接近；紫：携带工件搬运；绿：释放后撤离。"
        "橙色工件静止，绿色工件随 TCP 移动。仅供离线演示。"
    )
    status = server.gui.add_markdown("")
    slider = server.gui.add_slider(
        "抓放采样点", min=0, max=len(playback["q"]) - 1,
        step=1, initial_value=0,
    )
    play = server.gui.add_button("播放")
    stop = server.gui.add_button("停止")
    state = {"play": False, "stop": False}

    def display(index):
        frame = viewer_frame_state(result, playback, index)
        viz.display(frame["q"])
        set_pose(gripper_handle, frame["gripper_pose"])
        active = attached_handle if frame["workpiece_attached"] else world_handle
        set_pose(active, frame["workpiece_pose"])
        attached_handle.visible = frame["workpiece_attached"]
        world_handle.visible = not frame["workpiece_attached"]
        event = ""
        if index and frame["phase"] != playback["phase"][index - 1]:
            event = "；已抓取工件" if frame["workpiece_attached"] else "；已释放工件"
        status.content = f"**阶段：{names[frame['phase']]}{event}**"

    @slider.on_update
    def _(_event):
        display(int(slider.value))

    @play.on_click
    def _(_event):
        state["play"] = True
        state["stop"] = False

    @stop.on_click
    def _(_event):
        state["stop"] = True

    display(0)
    url = f"http://localhost:{server.get_port()}"
    print(f"Viser：{url}", flush=True)
    webbrowser.open(url)
    while True:
        if not state["play"]:
            time.sleep(0.05)
            continue
        state["play"] = False
        for index in range(len(playback["q"])):
            if state["stop"]:
                break
            if index:
                time.sleep(max(0.001, float(
                    playback["time"][index] - playback["time"][index - 1]
                )))
            slider.value = index
