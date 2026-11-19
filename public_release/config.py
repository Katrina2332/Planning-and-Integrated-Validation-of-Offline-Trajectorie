# -*- coding: utf-8 -*-
"""统一加载、校验 Demo 配置，并提供旧版大写常量兼容层。"""

from copy import deepcopy
from pathlib import Path

import numpy as np
import yaml

from resource_dictionary import (
    ResourceDictionaryError,
    get_robot_info,
    get_scene_info,
    read_catalog,
    read_yaml_unique,
)


PROJECT_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG_PATH = PROJECT_DIR / "config.yaml"


class ConfigError(ValueError):
    """配置文件缺少字段或字段值不合法。"""

    def __init__(self, message, *, code="INVALID_CONFIG", path=None, field=None):
        self.message = message
        self.code = code
        self.path = Path(path) if path is not None else None
        self.field = field
        location = f" [{self.path}]" if self.path is not None else ""
        detail = f" ({field})" if field is not None else ""
        super().__init__(f"{code}{location}{detail}: {message}")


def _require(mapping, key, context, *, code="INVALID_CONFIG", path=None):
    if key not in mapping:
        raise ConfigError(
            f"配置缺少字段：{context}.{key}", code=code,
            path=path, field=f"{context}.{key}",
        )
    return mapping[key]


def _vector(values, length, context, *, positive=False, code="INVALID_CONFIG", path=None):
    try:
        result = np.asarray(values, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ConfigError(
            f"{context} 必须是数值数组。", code=code, path=path, field=context
        ) from exc
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ConfigError(
            f"{context} 必须是长度为 {length} 的有限数值数组。",
            code=code, path=path, field=context,
        )
    if positive and np.any(result <= 0.0):
        raise ConfigError(
            f"{context} 中的值必须全部大于零。",
            code=code, path=path, field=context,
        )
    return result


def _positive(value, context, *, allow_zero=False, code="INVALID_CONFIG", path=None):
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(
            f"{context} 必须是数值。", code=code, path=path, field=context
        ) from exc
    invalid = result < 0.0 if allow_zero else result <= 0.0
    if not np.isfinite(result) or invalid:
        condition = "大于等于零" if allow_zero else "大于零"
        raise ConfigError(
            f"{context} 必须是{condition}的有限数值。",
            code=code, path=path, field=context,
        )
    return result


def _resolve_path(value, config_dir):
    path = Path(value).expanduser()
    return path if path.is_absolute() else (config_dir / path).resolve()


def _resource_path(config, kind, config_path):
    """返回错误字段所在的原始配置文件。"""
    if config.get("ids", {}).get("robot_id") == "legacy_inline":
        return config_path
    return config_path.parent / "catalogs" / f"{kind}.yaml"


def _with_resource_context(error, raw, config_path):
    """将资源 ID 补到可定位的配置错误中，保留原有错误码与字段。"""
    message = error.message
    ids = raw.get("ids", {})
    if isinstance(ids, dict):
        for kind, key in (
            ("robots", "robot_id"), ("scenes", "scene_id"),
            ("tasks", "task_id"),
        ):
            if error.path == config_path.parent / "catalogs" / f"{kind}.yaml":
                resource_id = ids.get(key)
                if isinstance(resource_id, str) and resource_id not in message:
                    message = f"{kind}.{resource_id}: {message}"
                break
    return ConfigError(message, code=error.code, path=error.path, field=error.field)


def _merge_motion_limits(config, robot, config_path):
    """机器人限制为上界；运行配置可设更严格的逐轴限制。"""
    robot_path = _resource_path(config, "robots", config_path)
    limits = _require(
        robot, "motion_limits", "robot", code="MISSING_MOTION_LIMITS",
        path=robot_path,
    )
    if not isinstance(limits, dict):
        raise ConfigError(
            "robot.motion_limits 必须是映射。", code="MISSING_MOTION_LIMITS",
            path=robot_path, field="robot.motion_limits",
        )
    robot.pop("motion_limits")
    planner = _require(config, "planner", "config", path=config_path)
    ruckig = _require(planner, "ruckig", "planner", path=config_path)
    if not isinstance(ruckig, dict):
        raise ConfigError("planner.ruckig 必须是映射。", path=config_path, field="planner.ruckig")
    dof = int(_require(robot, "expected_dof", "robot", path=robot_path))
    for key in (
        "max_velocity_deg_s", "max_acceleration_deg_s2", "max_jerk_deg_s3"
    ):
        field = f"robot.motion_limits.{key}"
        robot_limit = _vector(
            _require(limits, key, "robot.motion_limits", code="MISSING_MOTION_LIMITS", path=robot_path),
            dof, field, positive=True, code="INVALID_ROBOT_CONFIG", path=robot_path,
        )
        requested = _vector(
            ruckig[key], dof, f"planner.ruckig.{key}", positive=True,
            path=config_path,
        ) if key in ruckig else robot_limit.copy()
        if np.any(requested > robot_limit + 1e-12):
            raise ConfigError(
                f"planner.ruckig.{key} 超过机器人字典中的逐轴上限。",
                code="MOTION_LIMIT_EXCEEDED", path=config_path,
                field=f"planner.ruckig.{key}",
            )
        ruckig[key] = requested


def _attach_task_bodies(config, robot, task, config_path):
    task_path = _resource_path(config, "tasks", config_path)
    if "attached_bodies" in task:
        robot["attached_bodies"] = deepcopy(task.pop("attached_bodies"))
    elif "attached_bodies" not in robot:
        raise ConfigError(
            "任务缺少 attached_bodies。", code="INVALID_TASK_CONFIG",
            path=task_path, field="task.attached_bodies",
        )
    task.pop("robot_id", None)
    task.pop("scene_id", None)
    task.pop("compatible_scene_ids", None)


def _compose_catalog_config(raw, config_path):
    """Resolve IDs into the runtime shape consumed by existing modules."""
    config = deepcopy(raw)
    ids = _require(config, "ids", "config", path=config_path)
    if not isinstance(ids, dict):
        raise ConfigError("config.ids 必须是映射。")
    for key in ("robot_id", "scene_id", "task_id"):
        value = _require(ids, key, "ids", path=config_path)
        if not isinstance(value, str) or not value.strip():
            raise ConfigError(f"ids.{key} 必须是非空字符串。", path=config_path, field=f"ids.{key}")
    for section in ("robot", "scene", "task"):
        if section in config:
            raise ConfigError(
                f"config.yaml 不应内嵌 {section}；请在 catalogs 中按 ID 定义。",
                path=config_path, field=section,
            )

    config_dir = config_path.parent
    robot = get_robot_info(ids["robot_id"], config_dir)
    scene = get_scene_info(ids["scene_id"], config_dir)
    tasks = read_catalog("tasks", config_dir)
    if ids["task_id"] not in tasks:
        raise ConfigError(
            f"tasks.yaml 中找不到 ID {ids['task_id']!r}。",
            code="TASK_NOT_FOUND", path=config_dir / "catalogs" / "tasks.yaml",
            field=ids["task_id"],
        )
    task = tasks[ids["task_id"]]
    task_path = config_dir / "catalogs" / "tasks.yaml"
    if _require(task, "robot_id", f"tasks.{ids['task_id']}", path=task_path) != ids["robot_id"]:
        raise ConfigError(
            f"任务 {ids['task_id']!r} 不适用于机器人 {ids['robot_id']!r}。",
            code="INCOMPATIBLE_TASK_RESOURCE", path=task_path, field="robot_id",
        )
    has_single = "scene_id" in task
    has_multiple = "compatible_scene_ids" in task
    if has_single == has_multiple:
        raise ConfigError(
            "任务必须且只能定义 scene_id 或 compatible_scene_ids。",
            code="INVALID_TASK_CONFIG", path=task_path,
            field="compatible_scene_ids",
        )
    compatible = [task["scene_id"]] if has_single else task["compatible_scene_ids"]
    if not isinstance(compatible, list) or not compatible or any(
        not isinstance(scene_id, str) or not scene_id.strip() for scene_id in compatible
    ):
        raise ConfigError(
            "任务兼容场景必须是非空的场景 ID 列表。",
            code="INVALID_TASK_CONFIG", path=task_path,
            field="compatible_scene_ids",
        )
    if ids["scene_id"] not in compatible:
        raise ConfigError(
            f"任务 {ids['task_id']!r} 不适用于场景 {ids['scene_id']!r}。",
            code="INCOMPATIBLE_TASK_RESOURCE", path=task_path, field="scene_id",
        )
    _attach_task_bodies(config, robot, task, config_path)
    _merge_motion_limits(config, robot, config_path)

    config["robot"] = robot
    config["scene"] = scene
    config["task"] = task
    return config


def _compose_inline_config(raw, config_path):
    """把旧式内嵌配置送入同一套标准化流程。"""
    config = deepcopy(raw)
    for section in ("robot", "scene", "task"):
        _require(config, section, "config", path=config_path)
    config["ids"] = {
        "robot_id": "legacy_inline",
        "scene_id": "legacy_inline",
        "task_id": "legacy_inline",
    }
    _attach_task_bodies(config, config["robot"], config["task"], config_path)
    _merge_motion_limits(config, config["robot"], config_path)
    return config


def _validate_and_normalize(raw, config_path):
    config = deepcopy(raw)
    for section in ("ids", "robot", "scene", "task", "planner", "output"):
        _require(config, section, "config")

    robot_path = _resource_path(config, "robots", config_path)
    scene_path = _resource_path(config, "scenes", config_path)

    ids = config["ids"]
    for key in ("robot_id", "scene_id", "task_id"):
        if not str(_require(ids, key, "ids")).strip():
            raise ConfigError(f"ids.{key} 不能为空。")

    robot = config["robot"]
    for key in ("display_name", "tcp_link"):
        value = _require(robot, key, "robot")
        if not isinstance(value, str) or not value.strip():
            raise ConfigError(f"robot.{key} 必须是非空字符串。")
    dof = int(_require(robot, "expected_dof", "robot"))
    if dof <= 0:
        raise ConfigError("robot.expected_dof 必须大于零。")
    robot["expected_dof"] = dof
    config_dir = config_path.parent
    robot["urdf"] = _resolve_path(
        _require(robot, "urdf", "robot", code="INVALID_ROBOT_CONFIG", path=robot_path),
        config_dir,
    )
    if not robot["urdf"].is_file():
        raise ConfigError(
            f"URDF 文件不存在：{robot['urdf']}", code="URDF_NOT_FOUND",
            path=robot_path, field="robot.urdf",
        )
    robot["robodk_model"] = _resolve_path(
        _require(robot, "robodk_model", "robot"), config_dir
    )
    base_pose = _require(robot, "base_pose", "robot")
    base_pose["translation_m"] = _vector(
        _require(base_pose, "translation_m", "robot.base_pose"),
        3,
        "robot.base_pose.translation_m",
    )
    base_pose["rpy_deg"] = _vector(
        _require(base_pose, "rpy_deg", "robot.base_pose"),
        3,
        "robot.base_pose.rpy_deg",
    )

    limits = _require(
        robot, "fallback_joint_limits_deg", "robot",
        code="INVALID_JOINT_LIMITS", path=robot_path,
    )
    if not isinstance(limits, dict):
        raise ConfigError(
            "robot.fallback_joint_limits_deg 必须是映射。",
            code="INVALID_JOINT_LIMITS", path=robot_path,
            field="robot.fallback_joint_limits_deg",
        )
    limits["lower"] = _vector(
        _require(limits, "lower", "robot.fallback_joint_limits_deg", code="INVALID_JOINT_LIMITS", path=robot_path),
        dof, "robot.fallback_joint_limits_deg.lower",
        code="INVALID_JOINT_LIMITS", path=robot_path,
    )
    limits["upper"] = _vector(
        _require(limits, "upper", "robot.fallback_joint_limits_deg", code="INVALID_JOINT_LIMITS", path=robot_path),
        dof, "robot.fallback_joint_limits_deg.upper",
        code="INVALID_JOINT_LIMITS", path=robot_path,
    )
    if np.any(limits["upper"] <= limits["lower"]):
        raise ConfigError(
            "robot.fallback_joint_limits_deg 的上限必须大于下限。",
            code="INVALID_JOINT_LIMITS", path=robot_path,
            field="robot.fallback_joint_limits_deg",
        )

    task_path = _resource_path(config, "tasks", config_path)
    attached = _require(
        robot, "attached_bodies", "robot", code="INVALID_TASK_CONFIG",
        path=task_path,
    )
    if not isinstance(attached, list):
        raise ConfigError(
            "robot.attached_bodies 必须是列表。", code="INVALID_TASK_CONFIG",
            path=task_path, field="robot.attached_bodies",
        )
    for index, body in enumerate(attached):
        if not isinstance(body, dict):
            raise ConfigError(
                f"robot.attached_bodies[{index}] 必须是映射。",
                code="INVALID_TASK_CONFIG", path=task_path,
                field=f"robot.attached_bodies[{index}]",
            )
    attached_names = [str(body.get("name", "")) for body in attached]
    if len(attached_names) != len(set(attached_names)):
        raise ConfigError(
            "robot.attached_bodies 的 name 不能重复。",
            code="INVALID_TASK_CONFIG", path=task_path,
            field="robot.attached_bodies",
        )
    if not {"gripper", "workpiece"}.issubset(attached_names):
        raise ConfigError(
            "当前 Demo 要求 attached_bodies 包含 gripper 和 workpiece。",
            code="INVALID_TASK_CONFIG", path=task_path,
            field="robot.attached_bodies",
        )
    for index, body in enumerate(attached):
        prefix = f"robot.attached_bodies[{index}]"
        _require(body, "name", prefix, code="INVALID_TASK_CONFIG", path=task_path)
        body["size_m"] = _vector(
            _require(body, "size_m", prefix, code="INVALID_TASK_CONFIG", path=task_path),
            3, f"{prefix}.size_m", positive=True,
            code="INVALID_TASK_CONFIG", path=task_path,
        )
        body["tcp_offset_m"] = _vector(
            _require(body, "tcp_offset_m", prefix, code="INVALID_TASK_CONFIG", path=task_path),
            3, f"{prefix}.tcp_offset_m",
            code="INVALID_TASK_CONFIG", path=task_path,
        )
        body["tcp_rpy_deg"] = _vector(
            _require(body, "tcp_rpy_deg", prefix, code="INVALID_TASK_CONFIG", path=task_path),
            3, f"{prefix}.tcp_rpy_deg",
            code="INVALID_TASK_CONFIG", path=task_path,
        )
        collision_enabled = body.get("collision_enabled", True)
        if not isinstance(collision_enabled, bool):
            raise ConfigError(
                f"{prefix}.collision_enabled 必须是 YAML 布尔值。",
                code="INVALID_TASK_CONFIG", path=task_path,
                field=f"{prefix}.collision_enabled",
            )
        body["collision_enabled"] = collision_enabled

    scene = config["scene"]
    if not isinstance(scene, dict):
        raise ConfigError(
            "scene 必须是映射。", code="INVALID_SCENE_CONFIG",
            path=scene_path, field="scene",
        )
    scene["collision_safety_margin_m"] = _positive(
        _require(
            scene, "collision_safety_margin_m", "scene",
            code="INVALID_SCENE_CONFIG", path=scene_path,
        ),
        "scene.collision_safety_margin_m",
        allow_zero=True,
        code="INVALID_SCENE_CONFIG", path=scene_path,
    )
    objects = _require(
        scene, "objects", "scene", code="INVALID_SCENE_CONFIG",
        path=scene_path,
    )
    if not isinstance(objects, list) or not objects:
        raise ConfigError(
            "scene.objects 必须是非空列表。",
            code="INVALID_SCENE_CONFIG", path=scene_path,
            field="scene.objects",
        )
    for index, item in enumerate(objects):
        if not isinstance(item, dict):
            raise ConfigError(
                f"scene.objects[{index}] 必须是映射。",
                code="INVALID_OBSTACLE_GEOMETRY", path=scene_path,
                field=f"scene.objects[{index}]",
            )
    object_names = [str(item.get("name", "")) for item in objects]
    if len(object_names) != len(set(object_names)):
        raise ConfigError(
            "scene.objects 的 name 不能重复。",
            code="INVALID_SCENE_CONFIG", path=scene_path,
            field="scene.objects",
        )
    required_objects = {
        "left_pick_platform",
        "right_place_platform",
        "central_safety_block",
    }
    if not required_objects.issubset(object_names):
        raise ConfigError(
            "当前 Demo 要求 scene.objects 包含左右平台和 central_safety_block。",
            code="INVALID_SCENE_CONFIG", path=scene_path,
            field="scene.objects",
        )
    for index, item in enumerate(objects):
        prefix = f"scene.objects[{index}]"
        _require(item, "name", prefix, code="INVALID_SCENE_CONFIG", path=scene_path)
        if item.get("type", "box") != "box":
            raise ConfigError(
                f"{prefix}.type 只支持 box。",
                code="INVALID_OBSTACLE_GEOMETRY", path=scene_path,
                field=f"{prefix}.type",
            )
        item["size_m"] = _vector(
            _require(item, "size_m", prefix, code="INVALID_OBSTACLE_GEOMETRY", path=scene_path),
            3, f"{prefix}.size_m", positive=True,
            code="INVALID_OBSTACLE_GEOMETRY", path=scene_path,
        )
        item["center_m"] = _vector(
            _require(item, "center_m", prefix, code="INVALID_OBSTACLE_GEOMETRY", path=scene_path),
            3, f"{prefix}.center_m",
            code="INVALID_OBSTACLE_GEOMETRY", path=scene_path,
        )
        item["rpy_deg"] = _vector(
            item.get("rpy_deg", [0, 0, 0]), 3, f"{prefix}.rpy_deg",
            code="INVALID_OBSTACLE_GEOMETRY", path=scene_path,
        )
        collision_enabled = item.get("collision_enabled", True)
        if not isinstance(collision_enabled, bool):
            raise ConfigError(
                f"{prefix}.collision_enabled 必须是 YAML 布尔值。",
                code="INVALID_OBSTACLE_GEOMETRY", path=scene_path,
                field=f"{prefix}.collision_enabled",
            )
        item["collision_enabled"] = collision_enabled

    task = config["task"]
    task.setdefault("manual_candidates_deg", [])
    task["start_joint_deg"] = _vector(
        task["start_joint_deg"], dof, "task.start_joint_deg"
    )
    task["goal_joint_deg"] = _vector(
        task["goal_joint_deg"], dof, "task.goal_joint_deg"
    )
    if "pick_place_demo" in task:
        demo = task["pick_place_demo"]
        task_path = _resource_path(config, "tasks", config_path)
        if not isinstance(demo, dict):
            raise ConfigError(
                "task.pick_place_demo 必须是映射。", code="INVALID_TASK_CONFIG",
                path=task_path, field="task.pick_place_demo",
            )
        for key in ("pre_pick_joint_deg", "post_place_joint_deg"):
            field = f"task.pick_place_demo.{key}"
            demo[key] = _vector(
                _require(demo, key, "task.pick_place_demo",
                         code="INVALID_TASK_CONFIG", path=task_path),
                dof, field, code="INVALID_TASK_CONFIG", path=task_path,
            )
    for candidate_index, candidate in enumerate(
        task.get("manual_candidates_deg", [])
    ):
        for point_index, point in enumerate(candidate):
            candidate[point_index] = _vector(
                point,
                dof,
                f"task.manual_candidates_deg[{candidate_index}][{point_index}]",
            )

    planner = config["planner"]
    planner["random_seed"] = int(planner["random_seed"])
    rrt = _require(planner, "rrt", "planner")
    rrt["step_size_deg"] = _positive(
        rrt["step_size_deg"], "planner.rrt.step_size_deg"
    )
    rrt["max_iterations"] = int(rrt["max_iterations"])
    rrt["connect_max_extensions"] = int(rrt["connect_max_extensions"])
    if rrt["max_iterations"] <= 0 or rrt["connect_max_extensions"] <= 0:
        raise ConfigError("RRT 迭代次数和连接扩展次数必须大于零。")
    rrt["goal_bias"] = float(rrt["goal_bias"])
    if not 0.0 <= rrt["goal_bias"] <= 1.0:
        raise ConfigError("planner.rrt.goal_bias 必须位于 [0, 1]。")

    shortcut = _require(planner, "shortcut", "planner")
    shortcut["enabled"] = bool(shortcut["enabled"])
    shortcut["attempts"] = int(shortcut["attempts"])
    if shortcut["attempts"] < 0:
        raise ConfigError("planner.shortcut.attempts 不能为负数。")

    validation = _require(planner, "validation", "planner")
    for key in (
        "edge_check_step_deg",
        "max_edge_tcp_rotation_deg",
        "max_ruckig_segment_tcp_rotation_deg",
    ):
        validation[key] = _positive(validation[key], f"planner.validation.{key}")
    validation["orientation_constraint_enabled"] = bool(
        validation["orientation_constraint_enabled"]
    )

    ruckig = _require(planner, "ruckig", "planner")
    for key in (
        "max_velocity_deg_s",
        "max_acceleration_deg_s2",
        "max_jerk_deg_s3",
    ):
        ruckig[key] = _vector(
            ruckig[key], dof, f"planner.ruckig.{key}", positive=True
        )
    ruckig["sample_dt_s"] = _positive(
        ruckig["sample_dt_s"], "planner.ruckig.sample_dt_s"
    )

    output = config["output"]
    output["directory"] = _resolve_path(output["directory"], config_dir)
    output["save_csv"] = bool(output["save_csv"])
    output["save_figures"] = bool(output["save_figures"])
    output["show_visualization"] = bool(output["show_visualization"])
    output["figure_dpi"] = int(output["figure_dpi"])
    if output["figure_dpi"] <= 0:
        raise ConfigError("output.figure_dpi 必须大于零。")
    output_files = _require(output, "files", "output")
    required_output_files = (
        "controller_program",
        "ruckig_trajectory",
        "manual_keypoints",
        "rrt_keypoints",
        "shortcut_keypoints",
        "metrics",
        "path_figure",
        "joint_position_figure",
        "joint_velocity_figure",
        "joint_acceleration_figure",
    )
    for key in required_output_files:
        if not str(_require(output_files, key, "output.files")).strip():
            raise ConfigError(f"output.files.{key} 不能为空。")

    robodk = config.get("robodk", {})
    if robodk:
        for key in ("executable", "station_file", "report_file"):
            robodk[key] = _resolve_path(_require(robodk, key, "robodk"), config_dir)
        robodk["replay_speed"] = _positive(
            robodk["replay_speed"], "robodk.replay_speed"
        )
        for key in ("pick_pause_s", "place_pause_s"):
            robodk[key] = _positive(
                robodk[key], f"robodk.{key}", allow_zero=True
            )
        robodk["movej_check_step_deg"] = _positive(
            robodk["movej_check_step_deg"], "robodk.movej_check_step_deg"
        )
        robodk["program_max_sample_span"] = int(
            robodk["program_max_sample_span"]
        )
        if robodk["program_max_sample_span"] <= 0:
            raise ConfigError("robodk.program_max_sample_span 必须大于零。")

    config["config_path"] = config_path
    return config


def load_config(config_path=None):
    """组合 Dictionary 或旧式内嵌配置并在规划前校验。"""
    path = Path(config_path or DEFAULT_CONFIG_PATH).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"配置文件不存在：{path}")
    try:
        raw = read_yaml_unique(path)
    except ResourceDictionaryError as exc:
        raise _with_resource_context(exc, {}, path) from exc
    except yaml.YAMLError as exc:
        raise ConfigError(
            f"运行配置 YAML 格式错误：{exc}", path=path
        ) from exc
    if not isinstance(raw, dict):
        raise ConfigError("配置文件顶层必须是映射。", path=path)
    try:
        composed = (
            _compose_catalog_config(raw, path)
            if "ids" in raw else _compose_inline_config(raw, path)
        )
        return _validate_and_normalize(composed, path)
    except ResourceDictionaryError as exc:
        raise _with_resource_context(exc, raw, path) from exc
    except ConfigError as exc:
        contextual = _with_resource_context(exc, raw, path)
        if contextual.message == exc.message:
            raise
        raise contextual from exc
    except (KeyError, TypeError, ValueError) as exc:
        raise ConfigError(
            f"配置字段缺失或格式错误：{exc}", path=path
        ) from exc


DEFAULT_CONFIG = load_config()

# 以下常量由 config.yaml 派生，仅用于兼容现有模块和外部脚本。
ROBOT_ID = DEFAULT_CONFIG["ids"]["robot_id"]
SCENE_ID = DEFAULT_CONFIG["ids"]["scene_id"]
TASK_ID = DEFAULT_CONFIG["ids"]["task_id"]

_ROBOT = DEFAULT_CONFIG["robot"]
ROBOT_DISPLAY_NAME = _ROBOT["display_name"]
EXPECTED_DOF = _ROBOT["expected_dof"]
URDF_PATH = _ROBOT["urdf"]
ROBODK_ROBOT_MODEL = _ROBOT["robodk_model"]
TCP_LINK = _ROBOT["tcp_link"]
ROBOT_BASE_TRANSLATION = _ROBOT["base_pose"]["translation_m"]
ROBOT_BASE_RPY_DEG = _ROBOT["base_pose"]["rpy_deg"]
ROBOT_BASE_Z_OFFSET = float(ROBOT_BASE_TRANSLATION[2])
FALLBACK_JOINT_LIMITS_DEG = np.vstack([
    _ROBOT["fallback_joint_limits_deg"]["lower"],
    _ROBOT["fallback_joint_limits_deg"]["upper"],
])
ATTACHED_BODY_CONFIGS = _ROBOT["attached_bodies"]

_ATTACHED_BY_NAME = {body["name"]: body for body in ATTACHED_BODY_CONFIGS}
_GRIPPER = _ATTACHED_BY_NAME["gripper"]
_WORKPIECE = _ATTACHED_BY_NAME["workpiece"]
ENABLE_GRIPPER_COLLISION = _GRIPPER["collision_enabled"]
ENABLE_WORKPIECE_COLLISION = _WORKPIECE["collision_enabled"]
GRIPPER_SIZE = _GRIPPER["size_m"]
GRIPPER_TCP_OFFSET = _GRIPPER["tcp_offset_m"]
GRIPPER_TCP_RPY_DEG = _GRIPPER["tcp_rpy_deg"]
WORKPIECE_SIZE = _WORKPIECE["size_m"]
WORKPIECE_TCP_OFFSET = _WORKPIECE["tcp_offset_m"]
WORKPIECE_TCP_RPY_DEG = _WORKPIECE["tcp_rpy_deg"]

_SCENE = DEFAULT_CONFIG["scene"]
COLLISION_SAFETY_MARGIN = _SCENE["collision_safety_margin_m"]
SCENE_OBJECT_CONFIGS = _SCENE["objects"]
_SCENE_BY_NAME = {item["name"]: item for item in SCENE_OBJECT_CONFIGS}
_LEFT_PLATFORM = _SCENE_BY_NAME["left_pick_platform"]
_RIGHT_PLATFORM = _SCENE_BY_NAME["right_place_platform"]
_OBSTACLE = _SCENE_BY_NAME["central_safety_block"]
LEFT_PLATFORM_SIZE = _LEFT_PLATFORM["size_m"]
LEFT_PLATFORM_CENTER = _LEFT_PLATFORM["center_m"]
RIGHT_PLATFORM_SIZE = _RIGHT_PLATFORM["size_m"]
RIGHT_PLATFORM_CENTER = _RIGHT_PLATFORM["center_m"]
OBSTACLE_NAME = _OBSTACLE["name"]
OBSTACLE_SIZE = _OBSTACLE["size_m"]
OBSTACLE_CENTER = _OBSTACLE["center_m"]
OBSTACLE_RPY_DEG = _OBSTACLE["rpy_deg"]

_TASK = DEFAULT_CONFIG["task"]
Q_START_DEG = _TASK["start_joint_deg"]
Q_GOAL_DEG = _TASK["goal_joint_deg"]
MANUAL_CANDIDATES_DEG = _TASK["manual_candidates_deg"]

_PLANNER = DEFAULT_CONFIG["planner"]
RANDOM_SEED = _PLANNER["random_seed"]
RRT_STEP = np.deg2rad(_PLANNER["rrt"]["step_size_deg"])
MAX_ITERATIONS = _PLANNER["rrt"]["max_iterations"]
GOAL_BIAS = _PLANNER["rrt"]["goal_bias"]
RRT_CONNECT_MAX_EXTENSIONS = _PLANNER["rrt"]["connect_max_extensions"]
SHORTCUT_ENABLED = _PLANNER["shortcut"]["enabled"]
SHORTCUT_ATTEMPTS = _PLANNER["shortcut"]["attempts"]
EDGE_CHECK_STEP = np.deg2rad(
    _PLANNER["validation"]["edge_check_step_deg"]
)
ENABLE_ORIENTATION_CONSTRAINT = _PLANNER["validation"][
    "orientation_constraint_enabled"
]
MAX_EDGE_TCP_ROTATION_DEG = _PLANNER["validation"][
    "max_edge_tcp_rotation_deg"
]
MAX_RUCKIG_SEGMENT_TCP_ROTATION_DEG = _PLANNER["validation"][
    "max_ruckig_segment_tcp_rotation_deg"
]
RUCKIG_MAX_VEL = np.deg2rad(
    _PLANNER["ruckig"]["max_velocity_deg_s"]
)
RUCKIG_MAX_ACC = np.deg2rad(
    _PLANNER["ruckig"]["max_acceleration_deg_s2"]
)
RUCKIG_MAX_JERK = np.deg2rad(
    _PLANNER["ruckig"]["max_jerk_deg_s3"]
)
RUCKIG_SAMPLE_DT = _PLANNER["ruckig"]["sample_dt_s"]

_OUTPUT = DEFAULT_CONFIG["output"]
OUTPUT_DIR = _OUTPUT["directory"]
SAVE_CSV = _OUTPUT["save_csv"]
SAVE_FIGURES = _OUTPUT["save_figures"]
SHOW_VISUALIZATION = _OUTPUT["show_visualization"]
FIGURE_DPI = _OUTPUT["figure_dpi"]
OUTPUT_FILES = _OUTPUT["files"]

_ROBODK = DEFAULT_CONFIG.get("robodk", {})
ROBODK_EXECUTABLE = _ROBODK.get("executable")
ROBODK_STATION_FILE = _ROBODK.get("station_file")
ROBODK_REPORT_FILE = _ROBODK.get("report_file")
ROBODK_REPLAY_SPEED = _ROBODK.get("replay_speed", 1.0)
ROBODK_PICK_PAUSE_S = _ROBODK.get("pick_pause_s", 0.8)
ROBODK_PLACE_PAUSE_S = _ROBODK.get("place_pause_s", 0.8)
ROBODK_MOVEJ_CHECK_STEP_DEG = _ROBODK.get("movej_check_step_deg", 0.5)
ROBODK_PROGRAM_MAX_SAMPLE_SPAN = _ROBODK.get(
    "program_max_sample_span", 10
)
