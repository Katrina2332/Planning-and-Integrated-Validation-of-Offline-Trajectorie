"""Dictionary 组合阶段的资源安全边界和旧配置兼容。"""

from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from config import ConfigError, load_config


PROJECT_DIR = Path(__file__).resolve().parents[1]


def _write_yaml(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, allow_unicode=True), encoding="utf-8")


def _fixture(tmp_path):
    raw = yaml.safe_load((PROJECT_DIR / "config.yaml").read_text(encoding="utf-8"))
    resources = {}
    for kind, entry_id in (
        ("robots", "ar2010"),
        ("scenes", "ar2010_transport_demo"),
        ("tasks", "left_to_right_transport"),
    ):
        resources[kind] = yaml.safe_load(
            (PROJECT_DIR / "catalogs" / f"{kind}.yaml").read_text(encoding="utf-8")
        )[entry_id]
        _write_yaml(tmp_path / "catalogs" / f"{kind}.yaml", {
            entry_id: resources[kind]
        })
    urdf_path = tmp_path / resources["robots"]["urdf"]
    urdf_path.parent.mkdir(parents=True, exist_ok=True)
    urdf_path.write_text(
        '<robot name="test"></robot>', encoding="utf-8"
    )
    config_path = tmp_path / "config.yaml"
    _write_yaml(config_path, raw)
    return config_path, raw, resources


def _replace_catalog(tmp_path, kind, entry_id, entry):
    _write_yaml(tmp_path / "catalogs" / f"{kind}.yaml", {entry_id: entry})


def _expect_code(config_path, code, field=None):
    with pytest.raises(ConfigError) as caught:
        load_config(config_path)
    assert caught.value.code == code
    assert code in str(caught.value)
    if field:
        assert field in str(caught.value)
    return caught.value


def test_catalog_error_is_exposed_as_config_error_with_code(tmp_path):
    config_path, raw, _ = _fixture(tmp_path)
    raw["ids"]["robot_id"] = "absent"
    _write_yaml(config_path, raw)
    error = _expect_code(config_path, "ROBOT_NOT_FOUND", "absent")
    assert error.path == tmp_path / "catalogs" / "robots.yaml"


def test_duplicate_key_in_run_config_is_rejected(tmp_path):
    config_path, _, _ = _fixture(tmp_path)
    text = config_path.read_text(encoding="utf-8")
    config_path.write_text(text + "\nids:\n  robot_id: absent\n", encoding="utf-8")
    error = _expect_code(config_path, "DUPLICATE_RESOURCE_ID", "ids")
    assert error.path == config_path


def test_missing_urdf_fails_before_planning(tmp_path):
    config_path, _, resources = _fixture(tmp_path)
    resources["robots"]["urdf"] = "missing.urdf"
    _replace_catalog(tmp_path, "robots", "ar2010", resources["robots"])
    _expect_code(config_path, "URDF_NOT_FOUND", "robot.urdf")


def test_reversed_fallback_joint_limits_have_specific_code(tmp_path):
    config_path, _, resources = _fixture(tmp_path)
    resources["robots"]["fallback_joint_limits_deg"]["upper"][0] = -999
    _replace_catalog(tmp_path, "robots", "ar2010", resources["robots"])
    _expect_code(config_path, "INVALID_JOINT_LIMITS", "fallback_joint_limits_deg")


def test_missing_motion_limits_have_specific_code(tmp_path):
    config_path, _, resources = _fixture(tmp_path)
    del resources["robots"]["motion_limits"]["max_jerk_deg_s3"]
    _replace_catalog(tmp_path, "robots", "ar2010", resources["robots"])
    _expect_code(config_path, "MISSING_MOTION_LIMITS", "max_jerk_deg_s3")


@pytest.mark.parametrize("size", [[0.0, 0.5, 0.2], [-1.0, 0.5, 0.2]])
def test_invalid_obstacle_size_has_specific_code(tmp_path, size):
    config_path, _, resources = _fixture(tmp_path)
    resources["scenes"]["objects"][2]["size_m"] = size
    _replace_catalog(tmp_path, "scenes", "ar2010_transport_demo", resources["scenes"])
    error = _expect_code(config_path, "INVALID_OBSTACLE_GEOMETRY", "size_m")
    assert "ar2010_transport_demo" in str(error)


def test_unknown_obstacle_type_is_rejected(tmp_path):
    config_path, _, resources = _fixture(tmp_path)
    resources["scenes"]["objects"][2]["type"] = "sphere"
    _replace_catalog(tmp_path, "scenes", "ar2010_transport_demo", resources["scenes"])
    _expect_code(config_path, "INVALID_OBSTACLE_GEOMETRY", "type")


def test_nonmapping_scene_object_has_locatable_error(tmp_path):
    config_path, _, resources = _fixture(tmp_path)
    resources["scenes"]["objects"][0] = None
    _replace_catalog(tmp_path, "scenes", "ar2010_transport_demo", resources["scenes"])
    error = _expect_code(config_path, "INVALID_OBSTACLE_GEOMETRY", "scene.objects[0]")
    assert error.path == tmp_path / "catalogs" / "scenes.yaml"


def test_invalid_scene_margin_identifies_scene(tmp_path):
    config_path, _, resources = _fixture(tmp_path)
    resources["scenes"]["collision_safety_margin_m"] = -0.01
    _replace_catalog(tmp_path, "scenes", "ar2010_transport_demo", resources["scenes"])
    error = _expect_code(
        config_path, "INVALID_SCENE_CONFIG", "collision_safety_margin_m"
    )
    assert error.path == tmp_path / "catalogs" / "scenes.yaml"
    assert "ar2010_transport_demo" in str(error)


def test_nonmapping_attachment_has_locatable_error(tmp_path):
    config_path, _, resources = _fixture(tmp_path)
    resources["tasks"]["attached_bodies"][0] = None
    _replace_catalog(tmp_path, "tasks", "left_to_right_transport", resources["tasks"])
    error = _expect_code(config_path, "INVALID_TASK_CONFIG", "attached_bodies[0]")
    assert error.path == tmp_path / "catalogs" / "tasks.yaml"


@pytest.mark.parametrize("kind", ["scenes", "tasks"])
def test_collision_enabled_must_be_yaml_boolean(tmp_path, kind):
    config_path, _, resources = _fixture(tmp_path)
    if kind == "scenes":
        resources[kind]["objects"][2]["collision_enabled"] = "false"
        code, field = "INVALID_OBSTACLE_GEOMETRY", "scene.objects[2].collision_enabled"
        entry_id = "ar2010_transport_demo"
    else:
        resources[kind]["attached_bodies"][0]["collision_enabled"] = "false"
        code, field = "INVALID_TASK_CONFIG", "robot.attached_bodies[0].collision_enabled"
        entry_id = "left_to_right_transport"
    _replace_catalog(tmp_path, kind, entry_id, resources[kind])
    _expect_code(config_path, code, field)


def test_task_accepts_each_compatible_scene(tmp_path):
    config_path, raw, resources = _fixture(tmp_path)
    scene = resources["scenes"]
    _write_yaml(tmp_path / "catalogs" / "scenes.yaml", {
        "ar2010_transport_demo": scene,
        "alternative_scene": deepcopy(scene),
    })
    task = resources["tasks"]
    task.pop("scene_id", None)
    task["compatible_scene_ids"] = ["ar2010_transport_demo", "alternative_scene"]
    _replace_catalog(tmp_path, "tasks", "left_to_right_transport", task)
    for scene_id in task["compatible_scene_ids"]:
        raw["ids"]["scene_id"] = scene_id
        _write_yaml(config_path, raw)
        assert load_config(config_path)["ids"]["scene_id"] == scene_id


def test_task_rejects_scene_outside_compatible_list(tmp_path):
    config_path, raw, resources = _fixture(tmp_path)
    scene = resources["scenes"]
    _write_yaml(tmp_path / "catalogs" / "scenes.yaml", {
        "ar2010_transport_demo": scene,
        "other_scene": deepcopy(scene),
    })
    task = resources["tasks"]
    task.pop("scene_id", None)
    task["compatible_scene_ids"] = ["ar2010_transport_demo"]
    _replace_catalog(tmp_path, "tasks", "left_to_right_transport", task)
    raw["ids"]["scene_id"] = "other_scene"
    _write_yaml(config_path, raw)
    _expect_code(config_path, "INCOMPATIBLE_TASK_RESOURCE", "scene_id")


def test_task_cannot_define_both_scene_fields(tmp_path):
    config_path, _, resources = _fixture(tmp_path)
    resources["tasks"]["scene_id"] = "ar2010_transport_demo"
    _replace_catalog(tmp_path, "tasks", "left_to_right_transport", resources["tasks"])
    _expect_code(config_path, "INVALID_TASK_CONFIG", "compatible_scene_ids")


def test_equal_task_motion_limits_are_allowed(tmp_path):
    config_path, raw, resources = _fixture(tmp_path)
    raw["planner"]["ruckig"]["max_velocity_deg_s"] = resources["robots"][
        "motion_limits"
    ]["max_velocity_deg_s"]
    _write_yaml(config_path, raw)
    config = load_config(config_path)
    assert config["planner"]["ruckig"]["max_velocity_deg_s"][0] == 90


def test_task_motion_limit_above_robot_limit_is_rejected(tmp_path):
    config_path, raw, resources = _fixture(tmp_path)
    requested = deepcopy(resources["robots"]["motion_limits"]["max_velocity_deg_s"])
    requested[0] += 1
    raw["planner"]["ruckig"]["max_velocity_deg_s"] = requested
    _write_yaml(config_path, raw)
    _expect_code(config_path, "MOTION_LIMIT_EXCEEDED", "max_velocity_deg_s")


def test_legacy_inline_config_is_normalized(tmp_path):
    config_path, raw, resources = _fixture(tmp_path)
    del raw["ids"]
    raw["robot"] = resources["robots"]
    raw["scene"] = resources["scenes"]
    raw["task"] = resources["tasks"]
    _write_yaml(config_path, raw)
    config = load_config(config_path)
    assert config["ids"] == {
        "robot_id": "legacy_inline",
        "scene_id": "legacy_inline",
        "task_id": "legacy_inline",
    }
    assert config["robot"]["urdf"] == tmp_path / resources["robots"]["urdf"]


def test_ids_and_inline_resources_cannot_be_mixed(tmp_path):
    config_path, raw, resources = _fixture(tmp_path)
    raw["robot"] = resources["robots"]
    _write_yaml(config_path, raw)
    _expect_code(config_path, "INVALID_CONFIG", "robot")
