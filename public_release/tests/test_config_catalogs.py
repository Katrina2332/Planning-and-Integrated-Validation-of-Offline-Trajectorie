"""Behavioral checks for ID-based catalog configuration."""

from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest
import yaml

from config import ConfigError, load_config


PROJECT_DIR = Path(__file__).resolve().parents[1]


def _write_yaml(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, allow_unicode=True), encoding="utf-8")


def _catalog_fixture(tmp_path):
    """Copy the current demo definitions into an isolated catalog fixture."""
    raw = yaml.safe_load((PROJECT_DIR / "config.yaml").read_text(encoding="utf-8"))
    robot = yaml.safe_load(
        (PROJECT_DIR / "catalogs" / "robots.yaml").read_text(encoding="utf-8")
    )["ar2010"]
    scene = yaml.safe_load(
        (PROJECT_DIR / "catalogs" / "scenes.yaml").read_text(encoding="utf-8")
    )["ar2010_transport_demo"]
    task = yaml.safe_load(
        (PROJECT_DIR / "catalogs" / "tasks.yaml").read_text(encoding="utf-8")
    )["left_to_right_transport"]

    alternative = deepcopy(robot)
    alternative["display_name"] = "Alternative AR2010"
    alternative["base_pose"]["translation_m"] = [0.1, 0.2, -0.4]
    alternative["motion_limits"]["max_velocity_deg_s"][0] = 75
    alternative_scene = deepcopy(scene)
    alternative_scene["objects"][2]["center_m"] = [0.9, 0.0, 0.5]
    alternative_task = deepcopy(task)
    alternative_task["robot_id"] = "alternate"
    alternative_task.pop("scene_id", None)
    alternative_task["compatible_scene_ids"] = ["alternative_scene"]
    alternative_task["start_joint_deg"][0] = -30

    _write_yaml(tmp_path / "catalogs" / "robots.yaml", {
        "ar2010": robot,
        "alternate": alternative,
    })
    _write_yaml(tmp_path / "catalogs" / "scenes.yaml", {
        "ar2010_transport_demo": scene,
        "alternative_scene": alternative_scene,
    })
    _write_yaml(tmp_path / "catalogs" / "tasks.yaml", {
        "left_to_right_transport": task,
        "alternative_task": alternative_task,
    })
    urdf_path = tmp_path / robot["urdf"]
    urdf_path.parent.mkdir(parents=True, exist_ok=True)
    urdf_path.write_text(
        '<robot name="test"></robot>', encoding="utf-8"
    )
    config_path = tmp_path / "config.yaml"
    _write_yaml(config_path, raw)
    return config_path, raw


def test_selected_ids_compose_runtime_config(tmp_path):
    config_path, raw = _catalog_fixture(tmp_path)
    raw["ids"]["robot_id"] = "alternate"
    raw["ids"]["scene_id"] = "alternative_scene"
    raw["ids"]["task_id"] = "alternative_task"
    _write_yaml(config_path, raw)

    config = load_config(config_path)

    assert config["robot"]["display_name"] == "Alternative AR2010"
    assert config["robot"]["base_pose"]["translation_m"].tolist() == [0.1, 0.2, -0.4]
    assert config["robot"]["attached_bodies"][1]["name"] == "workpiece"
    assert config["task"]["start_joint_deg"][0] == -30
    assert config["planner"]["ruckig"]["max_velocity_deg_s"][0] == 75
    assert config["robot"]["urdf"] == tmp_path / "models/ar2010_ros_industrial/ar2010.urdf"
    assert config["scene"]["objects"][2]["center_m"].tolist() == [0.9, 0.0, 0.5]


@pytest.mark.parametrize("kind,bad_id", [
    ("robot", "missing_robot"),
    ("scene", "missing_scene"),
    ("task", "missing_task"),
])
def test_unknown_id_reports_catalog_and_id(tmp_path, kind, bad_id):
    config_path, raw = _catalog_fixture(tmp_path)
    raw["ids"][f"{kind}_id"] = bad_id
    _write_yaml(config_path, raw)

    with pytest.raises(ConfigError, match=bad_id):
        load_config(config_path)


@pytest.mark.parametrize("kind", ["robot", "scene"])
def test_task_cannot_use_incompatible_robot_or_scene(tmp_path, kind):
    config_path, raw = _catalog_fixture(tmp_path)
    if kind == "robot":
        raw["ids"]["robot_id"] = "alternate"
    else:
        scenes_path = tmp_path / "catalogs" / "scenes.yaml"
        scenes = yaml.safe_load(scenes_path.read_text(encoding="utf-8"))
        scenes["other_scene"] = deepcopy(scenes["ar2010_transport_demo"])
        _write_yaml(scenes_path, scenes)
        raw["ids"]["scene_id"] = "other_scene"
    _write_yaml(config_path, raw)

    with pytest.raises(ConfigError, match=f"{kind}_id"):
        load_config(config_path)


def test_repeated_loads_do_not_share_mutable_catalog_values(tmp_path):
    config_path, _ = _catalog_fixture(tmp_path)
    first = load_config(config_path)
    first["robot"]["base_pose"]["translation_m"][0] = 999
    first["scene"]["objects"][0]["size_m"][0] = 999

    second = load_config(config_path)

    assert np.isclose(second["robot"]["base_pose"]["translation_m"][0], 0)
    assert np.isclose(second["scene"]["objects"][0]["size_m"][0], 0.8)


def test_motion_limits_must_match_selected_robot_dof(tmp_path):
    config_path, _ = _catalog_fixture(tmp_path)
    robot_path = tmp_path / "catalogs" / "robots.yaml"
    robots = yaml.safe_load(robot_path.read_text(encoding="utf-8"))
    robots["ar2010"]["motion_limits"]["max_velocity_deg_s"] = [90, 90]
    _write_yaml(robot_path, robots)

    with pytest.raises(ConfigError, match="max_velocity_deg_s"):
        load_config(config_path)


def test_malformed_catalog_yaml_has_config_error(tmp_path):
    config_path, _ = _catalog_fixture(tmp_path)
    robot_path = tmp_path / "catalogs" / "robots.yaml"
    robot_path.write_text("ar2010: [unterminated", encoding="utf-8")

    with pytest.raises(ConfigError, match="robots.yaml"):
        load_config(config_path)


@pytest.mark.parametrize("field", ["display_name", "tcp_link"])
def test_selected_robot_requires_identifying_fields(tmp_path, field):
    config_path, _ = _catalog_fixture(tmp_path)
    robot_path = tmp_path / "catalogs" / "robots.yaml"
    robots = yaml.safe_load(robot_path.read_text(encoding="utf-8"))
    robots["ar2010"].pop(field)
    _write_yaml(robot_path, robots)

    with pytest.raises(ConfigError, match=field):
        load_config(config_path)
