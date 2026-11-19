"""已有几何资源在不同场景 ID 下的切换行为。"""

from pathlib import Path

import yaml

from config import load_config
from resource_dictionary import get_scene_info, list_scenes


PROJECT_DIR = Path(__file__).resolve().parents[1]


def _write_yaml(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, allow_unicode=True), encoding="utf-8")


def _fixture(tmp_path):
    raw = yaml.safe_load((PROJECT_DIR / "config.yaml").read_text(encoding="utf-8"))
    for filename in ("robots.yaml", "scenes.yaml", "tasks.yaml"):
        source = PROJECT_DIR / "catalogs" / filename
        target = tmp_path / "catalogs" / filename
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(source.read_text(encoding="utf-8"), encoding="utf-8")
    urdf_path = tmp_path / yaml.safe_load(
        (tmp_path / "catalogs" / "robots.yaml").read_text(encoding="utf-8")
    )["ar2010"]["urdf"]
    urdf_path.parent.mkdir(parents=True, exist_ok=True)
    urdf_path.write_text(
        '<robot name="test"></robot>', encoding="utf-8"
    )
    config_path = tmp_path / "config.yaml"
    _write_yaml(config_path, raw)
    return config_path, raw


def test_default_catalog_lists_three_scenes():
    assert list_scenes(PROJECT_DIR) == [
        "ar2010_transport_demo", "empty_scene", "obstacle_scene"
    ]


def test_empty_scene_has_no_enabled_collision_objects():
    base = get_scene_info("ar2010_transport_demo", PROJECT_DIR)
    empty = get_scene_info("empty_scene", PROJECT_DIR)
    assert [item["name"] for item in empty["objects"]] == [
        item["name"] for item in base["objects"]
    ]
    for original, item in zip(base["objects"], empty["objects"]):
        assert item["size_m"] == original["size_m"]
        assert item["center_m"] == original["center_m"]
        assert item["rpy_deg"] == original["rpy_deg"]
        assert item["collision_enabled"] is False


def test_obstacle_scene_only_enables_existing_central_block():
    base = get_scene_info("ar2010_transport_demo", PROJECT_DIR)
    scene = get_scene_info("obstacle_scene", PROJECT_DIR)
    assert [item["name"] for item in scene["objects"] if item["collision_enabled"]] == [
        "central_safety_block"
    ]
    for original, item in zip(base["objects"], scene["objects"]):
        assert item["size_m"] == original["size_m"]
        assert item["center_m"] == original["center_m"]
        assert item["rpy_deg"] == original["rpy_deg"]


def test_switching_scenes_does_not_reuse_previous_scene_values(tmp_path):
    config_path, raw = _fixture(tmp_path)
    raw["ids"]["scene_id"] = "empty_scene"
    _write_yaml(config_path, raw)
    first = load_config(config_path)
    first["scene"]["objects"][2]["size_m"][0] = 999
    raw["ids"]["scene_id"] = "obstacle_scene"
    _write_yaml(config_path, raw)
    second = load_config(config_path)
    assert not any(item["collision_enabled"] for item in first["scene"]["objects"])
    assert second["scene"]["objects"][2]["size_m"][0] == 0.35
    assert sum(item["collision_enabled"] for item in second["scene"]["objects"]) == 1


def test_transport_task_accepts_all_declared_scenes(tmp_path):
    config_path, raw = _fixture(tmp_path)
    for scene_id in list_scenes(tmp_path):
        raw["ids"]["scene_id"] = scene_id
        _write_yaml(config_path, raw)
        config = load_config(config_path)
        assert config["ids"]["scene_id"] == scene_id
