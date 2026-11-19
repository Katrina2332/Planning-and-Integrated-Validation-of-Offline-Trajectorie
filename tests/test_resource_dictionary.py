"""Robot / Scene Dictionary 的外部可观察行为。"""

from pathlib import Path
from xml.etree import ElementTree

import pytest

from resource_dictionary import (
    ResourceDictionaryError,
    get_robot_info,
    get_scene_info,
    list_robots,
    list_scenes,
    read_catalog,
)


PROJECT_DIR = Path(__file__).resolve().parents[1]


def _catalogs(tmp_path):
    catalogs = tmp_path / "catalogs"
    catalogs.mkdir()
    (catalogs / "robots.yaml").write_text(
        "zeta:\n  urdf: models/zeta.urdf\n  robodk_model: models/zeta.robot\n"
        "alpha:\n  urdf: models/alpha.urdf\n  robodk_model: models/alpha.robot\n",
        encoding="utf-8",
    )
    (catalogs / "scenes.yaml").write_text(
        "work:\n  objects:\n    - name: block\n      size_m: [1, 2, 3]\n"
        "empty:\n  objects: []\n",
        encoding="utf-8",
    )
    (catalogs / "tasks.yaml").write_text(
        "move:\n  robot_id: alpha\n  scene_id: work\n", encoding="utf-8"
    )
    return catalogs


def test_list_resources_returns_sorted_ids(tmp_path):
    _catalogs(tmp_path)
    assert list_robots(tmp_path) == ["alpha", "zeta"]
    assert list_scenes(tmp_path) == ["empty", "work"]


def test_get_resource_returns_deep_copy(tmp_path):
    _catalogs(tmp_path)
    first = get_scene_info("work", tmp_path)
    first["objects"][0]["size_m"][0] = 999
    assert get_scene_info("work", tmp_path)["objects"][0]["size_m"] == [1, 2, 3]


def test_relative_robot_paths_resolve_from_config_directory(tmp_path):
    _catalogs(tmp_path)
    robot = get_robot_info("alpha", tmp_path)
    assert robot["urdf"] == tmp_path / "models" / "alpha.urdf"
    assert robot["robodk_model"] == tmp_path / "models" / "alpha.robot"


@pytest.mark.parametrize(
    "kind,load,resource_id,code,filename",
    [
        ("robot", get_robot_info, "missing_robot", "ROBOT_NOT_FOUND", "robots.yaml"),
        ("scene", get_scene_info, "missing_scene", "SCENE_NOT_FOUND", "scenes.yaml"),
    ],
)
def test_missing_resource_has_specific_code(
    tmp_path, kind, load, resource_id, code, filename
):
    _catalogs(tmp_path)
    with pytest.raises(ResourceDictionaryError) as caught:
        load(resource_id, tmp_path)
    assert caught.value.code == code
    assert caught.value.path == tmp_path / "catalogs" / filename
    assert resource_id in str(caught.value)


def test_duplicate_resource_id_is_rejected(tmp_path):
    catalogs = _catalogs(tmp_path)
    (catalogs / "robots.yaml").write_text(
        "alpha:\n  urdf: first.urdf\nalpha:\n  urdf: second.urdf\n",
        encoding="utf-8",
    )
    with pytest.raises(ResourceDictionaryError) as caught:
        list_robots(tmp_path)
    assert caught.value.code == "DUPLICATE_RESOURCE_ID"
    assert caught.value.path == catalogs / "robots.yaml"
    assert "alpha" in str(caught.value)


def test_duplicate_nested_key_is_rejected(tmp_path):
    catalogs = _catalogs(tmp_path)
    (catalogs / "scenes.yaml").write_text(
        "work:\n  objects: []\n  objects: [block]\n", encoding="utf-8"
    )
    with pytest.raises(ResourceDictionaryError) as caught:
        get_scene_info("work", tmp_path)
    assert caught.value.code == "DUPLICATE_RESOURCE_ID"
    assert caught.value.field == "objects"


def test_malformed_catalog_reports_file_path(tmp_path):
    catalogs = _catalogs(tmp_path)
    (catalogs / "scenes.yaml").write_text("work: [unterminated", encoding="utf-8")
    with pytest.raises(ResourceDictionaryError) as caught:
        list_scenes(tmp_path)
    assert caught.value.code == "INVALID_SCENE_CONFIG"
    assert caught.value.path == catalogs / "scenes.yaml"


def test_read_catalog_supports_existing_task_composition(tmp_path):
    _catalogs(tmp_path)
    first = read_catalog("tasks", tmp_path)
    first["move"]["scene_id"] = "other"
    assert read_catalog("tasks", tmp_path)["move"]["scene_id"] == "work"


def test_invalid_robot_model_path_has_dictionary_error(tmp_path):
    catalogs = _catalogs(tmp_path)
    (catalogs / "robots.yaml").write_text(
        "alpha:\n  urdf: [not, a, path]\n  robodk_model: model.robot\n",
        encoding="utf-8",
    )
    with pytest.raises(ResourceDictionaryError) as caught:
        get_robot_info("alpha", tmp_path)
    assert caught.value.code == "INVALID_ROBOT_CONFIG"
    assert caught.value.path == catalogs / "robots.yaml"
    assert caught.value.field == "robots.alpha.urdf"


def test_step_scr5_catalog_exposes_confirmed_specs_without_invented_dynamics():
    robot = get_robot_info("step_scr5", PROJECT_DIR)

    assert robot["display_name"] == "STEP SCR5"
    assert robot["expected_dof"] == 6
    assert robot["urdf"] == PROJECT_DIR / "step_scr5_planning.urdf"
    assert robot["model_status"] == "provisional_planning_model"
    assert robot["confirmed_specs"] == {
        "payload_kg": 5,
        "reach_m": 0.9,
        "repeatability_mm": 0.02,
        "body_mass_kg": 23,
        "protection_rating": "IP54",
        "max_tool_speed_m_s": 3,
    }
    assert robot["motion_limits"] == {
        "max_velocity_deg_s": [180, 180, 180, 180, 180, 180]
    }
    assert "robodk_model" not in robot


def test_step_scr5_provisional_urdf_uses_agreed_six_axis_skeleton():
    robot = get_robot_info("step_scr5", PROJECT_DIR)
    root = ElementTree.parse(robot["urdf"]).getroot()
    revolute = [
        joint for joint in root.findall("joint")
        if joint.attrib.get("type") == "revolute"
    ]

    assert [joint.attrib["name"] for joint in revolute] == [
        "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
    ]
    assert [joint.find("axis").attrib["xyz"] for joint in revolute] == [
        "0 0 1", "0 1 0", "0 1 0", "0 0 1", "0 1 0", "0 0 1"
    ]
    assert [joint.find("origin").attrib["xyz"] for joint in revolute] == [
        "0 0 0.1615",
        "0 0 0",
        "0 0 0.420",
        "0 0 0.368",
        "0 0 0.112",
        "0 0 0",
    ]
    tool_joint = root.find("joint[@name='wrist_3-tool0']")
    assert tool_joint is not None
    assert tool_joint.find("origin").attrib["xyz"] == "-0.095 0 0"
