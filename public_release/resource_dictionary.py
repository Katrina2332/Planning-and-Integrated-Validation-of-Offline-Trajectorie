# -*- coding: utf-8 -*-
"""本地 YAML 资源字典的读取、查询与重复 ID 检测。"""

from copy import deepcopy
from pathlib import Path

import yaml


PROJECT_DIR = Path(__file__).resolve().parent
_CATALOG_CODES = {
    "robots": "INVALID_ROBOT_CONFIG",
    "scenes": "INVALID_SCENE_CONFIG",
    "tasks": "INVALID_TASK_CONFIG",
}


class ResourceDictionaryError(ValueError):
    """携带错误码、源文件及字段位置的资源读取错误。"""

    def __init__(self, code, message, *, path=None, field=None):
        self.message = message
        self.code = code
        self.path = Path(path) if path is not None else None
        self.field = field
        location = f" [{self.path}]" if self.path is not None else ""
        detail = f" ({field})" if field is not None else ""
        super().__init__(f"{code}{location}{detail}: {message}")


def _catalog_path(catalog_name, config_dir):
    if catalog_name not in _CATALOG_CODES:
        raise ResourceDictionaryError(
            "INVALID_CATALOG", f"未知资源字典：{catalog_name}", field="catalog_name"
        )
    directory = Path(config_dir) if config_dir is not None else PROJECT_DIR
    return directory.resolve() / "catalogs" / f"{catalog_name}.yaml"


def read_yaml_unique(path):
    """读取 YAML，拒绝任意层级的重复映射键。"""
    path = Path(path)
    class UniqueKeyLoader(yaml.SafeLoader):
        pass

    def construct_mapping(loader, node):
        mapping = {}
        for key_node, value_node in node.value:
            key = loader.construct_object(key_node, deep=True)
            if key in mapping:
                raise ResourceDictionaryError(
                    "DUPLICATE_RESOURCE_ID",
                    f"重复定义资源键 {key!r}（第 {key_node.start_mark.line + 1} 行）",
                    path=path,
                    field=str(key),
                )
            mapping[key] = loader.construct_object(value_node, deep=True)
        return mapping

    UniqueKeyLoader.add_constructor(
        yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, construct_mapping
    )
    with path.open("r", encoding="utf-8") as stream:
        return yaml.load(stream, Loader=UniqueKeyLoader)


def read_catalog(catalog_name, config_dir=None):
    """读取资源映射，拒绝任意层级的重复 YAML 键，并返回独立数据。"""
    path = _catalog_path(catalog_name, config_dir)
    try:
        catalog = read_yaml_unique(path)
    except ResourceDictionaryError:
        raise
    except (OSError, yaml.YAMLError) as exc:
        raise ResourceDictionaryError(
            _CATALOG_CODES[catalog_name], f"无法读取或解析资源字典：{exc}", path=path
        ) from exc

    if not isinstance(catalog, dict):
        raise ResourceDictionaryError(
            _CATALOG_CODES[catalog_name], "资源字典顶层必须是 ID 到配置的映射", path=path
        )
    for entry_id, entry in catalog.items():
        if not isinstance(entry_id, str) or not entry_id.strip():
            raise ResourceDictionaryError(
                _CATALOG_CODES[catalog_name], "资源 ID 必须是非空字符串",
                path=path, field=str(entry_id),
            )
        if not isinstance(entry, dict):
            raise ResourceDictionaryError(
                _CATALOG_CODES[catalog_name], f"资源 {entry_id!r} 必须是映射",
                path=path, field=entry_id,
            )
    return deepcopy(catalog)


def _get_entry(catalog_name, entry_id, config_dir):
    catalog = read_catalog(catalog_name, config_dir)
    if entry_id not in catalog:
        kind = {"robots": "ROBOT", "scenes": "SCENE", "tasks": "TASK"}[catalog_name]
        raise ResourceDictionaryError(
            f"{kind}_NOT_FOUND", f"{catalog_name}.yaml 中找不到 ID {entry_id!r}",
            path=_catalog_path(catalog_name, config_dir), field=str(entry_id),
        )
    return catalog[entry_id]


def list_robots(config_dir=None):
    """返回当前 Robot Dictionary 中排序后的 ID。"""
    return sorted(read_catalog("robots", config_dir))


def get_robot_info(robot_id, config_dir=None):
    """返回指定机器人数据，并解析相对模型路径。"""
    entry = _get_entry("robots", robot_id, config_dir)
    directory = Path(config_dir).resolve() if config_dir is not None else PROJECT_DIR
    for key in ("urdf", "robodk_model"):
        if key in entry:
            if not isinstance(entry[key], (str, Path)) or not str(entry[key]).strip():
                raise ResourceDictionaryError(
                    "INVALID_ROBOT_CONFIG", f"{robot_id!r} 的 {key} 必须是非空路径",
                    path=_catalog_path("robots", config_dir),
                    field=f"robots.{robot_id}.{key}",
                )
            value = Path(entry[key]).expanduser()
            entry[key] = value if value.is_absolute() else (directory / value).resolve()
    return entry


def list_scenes(config_dir=None):
    """返回当前 Scene Dictionary 中排序后的 ID。"""
    return sorted(read_catalog("scenes", config_dir))


def get_scene_info(scene_id, config_dir=None):
    """返回指定场景数据的独立副本。"""
    return _get_entry("scenes", scene_id, config_dir)
