# PIVOT — 离线轨迹规划与一体化复核

**Planning and Integrated Validation of Offline Trajectories**

[English](README.md) · 简体中文

本仓库演示六轴机器人从资源配置到轨迹导出的离线规划链路：Manual Teach 参考路径 → Bi-RRT Connect → Shortcut → Ruckig → 稠密轨迹复核 → CSV 与图表。另有一个假设抓取姿态的分阶段抓放演示。规划、路径简化、复核和词典配置代码在本仓库实现；Ruckig、机器人建模和可视化由第三方库提供。

本仓库不包含 RoboPlan 实验链路或 RoboDK 仿真脚本。代码参考过 RoboPlan 对机器人、场景和任务的组织思路，但运行时不依赖 RoboPlan。

## 运行环境

已在 Windows、Python 3.11.16 与 `requirements.txt` 列出的版本组合下验证。建议在新环境中安装依赖；机器人库及其几何后端在不同平台上可能需要额外的系统组件。

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

运行默认 AR2010 搬运示例：

```powershell
$env:MPLBACKEND = 'Agg'
.\.venv\Scripts\python.exe main.py
```

运行假设抓取姿态的接近、搬运、撤离演示：

```powershell
.\.venv\Scripts\python.exe main_pick_place.py
```

两个入口都支持 `--viewer`，使用 Viser 在 `http://localhost:8080` 回放；`--port` 可更换端口。`main.py --viewer` 显示搬运规划，`main_pick_place.py --viewer` 显示工件在取料位静止、随 TCP 搬运、放料后静止的状态。Viewer 仅作离线展示，不控制夹爪。

## 代码与资源

- `main.py`：完整搬运链路；`main_pick_place.py`：三阶段抓放演示。
- `rrt_planner.py`、`shortcut.py`、`ruckig_planner.py`：路径搜索、简化、时间参数化及稠密复核。
- `collision.py`、`kinematic_constraints.py`：机器人、附件、场景碰撞及 TCP 姿态检查。
- `resource_dictionary.py` 查询资源；`config.py` 组合与校验；`robot_scene.py` 创建机器人和场景实例。
- `catalogs/robots.yaml`、`scenes.yaml`、`tasks.yaml` 分别保存机器人、环境、任务；`config.yaml` 仅选择 ID 和设置本次运行参数。字段与单位见 [词典说明](catalogs/README.md)。
- `models/ar2010_ros_industrial/` 包含第三方 AR2010 网格和派生 URDF，来源与许可见 [模型说明](models/ar2010_ros_industrial/README.md)及 [第三方声明](THIRD_PARTY_NOTICES.md)。

规划内部关节量用弧度，YAML 中关节角和 RPY 用度，长度用米。默认随机种子固定；默认运动上限是离线 Demo 参数，不是厂家或控制器安全参数。`empty_scene` 只代表场景物体不参与规划碰撞检查。`step_scr5` 是尚缺加速度、jerk 和控制器正运动学对照的暂定资源，不作为完整流程示例。

## 验证

从仓库根目录运行：

```powershell
.\.venv\Scripts\python.exe -m pytest -q -p no:cacheprovider tests
```

测试包括配置和资源校验、场景切换、AR2010 离线规划、抓放阶段、附件及最终导出门禁。运行 `main.py` 或 `main_pick_place.py` 会更新 `output/` 中的同名文件；该目录不纳入版本控制。测试通过不代表轨迹可下发实机，模型、工装、控制器限制和现场安全仍需单独核对。

## 许可

本仓库自有代码以 [MIT](LICENSE) 发布。AR2010 模型来自 ROS-Industrial，按其 BSD-3-Clause 许可单独标注；依赖库各按其上游许可发布。
