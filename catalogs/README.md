# Robot / Scene Dictionary

`config.yaml` 的 `ids` 决定运行时使用哪个 `robots.yaml`、`scenes.yaml`、`tasks.yaml` 条目。资源文件只是数据；`resource_dictionary.py` 提供查询，`config.py` 负责组合与校验，`robot_scene.py` 将已选数据构建成 URDF 机器人与碰撞几何。配置文件和字典中的重复 YAML 键会报错，不会静默使用后一个值。

## 字段归属和单位

- `robots.yaml`：机器人名称、自由度、`urdf`、可选后续仿真模型路径 `robodk_model`、`tcp_link`、基座位姿、URDF 无效限位时使用的回退关节范围，以及逐轴 Ruckig 上限 `motion_limits`。相对模型路径按所选 `config.yaml` 所在目录解析。模型文件和 TCP link 必须与实际机器人核对。当前公开版不包含后续仿真脚本。
- `scenes.yaml`：`collision_safety_margin_m` 与 `objects`。目前物体仅支持 `type: box`（可省略）；每个物体有 `name`、`size_m`、`center_m`、`rpy_deg`、`collision_enabled`，`color` 用于显示。位置以机器人基座世界坐标系计，长度用米；RPY 使用角度、`xyz` 顺序。尺寸必须为正数，安全裕量可为零。
- `collision_enabled` 必须写成 YAML 布尔值 `true` 或 `false`，不能写成带引号的字符串。
- `tasks.yaml`：任务的 `robot_id`，旧式单个 `scene_id` 或 `compatible_scene_ids` 列表（二者只能选一），起终关节角、可选人工示教候选点和夹具/工件附件。关节角用度；`attached_bodies` 的尺寸及 TCP 相对位移用米，姿态用度。
- `config.yaml`：当前选择、RRT/Shortcut/验证参数、Ruckig 采样周期和输出设置。`planner.ruckig` 可选填逐轴速度/加速度/jerk 上限，分别使用 deg/s、deg/s²、deg/s³；留空继承机器人字典中的值，填写时不能超过机器人上限。

AR2010 的 `motion_limits` 是本 Demo 的离线规划参数，不应视为厂家额定参数。新增机器人时必须根据模型、控制器和现场要求核实这些值，不能复制 AR2010 的数字作为新机器人的限制。

`step_scr5` 当前是根据厂家尺寸图建立的暂定规划资源。已记录六轴、负载、工作半径、关节范围、最大关节速度和近似轴间尺寸；URDF 使用典型协作六轴轴系与保守基础几何。厂家尚未提供加速度、jerk 和 RoboDK 模型，因此该资源暂不作为 `config.yaml` 的运行选择。补齐 `motion_limits.max_acceleration_deg_s2`、`motion_limits.max_jerk_deg_s3` 和 `robodk_model`，并完成控制器正运动学对照后，才能进入完整离线规划及仿真验证。

## 查询 API

在项目根目录启动 Python 后：

```python
from resource_dictionary import list_robots, get_robot_info, list_scenes, get_scene_info

print(list_robots())                         # ['ar2010']
print(list_scenes())                         # 按 ID 排序
robot = get_robot_info('ar2010')             # 独立深拷贝，模型路径已解析
scene = get_scene_info('empty_scene')        # 独立深拷贝
```

这些函数可选传 `config_dir`，表示目标 `config.yaml` 所在目录；不传时使用项目目录。资源找不到或格式错误会抛出 `ResourceDictionaryError`，其 `code`、`path`、`field` 可供程序检查。`load_config()` 将其转为同样带这些属性的 `ConfigError`。查询不创建 URDF 或几何实例；实际规划仍通过 `main.py` 和 `robot_scene.py`。

## 选择场景的两个例子

当前默认：

```yaml
ids:
  robot_id: ar2010
  scene_id: ar2010_transport_demo
  task_id: left_to_right_transport
```

仅把 `scene_id` 改为 `empty_scene`，其他字段不变：

```yaml
ids:
  robot_id: ar2010
  scene_id: empty_scene
  task_id: left_to_right_transport
```

`empty_scene` 保留现有三个物体供显示，但都关闭规划碰撞；`obstacle_scene` 只启用现有中央障碍物。它们复用原场景的尺寸和位姿，`obstacle_scene` 与默认场景目前在碰撞几何上相同。任务的 `compatible_scene_ids` 已包含这三个 ID。切换后须重新运行规划，不能将旧 CSV 当作新场景的结果。

## 增加资源

1. 在 `robots.yaml` 或 `scenes.yaml` 增加唯一 ID。机器人应提供已检查的 URDF、TCP、自由度及逐轴限制；场景应提供实测 box 尺寸/位姿及碰撞开关。
2. 在 `tasks.yaml` 新增适配任务或更新任务的 `robot_id` 与 `compatible_scene_ids`。任务的起终姿态和附件也应按目标机器人重新核对。
3. 在 `config.yaml` 选择相应 ID。`load_config()` 会在规划前校验缺失资源、URDF、几何及运动限制。
4. 先运行自动化配置测试和离线规划回归，再单独进行控制器与现场验证。当前 Demo 的可视化路径要求六轴及名为 `gripper` 和 `workpiece` 的附件；新增资源暂应保持这些约定。核心 Bi-RRT、Shortcut、Ruckig 算法无需因新增 ID 而改动。

本轮没有新增面向用户的 Task Dictionary 查询接口，也没有新增数据库或网络服务。旧式顶层内嵌 `robot`、`scene`、`task` 的配置仍可加载，并接受同一套校验。
