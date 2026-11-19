# PIVOT — Planning and Integrated Validation of Offline Trajectories

English · [简体中文](README.zh-CN.md)

PIVOT is an offline motion planning demo for a six-axis robot. Its pipeline runs from resource selection to trajectory export: a manually taught reference path, Bi-RRT Connect, shortcutting, Ruckig time parameterization, dense trajectory validation, and CSV and plot output. A separate demo shows approach, transport, and retreat around an assumed pick pose. This repository implements the planning, shortcutting, validation, and dictionary-based configuration code; Ruckig, robot modeling, and visualization are provided by third-party libraries.

This release excludes the RoboPlan experiments and the RoboDK simulation script. RoboPlan's organization of robots, scenes, and tasks influenced the design, but PIVOT has no runtime dependency on RoboPlan.

## Environment

The project has been tested on Windows with Python 3.11.16 and the package versions in `requirements.txt`. A fresh installation is recommended, although some platforms may need additional system components for the robotics libraries and collision backends.

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Run the default AR2010 transport demo:

```powershell
$env:MPLBACKEND = 'Agg'
.\.venv\Scripts\python.exe main.py
```

Run the staged pick-and-place demo with an assumed grasp pose:

```powershell
.\.venv\Scripts\python.exe main_pick_place.py
```

Both entry points accept `--viewer` for Viser playback at `http://localhost:8080`; use `--port` to select another port. `main.py --viewer` shows the transport plan. `main_pick_place.py --viewer` shows the workpiece stationary at the pick location, attached to the TCP during transport, and stationary after placement. The viewer is for offline demonstration only and does not actuate a gripper.

## Code and resources

- `main.py` runs the transport pipeline; `main_pick_place.py` runs the three-stage pick-and-place demo.
- `rrt_planner.py`, `shortcut.py`, and `ruckig_planner.py` perform path search, shortcutting, time parameterization, and dense trajectory validation.
- `collision.py` and `kinematic_constraints.py` check the robot, attachments, scene collisions, and TCP orientation.
- `resource_dictionary.py` reads resources, `config.py` combines and validates them, and `robot_scene.py` creates robot and scene objects.
- `catalogs/robots.yaml`, `scenes.yaml`, and `tasks.yaml` hold robot, environment, and task data. `config.yaml` selects their IDs and sets run parameters. See the [dictionary reference](catalogs/README.md) for fields and units.
- `models/ar2010_ros_industrial/` contains third-party AR2010 meshes and a derived URDF. See the [model provenance](models/ar2010_ros_industrial/README.md) and [third-party notices](THIRD_PARTY_NOTICES.md).

Joint values use radians inside the planner and degrees in YAML; RPY angles in YAML use degrees, and lengths use meters. The default random seed is fixed. Motion limits are offline demo parameters, not manufacturer-certified or controller safety limits. `empty_scene` means scene objects are excluded from planning collision checks; it does not represent a verified obstacle-free workspace. `step_scr5` is a provisional resource without confirmed acceleration and jerk limits or a controller forward-kinematics comparison, so it is not a complete planning example.

## Verification

From the repository root, run:

```powershell
.\.venv\Scripts\python.exe -m pytest -q -p no:cacheprovider tests
```

Tests cover configuration and resource validation, scene switching, AR2010 offline planning, pick-and-place stages, attachments, and the final export gate. Running either demo updates files with matching names in `output/`; that directory is ignored by Git. Passing software tests does not qualify a trajectory for a physical robot. The model, tooling, controller limits, and site safety require separate checks.

## License

Project-authored code is released under [MIT](LICENSE). The AR2010 model is separately attributed under its upstream BSD-3-Clause license; Python dependencies retain their respective upstream licenses.
