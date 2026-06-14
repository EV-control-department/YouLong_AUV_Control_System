# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Workspace Structure

Two colcon workspaces in this repo:

```
YouLong_AUV_Control_System/
  auv_ws/       # AUV control stack (no sim dependency)
  sim_ws/       # Simulation overlay (sources auv_ws, adds stonefish_ros2)
```

### Build & Run — AUV Workspace (`auv_ws/`)

```bash
cd auv_ws
colcon build                          # build all packages
colcon build --packages-select uv_control  # build one package
colcon build --symlink-install        # symlink install (Python changes take effect without rebuild)
source install/setup.bash             # source the workspace overlay BEFORE any ros2 command

ros2 launch uv_bringup real_bringup.py   # real AUV stack
```

### Build & Run — Sim Workspace (`sim_ws/`)

```bash
cd sim_ws
colcon build                          # builds stonefish_ros2 only
source install/setup.bash             # also sources auv_ws underlay

ros2 launch uv_bringup sim_bringup.py   # full simulation stack
```

Note: `sim_ws` uses colcon overlay — source `auv_ws/install/setup.bash` first, then `sim_ws/install/setup.bash`.

## Architecture

This is a **ROS 2 Jazzy** project for the **YouLong AUV** (autonomous underwater vehicle), designed for the SAUVC competition. It uses colcon with mixed ament_python (application logic) and ament_cmake (custom messages, Stonefish wrapper) packages.

### 6-Layer Stack (bottom-up)

| Layer | Package | Role |
|---|---|---|
| 0 | `stonefish_ros2` (C++) | Stonefish 1.6 marine physics simulator, NO GPU mode |
| 1 | `uv_hm` | Cascaded PID + thruster mixing → 6 thrusters. `sim_bridge` for simulation, `hw_manager` placeholder for STM32 MCU |
| 2 | `uv_control` | Motion API: SET/WMOVE/BMOVE/TRAVEL. Only entry point is `basic_motion` |
| 3 | `uv_perception` | YOLO object detection + monocular ray intersection for 3D positioning |
| 4 | `uv_nav` | A* pathfinding + obstacle avoidance waypoint following |
| 5 | `uv_task` | JSON-based competition task runner |

### Coordinate System & Conventions

- **NED** (North-East-Down) throughout. Yaw 0 = North, **clockwise positive**.
- Angles stored in **degrees** internally, converted to radians for ROS messages.
- `Coordinate` class (`uv_control/coordinate.py`): 6-DOF NED with 4×4 homogeneous transform matrices.

### Control Modes (MotionCommand.type_mask)

The `type_mask` bitmask controls which axes are active. **CRITICAL — inverse logic: 0 = axis enabled, 1 = axis disabled.**

| Bit | Axis | Meaning when set (1) |
|---|---|---|
| bit0 | x | ignore x |
| bit1 | y | ignore y |
| bit2 | z | ignore z |
| bit3 | yaw | ignore yaw |

`0x0F` means **ALL axes disabled**, not enabled. `0x00` means all axes enabled.

9 control modes: `POS_WORLD`, `POS_BODY`, `VEL_WORLD`, `VEL_BODY`, `FORCE_BODY`, and step variants of each.

## Package Types

- **ament_python**: `uv_control`, `uv_hm`, `uv_nav`, `uv_perception`, `uv_task`, `uv_bringup`
- **ament_cmake**: `uv_msgs`, `zit6_interfaces`, `stonefish_ros2` (define custom `.msg`/`.srv` files)

## Conventions

- **Entry points** defined in `console_scripts` of `setup.py`; `launch/` dir at package root
- **Imports order**: stdlib → rclpy → ROS msgs → local packages (blank-line separated)
- **Architecture**: thin ROS nodes that import core logic from sibling modules. Each `package/__init__.py` is empty; actual code is in `package/module.py`.
- `.pt` YOLO weights under `src/datas/` are **gitignored** — models must be provided separately
- `stonefish_ros2` depends on external https://github.com/patrykcieslak/stonefish library
- No tests exist yet (infrastructure declared in `package.xml` but no test files)
