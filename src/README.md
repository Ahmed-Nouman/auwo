# AUWO Digital Twin Workspace (Centria)

ROS 2 workspace for the **AUWO** project digital twin: excavator + dump truck in Gazebo Harmonic, with RViz visualization, trajectory-smoothed control, and optional sensors (IMU, LiDAR). This repository reflects **Centria-side** development.

**Maintainer:** Ahmed Nouman (GitHub: [@AhmedNouman](https://github.com/AhmedNouman) — update with your username if different)

## Overview

- **Excavator**: 4-DoF (body, boom, stick, bucket), controlled via **arm_trajectory_controller** (JointTrajectoryController) with a trajectory adapter for smooth motion. **IMU** and **3D LiDAR** (gpu_lidar) sensors on the body; topics `/imu` and `/points` bridged from Gazebo.
- **Dump truck**: Static model (Pete-style) for dump target visualization.
- **Environment**: Ground plane, sun, and Pete environment mesh in `excavation_site_local.sdf`.

## Quick start

```bash
cd /home/ra2/Devel/auwo_ws
colcon build
source install/setup.bash
ros2 launch excavator_interactive_rviz auwo_twin.launch.py
```

- **Gazebo**: Server loads the **excavation site** world by default (`excavation_site_local.sdf`: ground, sun, and terrain mesh from the Pete dump-truck source). The **GUI** starts ~3 s after the server (`gz sim -g`); excavator spawns at ~4 s, truck at ~6 s. Optional: `gazebo_unified_gui:=true`, `gazebo_verbose:=0–4`. For a plain world without terrain, pass `world:=/path/to/empty.sdf`.
- **RViz**: Fixed frame `world`; RobotModel displays for Excavator and Truck (topics `/excavator_robot_description`, `/truck_robot_description`). The truck RViz URDF uses **`pete_dump_truck.stl`** for reliable Ogre loading; Gazebo spawn still uses the GLB URDF.
- **Control**: Use the **Preset Poses** panel: enable motion, choose Simulation, then **Idle** / **Dig** / **Dump** / **Transport**, or **Run Cycle**. Interactive markers also available. Commands go through the trajectory adapter for smooth motion.
- **Sensors**: `/imu` (sensor_msgs/Imu), `/points` (sensor_msgs/PointCloud2 from 3D LiDAR). See **Visualizing sensors in RViz** below.

## Visualizing sensors in RViz

With `view.rviz` (loaded by the digital twin launch):

- **3D LiDAR**: Display **"3D LiDAR"** (PointCloud2). Topic: `/points`. Shows the excavator’s 3D scan. If nothing appears, ensure the excavator is spawned in Gazebo and the bridge is running; you can set the topic’s **Reliability** to **Reliable** if needed.
- **IMU orientation**: Display **"IMU orientation"** (Pose). Topic: `/imu_pose`. Shows the excavator’s orientation as axes (published by `imu_to_pose` from `/imu`). Position is fixed (e.g. 0.5, 0, 1.5 in `world`); the axes rotate with the IMU quaternion.

Both displays are enabled by default. To add them manually: **Add** → **By display type** → **PointCloud2** (topic `/points_viz`) or **Pose** (topic `/imu_pose`). Fixed frame: `world`.

**If `/imu` or `/points` have no data:** (1) **Press Play** in the Gazebo window. (2) The launch sets `GZ_PARTITION=auwo_sim` so the bridge and gz sim share the same transport partition; the bridge maps gz `/excavator/imu` and `/excavator/points` to ROS `/imu` and `/points`. (3) To verify gz data from another terminal, use the same partition: `GZ_PARTITION=auwo_sim gz topic -e -t /excavator/imu`.

## Packages

| Package | Role |
|--------|-----|
| `excavator_description` | Excavator URDF/xacro, meshes, world SDF, controller config, RViz config |
| `truck_description` | Dump truck URDF/xacro and meshes (GLB for Gazebo, STL for RViz) |
| `excavator_gazebo` | Gazebo launch, robot spawn, clock/sensor bridge, publish_robot_descriptions |
| `excavator_interactive_rviz` | RViz launch, interactive markers, twin router |
| `excavator_teleop` | Teleop script |
| `excavator_cycle` | Automatic dig/dump cycle (publishes target positions) |
| `excavator_moveit_config` | MoveIt2: `move_group` + RViz MotionPlanning for the 4-DoF arm (`arm_trajectory_controller`) |

## MoveIt2 (interactive arm planning)

**All-in-one (Gazebo + MoveIt):** default launch starts Gazebo with **`use_sim_time:=true`**. Gazebo (**split** by default: `gz sim -s -r` server, then `gz sim -g` GUI after 3 s) + **event-driven** `move_group` (starts after `/arm_trajectory_controller/follow_joint_trajectory` exists) + MoveIt RViz:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --packages-select excavator_gazebo excavator_moveit_config --symlink-install
source install/setup.bash
ros2 launch excavator_moveit_config bucket_moveit.launch.py
```

**Mock only (no Gazebo window):** **`include_gazebo:=false use_sim_time:=false`** (local mock `ros2_control`, no sim clock).

If Gazebo starts but `move_group` / RViz never launches, first rebuild + re-source as above so installed launch files are in sync with source changes.

**If Gazebo opens but stays empty / terminal waits forever on controllers:** ensure **`GZ_PARTITION`** matches across processes (fixed in `excavator_gazebo` for `gz sim`, bridges, and `ros_gz_sim create`). Controllers are spawned **sequentially** (`joint_state_broadcaster` then `arm_trajectory_controller`) to avoid `controller_manager` races. For a lighter world while testing, use **`use_excavation_site:=false`** and optionally **`world:=$(ros2 pkg prefix excavator_description)/share/excavator_description/worlds/empty.sdf`**.

**RViz + MoveIt only (no Gazebo):** mock `ros2_control` hardware (`mock_components/GenericSystem`) runs `joint_state_broadcaster` and `arm_trajectory_controller` on the same machine so Plan and Execute use a consistent `/joint_states` and local `FollowJointTrajectory` server. Do not run this alongside another stack that already owns `/controller_manager` on the same `ROS_DOMAIN_ID`.

```bash
ros2 launch excavator_moveit_config demo_moveit_rviz.launch.py
```

Mock `ros2_control` sets **initial joint positions inside URDF limits** (arm joints are negative-only in this model). If you still see `CheckStartStateBounds` / “start state out of bounds”, the pose in RViz does not match `/joint_states`—use **Plan** from the current monitored state, or drag the interactive marker back into a valid configuration.

**Twin in one terminal, MoveIt in another (no second Gazebo, no second RViz):**

```bash
# Terminal 1
ros2 launch excavator_interactive_rviz auwo_twin.launch.py

# Terminal 2
ros2 launch excavator_moveit_config bucket_moveit.launch.py include_gazebo:=false launch_rviz:=false
```

- **Planning group**: `excavator_arm` (chain to `bucket` link). In **MotionPlanning**, **Plan** then **Execute**.
- **Trajectory path to Gazebo**: MoveIt sends `FollowJointTrajectory` goals to `/arm_trajectory_controller/follow_joint_trajectory`.
- **Preset panel Simulation mode note**: this toggle affects the twin router (`/arm_position_controller/commands`) and is **independent** of MoveIt execution.
- **`launch_rviz` default is `true`** for the all-in-one flow; use `launch_rviz:=false` with `auwo_twin` so only one RViz runs. With `include_gazebo:=false`, `move_group` starts immediately (no wait).
- **Startup timing**: with `include_gazebo:=true`, `bucket_moveit.launch.py` waits **`gazebo_controller_ready_delay_sec`** (default **35**), then runs `wait_for_arm_trajectory_action.py` until controllers + the `FollowJointTrajectory` action server are ready, then starts `move_group`. Tune **`wait_move_group_timeout_sec`** (default **120**) if spawn is very slow.
- **Gazebo GUI mode** (`excavator_gazebo`): `gazebo_unified_gui` defaults to **`false`** (split server + GUI). Set **`gazebo_unified_gui:=true`** for a **single** `gz sim` process (GUI+server); some machines get fewer “not responding” warnings that way. If spawn/controllers misbehave in unified mode, stay on split (default).
- **No Gazebo window**: use **`headless:=true`** on `bucket_moveit.launch.py` (or `ros2 launch excavator_gazebo gazebo.launch.py headless:=true`) to run **server only** — physics, controllers, and MoveIt still work; use MoveIt RViz (or another client) to visualize. The delayed **`gz sim -g`** step is skipped entirely.
- **Gazebo log verbosity**: `gazebo_verbose` defaults to **`1`** (`gz sim -v`; range 0–4). Use **`gazebo_verbose:=4`** for deep debug; lower values reduce console I/O during startup.
- **`use_sim_time`**: defaults **`true`** with Gazebo (required for `/clock` and MoveIt); set **`false`** when **`include_gazebo:=false`** (mock-only).
- **Dump truck obstacle**: `publish_truck_obstacle` default `true` (coarse box on `/collision_object`). Tune `truck_box_*` or set `publish_truck_obstacle:=false`.
- **OMPL / “no planning library” in RViz**: (1) Install and source the Debian plugin package: `sudo apt install ros-${ROS_DISTRO}-moveit-planners-ompl`, then `source /opt/ros/${ROS_DISTRO}/setup.bash && source install/setup.bash`. (2) **`/clock` must be publishing** if `use_sim_time` is true — without sim time, `move_group` and the MotionPlanning plugin often misbehave (planner dropdown stuck on **Unspecified**, execution fails). See **Sim clock / `/clock`** below. (3) Wait until **`move_group`** is running — with `include_gazebo:=true`, MoveIt RViz starts after the trajectory-action wait; look for **“You can start planning now!”** in the `move_group` log. (4) `ompl_planning.yaml` sets **`excavator_arm.default_planner_config: RRTConnect`** for the planning group.

### MoveIt: Plan OK, Execute fails (incl. `headless:=true`)

Planning only needs the model and scene; execution sends a **`FollowJointTrajectory`** goal to **`arm_trajectory_controller`**. If Execute aborts immediately or mid-path:

1. **Read the terminal running `move_group`** — look for `ABORTED`, `velocity`, `tolerance`, or `controller` messages.
2. **`joint_trajectory_controller`** (ROS 2) **rejects** goals whose **last waypoint has non-zero velocity** unless **`allow_nonzero_velocity_at_trajectory_end`** is true. This workspace sets that in **`excavator_description/config/controllers.yaml`** for **`arm_trajectory_controller`** so MoveIt time-parameterized paths are accepted. Rebuild and respawn controllers after changing it (`colcon build --packages-select excavator_description`, then restart the launch).
3. **Start-state mismatch**: if the first trajectory point does not match **`/joint_states`**, MoveIt aborts with **`Invalid Trajectory: start point deviates ... more than ... at joint`** (see `move_group` log). In RViz, plan from the **current** state. Tolerance is **`trajectory_execution.allowed_start_tolerance`** in **`excavator_moveit_config/config/excavator_controllers.yaml`** (must live under the **`trajectory_execution:`** YAML block — MoveIt 2 ignores a flat `allowed_start_tolerance` key).
4. **Sim time**: **`ros_gz_bridge`** nodes run with **`use_sim_time:=false`** so they publish **`/clock`** without waiting for it. With **`use_sim_time`** elsewhere, **`ros2 topic hz /clock`** should show a rate ~1.5 s after sim start. If empty, **`source install/setup.bash`** in the same shell and check **`ros2 topic list | grep clock`**. Clock uses **`clock_bridge.yaml`**; IMU/LiDAR use **`sensors_bridge.yaml`**.

### MoveIt troubleshooting (RViz moves but Gazebo does not)

1. **Check action server exists**

```bash
ros2 action list | grep follow_joint_trajectory
ros2 action info /arm_trajectory_controller/follow_joint_trajectory
```

2. **Check controllers are active**

```bash
ros2 control list_controllers
```

`arm_trajectory_controller` and `joint_state_broadcaster` should be `active`.

Also confirm the action server exists before first Execute:

```bash
ros2 action list | grep /arm_trajectory_controller/follow_joint_trajectory
```

3. **Confirm Gazebo is playing**

- If paused, MoveIt can execute on paper while simulation state does not advance.

**Gazebo GUI / OS “Application is not responding”**

This is often **not a crash**. GNOME/KDE mark the window unresponsive when the **Qt main thread** is busy loading **Ogre2**, scene sync from the server, and GUI plugins—especially right after `gz sim -g` attaches. **MoveIt RViz** also uses OpenGL, so **two heavy GL apps** can extend that pause on integrated GPUs.

- **Wait 30–60 s** on first open; if the window recovers, it was startup blocking.
- Click the Gazebo window and press **Play** to run physics and advance sim time (server starts **paused** by default).
- **Try unified GUI**: `ros2 launch excavator_moveit_config bucket_moveit.launch.py gazebo_unified_gui:=true` (one `gz sim` process; sometimes smoother than split `-s`/`-g`).
- **GPU / driver**: `LIBGL_ALWAYS_SOFTWARE=1 ros2 launch excavator_moveit_config bucket_moveit.launch.py` (slow but stable) to test driver-related freezes.
- **Session**: compare **Xorg** vs **Wayland** if your desktop allows; Gazebo may force **xcb** under Wayland.
- **Less console noise**: `gazebo_verbose:=1` (default) vs `4` for full Gazebo debug.
- **Note:** MoveIt `move_group` timing does not fix a stuck Gazebo GUI; that is display/event-loop or GPU load, not the trajectory wait script.

4. **Check sim clock is present (when `use_sim_time:=true`)**

```bash
ros2 topic hz /clock
```

5. **Watch `move_group` logs during Execute**

- Look for `FollowJointTrajectory` failures (`ABORTED`, start-state tolerance errors, or action unavailable).
- If needed, tune `allowed_start_tolerance` in `src/excavator_moveit_config/config/excavator_controllers.yaml` and/or controller tolerances in `src/excavator_description/config/controllers.yaml`.

The digital twin launch **`auwo_twin.launch.py`** stays on the classic stack (twin RViz + markers + teleop router, no MoveIt wiring).

## Daily / timeline log (Centria)

*One block per calendar day, **reverse chronological order** (newest → oldest). Add a new `### YYYY-MM-DD` **immediately below this paragraph** when you log work.*

### 2026-04-14 (ROS domain defaults for MoveIt launches)

- Changed default **`ros_domain_id`** to **`0`** in **`excavator_moveit_config/launch/bucket_moveit.launch.py`**.
- Changed default **`ros_domain_id`** to **`0`** in **`excavator_moveit_config/launch/demo_moveit_rviz.launch.py`**.
- Updated launch-file header comments/docstrings to match the new default domain behavior.

### 2026-03-25 (MoveIt RViz execution, Gazebo pause, naming, environment mesh)

- **MoveIt + RViz scene robot**: Added static **`world` → `base_link`** TF for SRDF virtual joint (`demo_moveit_rviz`, mock **`bucket_moveit`** path). **`excavator_gazebo`**: always publish **`excavator_world_tf`** at excavator spawn (not only `spawn_dumper`) so **`world`** exists without the dump-truck group. **`moveit.rviz`**: absolute **`/monitored_planning_scene`** and **`/display_planned_path`**.
- **Gazebo**: Removed **`-r`** from **`gz sim`** so the sim starts **paused**; press **Play** to run physics and advance **`/clock`**.
- **Excavator naming**: Mesh files **`arm1` / `arm2` / `shovel`** → **`boom` / `stick` / `bucket`** (`.dae`); URDF links and joints → **`boom`**, **`stick`**, **`bucket`**, **`boom_rotation`**, **`stick_rotation`**, **`bucket_rotation`**; SRDF, controllers, MoveIt YAML, teleop, cycle, interactive RViz, and **`view.rviz`** updated accordingly.
- **Pete environment**: Renamed **`untitled.stl`** → **`environment.stl`**; **`model.sdf`** URI and docs/comments updated.

### 2026-03-22 (auwo_twin launch run + shutdown findings)

- **Digital twin startup verification**: `auwo_twin.launch.py` started excavator/truck publishers, Gazebo server+GUI, RViz, joint markers, twin router, trajectory adapter, and both bridges (`/clock`, `/imu`, `/points`).
- **Spawn and bridge status**: Excavator and truck entities spawned successfully; `joint_state_broadcaster` was already active and skipped by `spawn_controller_safe.sh`; IMU and point cloud bridges reported successful GZ->ROS mappings.
- **RViz/runtime observations**: RViz subscribed to point cloud stream (`/points_viz`) and loaded normally; plugin portability warning and duplicate rosout publisher warning were observed but did not block operation.
- **Interrupt/shutdown behavior**: On Ctrl-C, most nodes exited cleanly; both `ros_gz_bridge` bridge nodes ended with exit code `-11` during teardown, and the delayed `arm_trajectory_controller` spawner exited with `-2` after interruption.

### 2026-03-20 (truck_description package, spawn reduction, RViz orientation)

- **truck_description package**: New `truck_description` package with dump truck URDF/xacro and meshes (GLB for Gazebo, STL for RViz). Truck assets moved from `excavator_description`; `excavator_description` no longer installs truck meshes.
- **Spawn time reduction**: Excavator spawn 6 s → 4 s; clock bridge 8 s → 6 s; controllers 9 s → 7 s; truck group delay 8 s → 6 s.
- **RViz truck orientation**: Added `pete_visual_rpy` xacro arg; RViz URDF uses `pete_visual_rpy:=0 0 0` for STL (Z-up) so truck matches Gazebo orientation. GLB keeps default roll +90° for Gazebo spawn.
- **Gazebo spawn race fix**: Truck RSP remapped to `/truck_robot_description_internal`; truck group delayed to 6 s so excavator spawn from `/robot_description` is deterministic first.

### 2026-03-20 (RViz truck STL visual)

- **Truck RViz mesh**: `truck.urdf.xacro` gains `pete_visual_ext` (default `glb`). Gazebo launch generates `/tmp/truck_rviz.urdf` with `pete_visual_ext:=stl` so RViz uses `pete_dump_truck.stl`. Gazebo spawn still uses default GLB. Truck `robot_state_publisher` remapped to `/truck_robot_description_internal` so it does not overwrite `/truck_robot_description` with the GLB string.

### 2026-03-20

### 2026-03-08 (trajectory controller + IMU + 3D LiDAR)

- **`bucket_moveit.launch.py`**: forwards **`gazebo_unified_gui`**, **`gazebo_verbose`**, and **`headless`** to **`excavator_gazebo`**.
- **`gazebo.launch.py`**: **`gazebo_unified_gui`** (default **`false`**) — set **`true`** for single-process **`gz sim`** if split GUI triggers OS “not responding”. **`gazebo_verbose`** (default **`1`**) for **`gz sim -v`** (0–4; previously hard-coded **`4`**).
- **MoveIt execute**: **`arm_trajectory_controller`** — **`allow_nonzero_velocity_at_trajectory_end: true`** and **`constraints.goal_time`** in **`excavator_description/config/controllers.yaml`** so MoveIt trajectories are not rejected for non-zero end velocity; **`allowed_start_tolerance`** nudged in **`excavator_controllers.yaml`**.
- **Gazebo ↔ ROS bridge**: **`clock_bridge.yaml`** only **`/clock`**; **`sensors_bridge.yaml`** for IMU/LiDAR; bridge nodes run with **`use_sim_time:=false`** so **`/clock`** is published without a deadlock. Timers: clock ~**1.5 s**, sensors ~**5 s**. **`moveit_rviz.launch.py`** prepends **`moveit_planners_ompl`** on **`AMENT_PREFIX_PATH`** like **`move_group`**.
- **MoveIt controllers YAML**: **`trajectory_execution.allowed_start_tolerance`** (nested under **`trajectory_execution:`**, not a flat key). **`excavator.srdf`**: end-effector **`bucket`** on link **`bucket`**. **`bucket_rotation`** URDF lower limit relaxed slightly for Gazebo/MoveIt bounds alignment.
- **README**: “Not responding” GUI notes; **`headless:=true`**; **Plan OK / Execute fails** troubleshooting (logs, clock, tolerances).

### 2026-03-11

- **`excavator_moveit_config`**: MoveIt2 for the 4-DoF arm — SRDF, KDL, OMPL pipeline, **`excavator_controllers.yaml`** → **`arm_trajectory_controller`**, **`joint_limits.yaml`**, truck box on **`/collision_object`**.
- **`bucket_moveit.launch.py`**: **Gazebo** when **`include_gazebo:=true`**; **`move_group`** only after **`wait_for_arm_trajectory_action.py`** sees **`/arm_trajectory_controller/follow_joint_trajectory`** (event-driven; **`wait_move_group_timeout_sec`** default 180 s). **MoveIt RViz** starts with **`move_group`** so MotionPlanning loads OMPL. Default **`launch_rviz:=true`**. With **`auwo_twin`** already running: **`include_gazebo:=false launch_rviz:=false`**.
- **Trajectory**: **`move_group`** remaps to **`/arm_trajectory_controller/follow_joint_trajectory`**.
- **OMPL**: **`moveit_planners_ompl`** in **`package.xml`**; **`ompl_planning.yaml`** → **`excavator_arm.default_planner_config: RRTConnect`**; **`move_group.launch.py`** prepends **`moveit_planners_ompl`** to **`AMENT_PREFIX_PATH`** when present.
- **Gazebo (MoveIt flow)**: Default **split** **`gz sim -s`** (paused until Play) + delayed **`gz sim -g`**; excavator spawn **`-world default`**, **`ros_gz_sim create`** from resolved URDF **`-file`** for reliable gz transport discovery with immediate spawn. **`headless:=true`** = server only.
- **`auwo_twin.launch.py`**: Classic twin only (no MoveIt on RViz). **`view.rviz`**: no embedded MotionPlanning.
- **Workflow**: **`bucket_moveit`** for dig/dump poses → joint targets into **`auwo_twin`** run-cycle presets.

### 2026-03-08

- **Documentation**: Added this README (overview, quick start, packages, timeline).
- **Smooth motion / control**: `arm_position_controller` → **`arm_trajectory_controller`** (JointTrajectoryController) in **`controllers.yaml`**; Gazebo launch spawns it. **`trajectory_command_adapter`** in **`excavator_teleop`** (from **`auwo_twin.launch.py`**) maps **`/arm_position_controller/commands`** to smooth **`JointTrajectory`** (0.6 s, 15 points); excavation cycle / RViz markers unchanged.
- **Rollback**: World temporarily back to **`empty.sdf`** during session reset.
- **Gazebo launch**: **Server-first** — **`gz sim -s … world.sdf`** loads the world immediately, **GUI** via **`gz sim -g`** after ~3 s (avoids GUI-first “pick a world” with nothing spawned). Spawn, clock bridge, and controller startup sequenced so **`/world/default/create`** and **`/clock`** exist before controllers.
- **Sensors & bridge**: URDF links **`sensor_imu_link`**, **`sensor_lidar_link`** (fixed to body). Gazebo **IMU** (~100 Hz) → **`sensors_bridge.yaml`** → **`/imu`**; **`imu_to_pose`** → **`/imu_pose`** for RViz. **3D gpu_lidar** → **`/points`** (PointCloud2). **`clock_bridge.yaml`** is **clock only** (`/clock`); sensors use **`sensors_bridge.yaml`**. World loads Imu (and related) systems as needed.

### Earlier (summary)

- Digital twin launch: Gazebo (excavation_site world) + RViz + dump truck; excavator and truck spawn; robot descriptions published to dedicated topics for RViz.
- Dump truck: Pete-style GLB mesh, scaling/orientation adjusted; truck at (4, 3) to avoid overlap with excavator.
- Excavation cycle: Wall-time state machine (PREPARE_DIG → SCOOP_AND_SLEW → LIFT_AND_ALIGN_DUMP → DUMP → RETURN_HOME); warmup 15 s; publishes target positions at 10 Hz.
- Environment: Pete environment mesh (environment.stl) in world; previous pit model removed.
- Controllers: joint_state_broadcaster + arm_trajectory_controller; clock bridged for sim time; publish_robot_descriptions ensures `/robot_description` is excavator (for gz_ros_control) and RViz topics are fed.

---

*Responsible: Centria side (Ahmed Nouman). Electric excavator and dump truck hardware expected in 2–3 months; digital twin and trajectory/sensor setup continue in the meantime.*
