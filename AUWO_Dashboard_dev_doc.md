# AUWO Dashboard — Build Guide

**Living document.** Update it as each piece lands: tick the checklist in §4,
fill in the component section in §5, add a line to the verification log in §6.
Anything surprising goes in §8 so it is not rediscovered later.

Companion documents: *Dashboard Requirements*, *Interface Contract*,
*Integrated Architecture*, *Goals Tasks and Actions*, *Twin Runbook*.

---

## 1. What is being built

An operator dashboard, plus the execution machinery behind it, so a remote
operator can survey a site, mark out a region to excavate, and have the machine
either dig it automatically or be driven by hand — with the machine freezing
safely on any loss of control.

### Modes of operation

| mode | operator does | machine does | dig region |
|---|---|---|---|
| **M1 Manual** | drives the joystick | follows commands | optional, ignored |
| **M2 Assisted** | marks the region, starts it | runs dig cycles until the region reaches target depth | **required** |
| **M3 Supervised** | confirms a proposal | proposes a region *and* executes it | machine-authored |

**M2 is the primary target of this project.** M1 already works and is the
fallback that keeps the machine useful when anything else fails. M3 is out of
scope but is designed for.

The key structural point: **M2 and M3 share the entire execution path.** The
only difference is who *writes* the dig region — the operator, or a planner.
The situational model accepts either, and the Task's start condition is "a dig
region is set", with no clause about its author. So M3 is later added by one
new node writing an existing field, with nothing downstream changing.

The planner for M2 can be as dumb as it likes to begin with: pick a cell in the
region that still has material, run one dig-and-dump cycle, repeat. Improving
it never changes an interface.

**On hardware:** all three modes work in the twin. On the real excavator only
M1 works until electrohydraulic control exists — M2 and M3 need the machine to
accept motion commands. That is the honest split, and it is why M1 is built
first even though M2 is the goal.

**Not in scope yet:** the dumper, and the M3 proposal step.

---

## 2. Repository layout

Packages map to architecture layers, so a file's location says which layer it
belongs to.

```
auwo_ws/src/
├── excavator_description/        PLATFORM (L1)
│   ├── urdf/                     excavator.urdf.xacro, meshes, worlds
│   └── meshes/
├── excavator_moveit_config/      PLATFORM — MoveIt config + Isaac USD scenes
│   └── isaac-sim/                excavator_*.usd
├── excavator_gazebo/             legacy, kept for reference
├── novatron_xsite3d_interface/   real-machine interface
│
├── auwo_perception/              WORLD MODEL (L2)
│   └── scripts/
│       ├── lidar_mapper.py           survey map: union-forever, for coverage study
│       ├── terrain_map_node.py       operator map: latest-wins, layered   [TODO]
│       ├── dig_zone_monitor.py       dig-zone elevation + change detection
│       └── safety_monitor.py         swing-zone watch -> alerts, estop    [TODO]
│
├── auwo_control/                 ACTUATION
│   └── scripts/
│       ├── auwo_arbiter.py           SINGLE WRITER to the machine
│       ├── auwo_joy_teleop.py        Joy -> deadman-gated velocity -> position
│       ├── keyboard_to_joy.py        keyboard -> /joy, for testing without a pad
│       └── isaac_command_bridge.py   Float64MultiArray -> JointState for Isaac
│
├── auwo_operator/                OPERATOR (L4)
│   └── scripts/
│       ├── clicked_point_adapter.py  RViz clicks -> SetDigRegion          [TODO]
│       └── dig_region_node.py        region+depth -> target/remaining     [TODO]
│
├── auwo_task/                    EXECUTION (L3) — mode M2
│   └── scripts/
│       ├── dig_planner.py            region -> next dig pose              [TODO]
│       └── excavation_executor.py    dig pose -> cycle -> /auwo/cmd/auto  [TODO]
│
├── auwo_bringup/                 LAUNCH + CONFIG, no nodes
│   ├── launch/
│   │   ├── auwo.launch.py            EVERYTHING - one command, see section 2a
│   │                                 (supersedes isaac_twin.launch.py)
│   ├── config/
│   │   ├── sensors/*.yaml            cab_only, boom_only, roof_os0_plus_boom...
│   │   └── rviz/*.rviz
│   └── isaac/                        Script Editor files, NOT ROS nodes
│       ├── sensor_rig.py             inspect + place every sensor
│       ├── add_surround_cameras.py   create the four cameras
│       └── lidar_configs/            AUWO_Puck16.json etc.
│
└── excavator_interactive_rviz/   RViz only: markers + the C++ panel
```

Three rules that keep it from rotting:

1. **Nothing platform-specific in `auwo_*`.** Joint names and limits arrive as
   parameters, never hard-coded. The test: could the RoArm reuse this package?
2. **Only the arbiter writes `/arm_position_controller/commands`.** Every other
   source publishes to `/auwo/cmd/*`. This is what makes safety reviewable.
3. **`auwo_bringup` contains no executable nodes.** Launch, config, and Isaac
   scripts only.

### Creating the packages

```bash
cd ~/Desktop/auwo_ws/src
for p in auwo_perception auwo_control auwo_operator auwo_bringup; do
  ros2 pkg create --build-type ament_cmake "$p" >/dev/null
  mkdir -p "$p/scripts"
done
mkdir -p auwo_bringup/{launch,config/sensors,config/rviz,isaac/lidar_configs}
```

Each package's `CMakeLists.txt` needs its scripts installed. Minimal form:

```cmake
find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/auwo_arbiter.py
  scripts/auwo_joy_teleop.py
  scripts/keyboard_to_joy.py
  scripts/isaac_command_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
```

`auwo_bringup` installs directories instead:

```cmake
install(DIRECTORY launch config isaac DESTINATION share/${PROJECT_NAME})
ament_package()
```

### Migrating out of `excavator_interactive_rviz`

Move rather than copy, so there is one copy of each file. `git mv` if the repo
is under version control.

```bash
cd ~/Desktop/auwo_ws/src/excavator_interactive_rviz/scripts
mv lidar_mapper.py dig_zone_monitor.py     ../../auwo_perception/scripts/
mv isaac_command_bridge.py                 ../../auwo_control/scripts/
# joint_imarkers.py and the C++ panel STAY - they are genuinely RViz
```

Then remove the moved entries from that package's `install(PROGRAMS ...)` list
and rebuild everything:

```bash
cd ~/Desktop/auwo_ws
colcon build --symlink-install && source install/setup.bash
```

`map_accumulator.py`, `self_filter.py` and `slew_scan.py` are superseded by
`lidar_mapper.py` and the direct rotation command — delete them once the new
layout works.

---

## 2a. Running it

Everything starts from one launch file. Isaac Sim must be open on the USD named
in the sensor config, with PLAY pressed.

```bash
cd ~/Desktop/auwo_ws && source install/setup.bash
ros2 launch auwo_bringup auwo.launch.py
```

That brings up: robot_state_publisher, the sensor TFs, the Isaac command
bridge, the RViz joint sliders (remapped off the machine topic), the arbiter,
joystick teleop, the terrain map, the dig-region adapter and RViz.

**One node stays separate.** `keyboard_to_joy` reads the keyboard directly, so
it needs its own terminal with focus - launch cannot give a node a TTY:

```bash
ros2 run auwo_control keyboard_to_joy.py
```

With a real gamepad, pass `joy:=true` instead and nothing separate is needed.

### Arguments

| argument | default | effect |
|---|---|---|
| `sensor_config` | `roof_os0_plus_boom` | which `config/sensors/<name>.yaml` |
| `mode` | `teleop` | arbiter start mode: `idle`, `teleop`, `auto` |
| `teleop` | `true` | arbiter + joystick teleop |
| `terrain_map` | `true` | operator map (latest-wins) |
| `survey_map` | `false` | coverage-study map (union-forever) |
| `operator` | `true` | dig-region adapter |
| `joy` | `false` | start `joy_node` for a gamepad |
| `rviz` | `true` | |
| `run_name` | `run` | survey map filename stem |

### Recipes

```bash
# sensor placement study: survey map on, nothing that can command the machine
ros2 launch auwo_bringup auwo.launch.py \
    survey_map:=true terrain_map:=false teleop:=false operator:=false \
    run_name:=roof_plus_boom__level

# watch only - the arbiter will refuse to forward anything
ros2 launch auwo_bringup auwo.launch.py mode:=idle

# a different rig
ros2 launch auwo_bringup auwo.launch.py sensor_config:=boom_only
```

### Driving

WASD is the left stick, IJKL the right, matching the gamepad layout.

| keys | joint | range (rad) |
|---|---|---|
| A / D | slew | -12.566 … 12.566 |
| I / K | boom | -1.400 … 0.200 |
| W / S | stick | -2.428 … 0.200 |
| J / L | bucket | -2.000 … 2.000 |
| B | turbo, double rate | |

Terminal auto-repeat has a long initial delay; `xset r rate 200 40` makes it
feel far better.

### Marking a dig region

1. RViz: add a **MarkerArray** on `/auwo/dig_region_markers`.
2. Pick the **Publish Point** tool.
3. Click the corners. Click near the first point again to close.

```bash
ros2 param set /clicked_point_adapter depth 0.8     # set BEFORE closing
ros2 service call /auwo/clear_region std_srvs/srv/Trigger
ros2 topic echo /auwo/progress
```

`/auwo/progress` is `[total_m3, removed_m3, remaining_m3, percent,
cells_with_target, cells_at_target]`.

### RViz displays worth having

| display | topic | notes |
|---|---|---|
| PointCloud2 | `/auwo/map_cloud` | the operator map |
| MarkerArray | `/auwo/dig_region_markers` | the region being drawn |
| PointCloud2 | `/lidar_*/points` | raw returns, for debugging |
| RobotModel | `/robot_description` | |

On the map cloud, set **Color Transformer → Intensity**, then type a field name
into **Channel Name**: `intensity` for height, `age` for staleness, `remaining`
for dig progress, `quality` for measurement confidence. Uncheck *Autocompute
Intensity Bounds* and set sensible limits (`age` 0-60, `quality` 0-45).

### Sending a pose by hand

The arbiter owns `/arm_position_controller/commands`, so publishing there is
overwritten within 50 ms. Publish to a source topic instead, and keep it fresh -
the arbiter drops anything older than 0.3 s, so `--once` will not do:

```bash
ros2 topic pub -r 20 /auwo/cmd/markers std_msgs/msg/Float64MultiArray \
  "{data: [6.283, -0.9, -2.0, 1.5]}"
```

---

## 3. Data flow

```
  keyboard / joystick ─► /joy ─► auwo_joy_teleop ─┐
                                                  │
  RViz markers ──────────────────────────────────►├─► ARBITER ─► /arm_position_
                                                  │              controller/commands
  (later) task layer ────────────────────────────►┘        │            │
                                                           │      isaac_command_bridge
                          /estop/stopped ──────────────────┘            │
                                                                  /joint_command
                                                                        │
                                                                    ISAAC SIM
                                                                        │
  ┌─────────────────────────────────────────────────────────────────────┤
  │                                                                     │
  ▼                                            ▼                        ▼
/joint_states                          /lidar_*/points            /rgb/cam_*
  │                                            │                        │
robot_state_publisher ─► /tf          auwo_perception ─► /auwo/map      │
  │                                            │                        │
  └────────────────────────────────────────────┴────────────────────────┘
                                    │
                            OPERATOR DASHBOARD
                        (RViz now, Foxglove next)
```

**Command topics** — the contract the arbiter enforces:

| topic | writer | notes |
|---|---|---|
| `/auwo/cmd/teleop` | `auwo_joy_teleop` | mode M1, needs a held deadman |
| `/auwo/cmd/markers` | RViz markers (remapped) | mode M1, accepted in teleop mode |
| `/auwo/cmd/auto` | `excavation_executor` | modes M2 and M3, accepted in auto mode |
| `/arm_position_controller/commands` | **arbiter only** | |
| `/auwo/control_mode` | arbiter | idle / teleop / auto / estop |

Mode M1 uses the arbiter's `teleop` mode; M2 and M3 both use `auto`. The
arbiter cannot tell M2 from M3 and does not need to — it sees one auto source
either way.

---

## 4. Build stages

### Stage 0 — prove the control loop is safe
- [x] `auwo_arbiter.py` — single writer, mode, fail-safe freeze
- [x] `auwo_joy_teleop.py` — deadman-gated velocity teleop
- [x] `keyboard_to_joy.py` — test without a gamepad
- [x] packages created, scripts migrated, everything builds
- [x] markers remapped to `/auwo/cmd/markers` in the launch
- [ ] **deadman test passes** (§6)
- [ ] **comms-loss test passes** (§6)

### Stage 1 — the operator map (serves M1 and M2)
- [x] `terrain_map_node.py` — latest-wins layered map, fixes ghosting
- [ ] staleness shading visible in the client
- [x] region + depth -> target/remaining + progress (folded into `terrain_map_node`, one owner of the grid)
- [x] `clicked_point_adapter.py` — RViz clicks -> region
- [ ] end to end **M1**: draw a region, dig by joystick, watch remaining fall

### Stage 2 — assisted excavation (M2, the primary target)
- [ ] `dig_planner.py` — pick the next cell with material inside the region
- [ ] `excavation_executor.py` — run one dig-and-dump cycle at a pose
- [ ] executor publishes to `/auwo/cmd/auto`; arbiter run in `auto` mode
- [ ] start / pause / abort from the dashboard, abort honoured mid-cycle
- [ ] terminates when every cell in the region reaches target depth
- [ ] end to end **M2**: draw a region, press start, walk away

Start dumb. The first planner can pick the cell with the most remaining
material and run the existing `excavator_cycle` trajectory at it. Refinement
(ALICE's ordering rules, reachability, bucket-fill estimates) never changes an
interface.

### Stage 3 — safety and the real dashboard
- [ ] `safety_monitor.py` — swing-zone watch -> `/auwo/alerts`, trips e-stop
- [ ] `/estop/stopped` publishing; arbiter run with `require_estop:=true`
- [ ] `foxglove_bridge` up, layout built
- [ ] **latency test done** (§6) — decides the video path
- [ ] custom panels: dig region, progress, mode switch, teleop

### Stage 4 — later
- [ ] hardware e-stop beneath the software interlock
- [ ] M3: planner proposes a region, confirm gate, same executor
- [ ] dumper detection and the LOCK protocol
- [ ] electrohydraulics on the real machine, unlocking M2 there

---

## 5. Components

### 5.1 `auwo_arbiter.py` — DONE

Single writer to the machine. Selects a source by control mode, checks
freshness and the e-stop, forwards to `/arm_position_controller/commands`.

**The freeze is the whole point.** This is a position interface, so a stale
target keeps pulling the joints toward it — "stop publishing" would let the
machine carry on. On e-stop, mode change, or source timeout the arbiter latches
the current *measured* pose and publishes that, which actually halts it.

```bash
ros2 run auwo_control auwo_arbiter.py --ros-args \
    -p start_mode:=teleop -p use_sim_time:=true
ros2 param set /auwo_arbiter mode idle      # change mode at runtime
```

Watch for: `require_estop` defaults to **false**, so it will move with no e-stop
present. Correct for sim, wrong for hardware.

### 5.2 `auwo_joy_teleop.py` — DONE

Joy → deadman-gated velocity → integrated position target → `/auwo/cmd/teleop`.
ISO/SAE excavator pattern, so what is learned in sim transfers to the machine.

On deadman release it simply stops publishing; the arbiter's timeout does the
stopping. One failure path covers button release, node crash and network loss.

```bash
ros2 run auwo_control auwo_joy_teleop.py --ros-args -p use_sim_time:=true
#   -p pattern:=sae      the other common control layout
#   -p max_rate:="[0.4,0.35,0.45,0.6]"    rad/s per joint
```

### 5.3 `keyboard_to_joy.py` — DONE

Publishes `sensor_msgs/Joy` from the keyboard, so the teleop node is unchanged
and a real pad drops in later with no edits. WASD is the left stick, IJKL the
right. Deadman = "a movement key is currently repeating".

Run it in a focused terminal. Terminal auto-repeat has a ~0.5 s initial delay,
so the first moment of a press stutters — `xset r rate 200 40` if it grates.

### 5.4 `terrain_map_node.py` — DONE

Latest-wins layered 2.5D map, replacing `lidar_mapper` for the operator view.
`lidar_mapper` accumulates the union of every observation and never forgets, so
a moved bucket leaves a permanent ghost. Correct for the coverage study, wrong
for a dashboard.

Layers: `elevation`, `n_obs`, `age`, `variance`, `quality`, `target`,
`remaining`, `reachable`. Publishes `grid_map_msgs/GridMap` when available plus
a PointCloud2 view carrying the same layers as colourable channels.

**Observations are weighted by grazing angle.** The boom lidar sits 0.40 m up
and the roof lidar 2.87 m, so at 6 m they see the ground at 3.8 deg and 25.6 deg
respectively. A grazing beam has a huge footprint and turns range error into
height error, so returns below `min_grazing_deg` (4 deg) are discarded and the
rest weighted by sin(grazing). The result needs no per-sensor tuning: the boom
lidar owns the near field where nothing else reaches, and the roof lidar wins
beyond about 3 m.

**`exclude_radius` (1.2 m) drops everything near the slew axis.** The URDF link
boxes cover base/body/boom/stick/bucket but not the sensor mast, the camera
housings at 1.85 m, or the lidar bodies - and the roof lidar looks straight down
onto all of them. No usable ground is lost; the nearest real ground return is
about 1.5 m from the axis.

Also owns the dig region: subscribes to `/auwo/dig_region` and `/auwo/dig_depth`,
fills `target` and `remaining`, and publishes `/auwo/progress`. One node owns the
grid, so there is no distributed map to reconcile.

```bash
ros2 run auwo_perception terrain_map_node.py --ros-args \
    -p input_topics:="['/lidar_boom/points','/lidar_roof/points']"
```

Note: `grid_map_msgs` had an ABI mismatch with Jazzy Fast-CDR on this machine
(`undefined symbol: ...fastcdr...`). The node detects its absence and publishes
PointCloud2 only, which loses nothing functionally.

### 5.5 `dig_region_node.py` — NOT NEEDED

Folded into `terrain_map_node`. Splitting the region out would mean two nodes
owning parts of one grid, with a reconciliation problem and no benefit.

### 5.6 `clicked_point_adapter.py` — DONE

Turns RViz `/clicked_point` clicks into `/auwo/dig_region` and
`/auwo/dig_depth`, both **latched** so a client connecting later still gets
them. Draws the outline, corners and an area x depth = volume label as markers.

Clicks outside the 1.86 - 4.36 m work envelope are **warned about but
accepted** - the machine may reposition, and an operator sketching a plan
should not be blocked by the current stance. The `reachable` layer lets the
planner make the real decision.

Exists only because `/clicked_point` is RViz-specific. When the dashboard
arrives it publishes the same two topics and this node is simply not launched.

Limitation: the click transform is translation-only, which is fine while RViz's
fixed frame is `base_link` or an unrotated `world`.

### 5.7 `dig_planner.py` — TODO  *(mode M2)*

Reads the region and the `remaining` layer, emits the next dig pose. Start
dumb: the cell with the most material left, checked for reachability against
the 1.86–4.36 m envelope.

Refine later using ALICE's criteria — prefer poses that fill the bucket, cap
the depth step so material does not slide into finished holes, prefer positions
farther from the machine first since digging drags material inward. None of
that changes an interface.

Writes the same `dig_target` field the operator writes. **This is the node that
becomes M3** when it also proposes the region, not just the pose within one.

### 5.8 `excavation_executor.py` — TODO  *(mode M2)*

Takes a dig pose and runs one dig-and-dump cycle, publishing joint targets to
`/auwo/cmd/auto`. `excavator_cycle` already contains a working trajectory —
start by parameterising it by position rather than writing a new one.

Must be **interruptible at any point**: abort from the dashboard, an alert from
the safety monitor, or an e-stop all have to stop it mid-cycle. Since the
arbiter freezes on source timeout, aborting is simply ceasing to publish.

Terminates the run when every reachable cell in the region reaches target
depth, or when nothing further is reachable — and says which.

### 5.9 `safety_monitor.py` — TODO
### 5.10 Foxglove layout — TODO

---

## 6. Verification log

Record the date, the result, and anything odd. A failed test recorded is worth
more than a passed test assumed.

### T1 — deadman release *(required, Stage 0)*
Hold the deadman, move a joint, release mid-motion. The machine must stop
immediately and not drift.

```bash
ros2 topic echo /arm_position_controller/commands
```
Values must go constant within ~0.3 s of release.

| date | result | notes |
|---|---|---|
| | | |

### T2 — comms loss *(required, Stage 0)*
While moving, `Ctrl-C` the joy source. The arbiter must freeze at the measured
pose. **If the values keep changing, stop and fix this before building
anything else.**

| date | result | notes |
|---|---|---|
| | | |

### T3 — mode isolation
In `idle`, confirm no source moves the machine. Switch to `teleop`, confirm
only teleop and markers do.

| date | result | notes |
|---|---|---|
| | | |

### T4 — glass-to-glass latency *(required before Stage 2)*
Put a millisecond timer in front of a camera, photograph the screen showing
both, subtract. Under ~200 ms is fine; over ~500 ms needs a separate video
path and changes the architecture.

| date | measured | path | notes |
|---|---|---|---|
| | | | |

### T5 — map staleness
Move the bucket through the lidar's view, then away. The ghost must clear once
the cells are re-observed.

| date | result | notes |
|---|---|---|
| | | |

---

## 7. Decisions

Why things are the way they are, so they are not relitigated.

| # | Decision | Reason |
|---|---|---|
| D1 | Arbiter is a standalone node | Safety review is far easier when exactly one node writes the command topic and does nothing else |
| D2 | Freeze latches the *measured* pose | Position interface: a stale target keeps pulling. Not publishing is not stopping |
| D3 | Deadman is a heartbeat | Makes button release, crash and network loss one failure with one handler |
| D4 | Teleop is an Action, not a mode | CACDAR: one Task, several Actions. The autonomy ladder is which Actions are registered, not a restructure |
| D5 | Keyboard publishes `/joy` | The teleop node never learns there was no gamepad |
| D6 | Two maps, not one | Union-forever for the coverage study, latest-wins for the operator. Different questions |
| D7 | ISO control pattern | What real operators expect; the mapping transfers to the machine |
| D8 | `foxglove_bridge`, not `rosbridge` | rosbridge has known trouble with high-rate topics and large messages |
| D9 | M2 and M3 share one execution path | Only the region's *author* differs. M3 becomes one new node writing an existing field, with nothing downstream changed |
| D10 | The M2 planner starts dumb | "Most remaining material, if reachable" is enough to close the loop. Refinement never changes an interface, so it can wait |
| D11 | Abort = stop publishing | The arbiter already freezes on source timeout, so the executor needs no separate stop path. One mechanism, one place to get right |
| D13 | Safety timeouts use `time.monotonic()`, never the ROS clock | If the simulator pauses, sim time stops and a sim-time timeout would never fire. A safety timeout must run on a clock nobody can stop |
| D14 | The teleop target re-seeds only past `resync_rad` | Re-seeding on every brief gap threw away integrated progress, so heavy joints crawled. Long pauses still re-seed, so the machine cannot jump |
| D15 | Region handling lives in `terrain_map_node` | One owner of the grid. A separate node would split ownership of the same data |
| D12 | M1 is built first though M2 is the goal | M1 is the fallback that keeps the machine useful when anything else fails, and it is the only mode the real machine can run before electrohydraulics |

---

## 8. Known issues and gotchas

Hard-won, easy to forget.

- **ROS timers are unreliable under `use_sim_time` here.** They silently failed
  in three separate nodes. Prefer callback-driven publishing or a wall-clock
  loop over `create_timer`.
- **Isaac's `SubscribeJointState` must be on `/joint_command`, not
  `/joint_states`.** Sharing the topic makes the robot re-command itself to its
  current pose and ignore everything you send.
- **`/clock` must be published** by a ROS2 Publish Clock node, or every
  sim-time node stalls waiting for a time source.
- **TF lookups use `Time()` (latest), not the message stamp.** TF runs ~1.5 s
  behind the cloud and exact-stamp lookups drop every scan.
- **USD and YAML sensor poses must agree.** A stale extrinsic silently smears
  the map with no error at all. After moving anything: read back, update YAML,
  relaunch.
- **Lidar tilt sign is `rotateXYZ(0, -TILT, -180)`.** Positive tilt aims the
  sensor backwards over the counterweight.
- **Publishing to `/arm_position_controller/commands` by hand no longer works.**
  The arbiter republishes at 20 Hz and overwrites it within 50 ms. Publish to
  `/auwo/cmd/markers` at `-r 20` instead.
- **Terminal auto-repeat has a ~0.5 s initial delay**, long enough to drop the
  keyboard deadman between the first press and the repeat stream. `HOLD_S` is
  0.45 s to cover it; `xset r rate 200 40` makes it feel much better.
- **`grid_map_msgs` may have an ABI mismatch with Jazzy** (`undefined symbol:
  ...eprosima...fastcdr...`), which kills the node on first publish. Remove the
  package; the terrain map falls back to PointCloud2 with no loss.
- **RViz does not list PointCloud2 channel names.** Set *Color Transformer* to
  *Intensity*, then type the field name into *Channel Name* (`age`, `quality`,
  `remaining`).
- **`ground_coverage_m` and friends are documentation only** — not read by any
  code. A wrong `xyz` or `quat_xyzw` breaks a run silently; a wrong doc field
  does not.