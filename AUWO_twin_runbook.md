# AUWO Digital Twin — Isaac Sim Runbook

Quick reference for running the excavator digital twin in Isaac Sim 5.1 with
ROS 2 Jazzy. Covers the two lidar configurations built so far, how the pieces
fit together, and the exact commands.

---

## 1. What this is

The AUWO project's excavator digital twin, migrated from Gazebo to Isaac Sim.
Isaac provides physics and sensors; ROS 2 provides control, TF, visualisation
and mapping. The original `auwo_ws` ROS packages are reused unchanged — Isaac
simply replaces Gazebo behind the same topic interface.

**Current capability:** joint control, RTX lidar simulation, self-filtered
point clouds, and a workspace map built by rotating the cab.

**Current goal:** decide where to mount lidars on the real machine by comparing
coverage between configurations in simulation.

---

## 2. How the pieces fit together

```
                    ┌─────────────────────────────────────────┐
                    │            ISAAC SIM 5.1                │
                    │                                         │
   /joint_command ─►│  SubscribeJointState                    │
                    │       └► ArticulationController         │
                    │            └► excavator joints          │
                    │                                         │
                    │  PublisherJointState  ──► /joint_states  │
                    │  Publish Clock        ──► /clock         │
                    │  RTX Lidar Helper(s)  ──► /lidar_*/points│
                    └─────────────────────────────────────────┘
                              ▲                    │
                              │                    ▼
  RViz markers ──┐            │            robot_state_publisher
  keyboard teleop├─► /arm_position_controller/commands   │
  cycle / CLI  ──┘            │                    ├──► /tf  (base_link →
                              │                    │         body → boom →
                    isaac_command_bridge           │         stick → bucket)
                    (Float64MultiArray             │
                       → JointState)               │
                                                   ▼
                                          static_transform_publisher
                                          (parent → lidar_* mount)
                                                   │
                                                   ▼
                                            lidar_mapper
                                     range crop → link-box self-filter
                                     → TF into base_link → voxel grid
                                              │         │
                                        /map_cloud   <run_name>.pcd
```

**Key design points**

- **Command path:** everything publishes `Float64MultiArray` to
  `/arm_position_controller/commands` in the order
  `[body_rotation, boom_rotation, stick_rotation, bucket_rotation]` (radians).
  `isaac_command_bridge` converts it to a `JointState` on `/joint_command`.
  One interface for RViz markers, keyboard teleop, the dig cycle and the CLI.

- **State path:** Isaac publishes `/joint_states`; `robot_state_publisher`
  turns it into the TF tree. This is the same contract the *physical* machine
  satisfies via `physical_tf_follower_node`, so the twin and the real excavator
  are interchangeable downstream.

- **Mapping is NOT SLAM.** The cab rotates about a known joint, so every scan
  is placed using the joint encoder through TF. No scan matching, no IMU, no
  drift. This is mapping-with-known-poses.

- **Sensor mount poses live in two places** — the USD prim and the YAML config.
  They must agree. See §7.

---

## 3. One-time Isaac setup (per USD scene)

Open the scene, then verify:

| Item | Required value |
|---|---|
| `SubscribeJointState` → `topicName` | `/joint_command` (NOT `/joint_states`) |
| `ROS2 Publish Clock` node | present, publishing `/clock` |
| `ArticulationController` → `targetPrim` | `/World/excavator` |
| Each `RTX Lidar Helper` → `type` | `point_cloud` |
| Each `RTX Lidar Helper` → `topicName` / `frameId` | must match the YAML config |
| Lidar profile | `AUWO_Puck16` (16 beams, ±15°, 10 Hz) |

Then press **PLAY**. Nothing works while the sim is stopped.

> If publisher and subscriber share `/joint_states`, the robot continuously
> re-commands itself to its current pose and ignores you. This was the single
> most confusing bug in bring-up.

---

## 4. Configuration A — `cab_only` (baseline)

One 16-beam lidar, centre of the cab mast, 30° forward tilt.

- USD scene: `excavator_cab_only.usd`
- Sensor at 2.43 m above ground → beams 15°–45° depression
- **Ground coverage: 2.4 m – 9.1 m**
- Limitation: the arm sits mid-scan, so the boom shadows the dig zone

```bash
ros2 launch excavator_interactive_rviz isaac_twin.launch.py \
  sensor_config:=cab_only run_name:=cab_only__tilt30
```

---

## 5. Configuration B — `front_corners`

Two 16-beam lidars at the front corners, on the existing camera mounts.

- USD scene: `excavator_front_corners.usd`
- Both parented to `sensor_imu_link`, at ±0.394 m lateral (0.79 m baseline)
- Both at 2.355 m above ground, 30° tilt → **2.36 m – 8.79 m**
- Rationale: lateral separation lets each sensor see *around* the boom, so one
  fills the other's arm shadow (parallax, not just redundancy). Matches the
  ALICE front-pair configuration (Frese et al. 2022).

```bash
ros2 launch excavator_interactive_rviz isaac_twin.launch.py \
  sensor_config:=front_corners run_name:=front_corners__tilt30
```

`lidar_mapper` subscribes to **both** topics and merges them in `base_link` —
`/map_cloud` and the saved PCD are the merged result.

---

## 6. Running a mapping session

```bash
# 1. Isaac Sim: open the USD for the config, press PLAY

# 2. Terminal 1 — full stack (RSP, lidar TFs, bridge, markers, mapper, RViz)
cd ~/Desktop/auwo_ws && source install/setup.bash
ros2 launch excavator_interactive_rviz isaac_twin.launch.py \
  sensor_config:=front_corners run_name:=front_corners__tilt30

# 3. RViz: add PointCloud2 on /map_cloud
#    Fixed Frame base_link, Size 0.05, Color Transformer AxisColor (Z)

# 4. Terminal 2 — rotate the cab a full turn
ros2 topic pub --once /arm_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [6.283, -0.9, -2.0, 1.5]}"

# 5. When rotation stops: Ctrl+C the launch → map saves automatically to
#    ~/Desktop/AUWO-testbed/maps/<run_name>.pcd
```

Rotation speed is set in Isaac, not ROS — see §9.

### Per-sensor maps (to measure the merge benefit)

Run with `mapping:=false`, then one mapper per lidar:

```bash
ros2 run excavator_interactive_rviz lidar_mapper.py --ros-args \
  -p input_topics:="['/lidar_fl/points']" -p use_sim_time:=true \
  -p save_path:=$HOME/Desktop/AUWO-testbed/maps/front_corners__fl_only.pcd

ros2 run excavator_interactive_rviz lidar_mapper.py --ros-args \
  -p input_topics:="['/lidar_fr/points']" -p use_sim_time:=true \
  -p save_path:=$HOME/Desktop/AUWO-testbed/maps/front_corners__fr_only.pcd
```

**Complementarity ratio = merged / (FL + FR)** at the same voxel size.
Near 1.0 → the sensors see different regions (complementary).
Near 0.5 → heavy overlap (redundant).

---

## 7. Changing a sensor mount — the critical procedure

The mount pose exists in the USD **and** in the YAML. If they disagree, points
are rotated by a stale extrinsic and the map smears — with no error message.

```
1. Move / re-tilt the prim in Isaac
2. Read the resulting pose (script below)
3. Paste the values into config/sensors/<config>.yaml
4. Save the USD, relaunch
```

**Set tilt** (Script Editor). Note the **minus** on the tilt — positive Y points
the sensor backwards over the counterweight:

```python
import omni.usd
from pxr import UsdGeom, Gf

TILT = 30.0
P = "/World/excavator/sensor_lidar_link/lidar_cab"

stage = omni.usd.get_context().get_stage()
x = UsdGeom.Xformable(stage.GetPrimAtPath(P))
ops = {op.GetOpName(): op for op in x.GetOrderedXformOps()}
t = ops["xformOp:translate"].Get()
x.ClearXformOpOrder()
x.AddTranslateOp().Set(t)
x.AddRotateXYZOp().Set(Gf.Vec3f(0.0, -TILT, -180.0))
x.AddScaleOp().Set(Gf.Vec3f(1, 1, 1))
```

**Read the pose back** for the YAML:

```python
import omni.usd
from pxr import UsdGeom

stage = omni.usd.get_context().get_stage()
PARENT = "/World/excavator/sensor_lidar_link"
SENSOR = PARENT + "/lidar_cab"

xc = UsdGeom.XformCache()
rel = (xc.GetLocalToWorldTransform(stage.GetPrimAtPath(SENSOR)) *
       xc.GetLocalToWorldTransform(stage.GetPrimAtPath(PARENT)).GetInverse())
t = rel.ExtractTranslation()
q = rel.ExtractRotationQuat(); i = q.GetImaginary()
z = rel.ExtractRotationMatrix().GetRow(2)

print("    xyz: [%.6f, %.6f, %.6f]" % (t[0], t[1], t[2]))
print("    quat_xyzw: [%.6f, %.6f, %.6f, %.6f]"
      % (i[0], i[1], i[2], q.GetReal()))
print("    # spin axis %s -> tilt toward %s X"
      % (z, "+" if z[0] > 0 else "-"))
```

The spin-axis line must say **toward + X** (forward, over the work area).

> USD quaternions are `(w, x, y, z)`; ROS TF wants `(x, y, z, w)`.
> The YAML field is named `quat_xyzw` to make this explicit.

### Tilt vs coverage (sensor at 2.4 m, ±15° FOV)

| tilt | beam depression | ground coverage |
|---|---|---|
| 45° | 30°–60° | 1.4 – 4.2 m ← too near |
| 35° | 20°–50° | 2.0 – 6.7 m |
| **30°** | 15°–45° | **2.4 – 9.1 m** |
| 25° | 10°–40° | 2.9 – 13.8 m |

`ground_range = height / tan(depression)`. With only 30° of vertical FOV you
cannot cover near and far at once — this trade is why ALICE used four lidars
with different roles.

---

## 8. Adding a new configuration

1. Save a new USD scene; create and position the lidar prims (§7).
2. Add a ROS graph per lidar: `Create Render Product` → `RTX Lidar Helper`,
   with `cameraPrim` set to the lidar prim, `type=point_cloud`, and a unique
   `topicName` / `frameId`.
3. Copy an existing YAML in `config/sensors/`, update `name`, `usd_scene`, and
   the `lidars:` list (frame, parent, topic, xyz, quat_xyzw).
4. Launch with `sensor_config:=<new_name>`.

No launch-file or code changes are needed.

**Planned next:** `cab_plus_boom` — cab lidar plus one mounted low, forward of
and below the boom pivot, parented to `body`. This is ALICE's "one below the
boom" idea: with the boom *above* the sensor it no longer shadows the ground.

---

## 9. Other controls

**Keyboard teleop** (velocity feel on the position interface):

```bash
ros2 run excavator_teleop teleop_excavator.py
# a/d body · w/s boom · i/k stick · j/l bucket · SPACE safe pose · q quit
```

**RViz interactive markers** — Add → InteractiveMarkers,
update topic `/excavator_joint_server/update`. Drag *along* the arrow axis.

**Direct command:**

```bash
ros2 topic pub --once /arm_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.5, -0.6, -1.0, 0.3]}"
```

**Slew speed** (Isaac Script Editor) — degrees per second:

```python
import omni.usd
from pxr import PhysxSchema

stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("/World/excavator/joints/body_rotation")
PhysxSchema.PhysxJointAPI.Apply(prim).CreateMaxJointVelocityAttr().Set(10.0)
```

Slower rotation → more scans per degree → denser map.

### Joint limits (from the USD)

| joint | axis | limits (rad) |
|---|---|---|
| `body_rotation` | Z | −12.566 … 12.566 |
| `boom_rotation` | Y | −1.400 … 0.200 |
| `stick_rotation` | Y | −2.428 … 0.200 |
| `bucket_rotation` | Y | −2.000 … 2.000 |

Keep the arm clear of the ground during a slew — a bucket dug into the terrain
will stall the rotation.

---

## 10. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Machine won't move | `SubscribeJointState` on `/joint_states` | set it to `/joint_command` |
| `/joint_command` has 0 subscribers | subscriber node missing from graph | Tools → Robotics → ROS 2 OmniGraphs → Articulation Controller |
| RViz model frozen, `/joint_states` fine | `/clock` not published | add `ROS2 Publish Clock` to the graph |
| `Lookup would require extrapolation into the future` | TF lags the cloud | mapper uses `Time()` (latest) — already applied |
| Topic listed but `hz` shows nothing | ROS timer not firing under sim time | publish from the callback, not `create_timer` |
| Points appear in impossible directions | YAML pose ≠ USD pose | redo §7 |
| Two rings of points in the map | self-hits on cab and arm | raise `min_range` / `inflate` |
| Isaac crashes on Play | lidar config emitter arrays ≠ `numberOfEmitters` | rebuild the JSON profile |
| Rotation stalls partway | bucket contacting terrain | raise the arm before slewing |

**ROS timers are unreliable under `use_sim_time` in this setup** — they broke
three separate nodes during bring-up. Prefer callback-driven publishing or
wall-clock loops.

---

## 11. File map

```
excavator_interactive_rviz/
├── launch/
│   └── isaac_twin.launch.py         config-driven launch (one for all setups)
├── config/sensors/
│   ├── cab_only.yaml
│   ├── front_corners.yaml
│   └── cab_plus_boom.yaml           (planned)
└── scripts/
    ├── isaac_command_bridge.py      Float64MultiArray → JointState
    ├── lidar_mapper.py              self-filter + accumulate + save
    └── joint_imarkers.py            RViz sliders (existing)

excavator_teleop/scripts/
└── teleop_excavator.py              keyboard control (existing)

~/Desktop/AUWO-testbed/
├── sim_assets/AUWO_Puck16.json      16-beam lidar profile
└── maps/<run_name>.pcd              saved workspace maps
```

Lidar profile also installed at
`~/isaacsim/source/extensions/isaacsim.sensors.rtx/data/lidar_configs/AUWO/Puck16/`

---

## 12. Run log

Record every run so configurations stay comparable.

| run_name | config | tilt | slew | voxel | points | notes |
|---|---|---|---|---|---|---|
| `cab_only__tilt45` | cab_only | 45° | — | 0.10 | | far region not covered |
| `cab_only__tilt30` | cab_only | 30° | | 0.10 | | baseline |
| `front_corners__tilt30` | front_corners | 30° | | 0.10 | | |

Also worth recording: arm pose during the slew, `min_range`, `inflate`, and
the `kept X% of returns` figure the mapper logs.
