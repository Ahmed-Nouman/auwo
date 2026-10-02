# excavator_v2_description

The new AUWO excavator model ("v2"), generated from `Excavator_rig_packed.blend`.
The original model stays unchanged in `excavator_description` ("v1").

Select it in any AUWO launch file with `excavator_model:=v2` (the default), or set
`export AUWO_EXCAVATOR_MODEL=v2`. See `MODEL_SWITCHING.md` at the workspace root.

    ros2 launch excavator_v2_description view_rviz.launch.py   # sliders + RViz

## Contents

| file | purpose |
|---|---|
| `urdf/excavator.urdf.xacro` | model; same link/joint names and `use_mock_hardware` arg as v1 |
| `urdf/excavator_control.urdf` | actuated joints only (slider GUI / demo pose) |
| `config/controllers.yaml` | `joint_state_broadcaster`, `arm_trajectory_controller`, `blade_controller`, `linkage_controller` |
| `config/model_profile.yaml` | limits, poses, scene layout, physical-twin offsets used by the other packages |
| `meshes/` | OBJ meshes + texture, one per link |

## Joints

| joint | negative | positive | limits [rad] |
|---|---|---|---|
| body_rotation   | clockwise (from above) | counter-clockwise | ±12.566 |
| boom_rotation   | boom up    | boom down  | -0.598 … 1.037 |
| stick_rotation  | stick out  | stick in   | -0.878 … 1.047 |
| bucket_rotation | dump (open)| curl       | -1.944 … 1.290 |
| blade_rotation  | blade up   | blade down | -0.349 … 0.401 |

Same sign convention as v1 (axis `0 1 0`). Zero is the rig rest pose; the ranges differ from v1.
Differences to v1: `base_link` +X is forward (no 180° yaw), OBJ meshes, a blade joint, and 10
linkage joints (4 hydraulic cylinders + bucket rocker/H-link). The linkage joints are computed
by `excavator_models/linkage_state_publisher` and commanded through `linkage_controller`.

Masses (~2.5 t total), efforts and joint limits are estimates derived from the model geometry.
