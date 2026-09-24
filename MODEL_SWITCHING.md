
## Isaac Sim stack (merged from sensor_test)

The Isaac launches take `excavator_model:=` like everything else:

    ros2 launch auwo_bringup isaac_twin.launch.py sensor_config:=front_corners
    ros2 launch auwo_bringup isaac_twin.launch.py sensor_config:=cab_only excavator_model:=v1

They build `robot_description` from the selected model's profile instead of a fixed
xacro path, and for v2 they also start `linkage_state_publisher` in **mirror** mode:
Isaac publishes the four arm joints on `/joint_states`, and the node adds the ten
cylinder/linkage joints so the TF tree is complete and the cylinders move in RViz.

Sensor configurations live in `auwo_bringup/config/sensors/*.yaml`. The older
`excavator_interactive_rviz/launch/isaac_twin.launch.py` still works and now falls back
to those configs. See `AUWO_twin_runbook.md` for the Isaac-side setup and mapping runs.

Note: the Isaac USD scenes in `env/` and `src/excavator_description/urdf/excavator/` were
built from the v1 model. A v2 scene has to be imported from
`excavator_v2_description/urdf/excavator.urdf.xacro` before `excavator_model:=v2` is
useful on the Isaac side.
