"""MoveIt configuration for the selected excavator model (used by excavator_moveit_config launches).

v1: exactly what the launch files did before (MoveItConfigsBuilder defaults from
    excavator_moveit_config/.setup_assistant -> excavator_description xacro,
    config/excavator.srdf, config/joint_limits.yaml).
v2: excavator_v2_description xacro + config/v2/ SRDF and joint limits.
"""
from excavator_models.registry import load_profile, moveit_file, resolve_package_uris

ROBOT_NAME = 'excavator'
MOVEIT_PACKAGE = 'excavator_moveit_config'


def moveit_configs(model=None, use_mock_hardware=False, pipelines=('ompl',), load_all=None):
    """Return (MoveItConfigs, profile) for the model."""
    from moveit_configs_utils import MoveItConfigsBuilder

    profile = load_profile(model)
    builder = MoveItConfigsBuilder(ROBOT_NAME, package_name=MOVEIT_PACKAGE)
    pipeline_kwargs = {'pipelines': list(pipelines)}
    if load_all is not None:
        pipeline_kwargs['load_all'] = load_all
    builder.planning_pipelines(**pipeline_kwargs)

    if profile['model'] == 'v1':
        if use_mock_hardware:
            builder.robot_description(mappings={'use_mock_hardware': 'true'})
    else:
        builder.robot_description(
            file_path=profile['xacro_path'],
            mappings={
                'use_mock_hardware': 'true' if use_mock_hardware else 'false',
                'use_sim': 'true',
            },
        )
        builder.robot_description_semantic(file_path=moveit_file(profile, 'srdf'))
        builder.joint_limits(file_path=moveit_file(profile, 'joint_limits'))
        builder.trajectory_execution(file_path=moveit_file(profile, 'controllers'))

    config = builder.to_moveit_configs()
    if profile.get('resolve_mesh_uris'):
        for key, value in list(config.robot_description.items()):
            if isinstance(value, str):
                config.robot_description[key] = resolve_package_uris(value, profile, scheme='file')
    return config, profile
