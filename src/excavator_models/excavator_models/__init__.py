"""Excavator model selection for the AUWO workspace.

Two excavator descriptions live side by side:

    v1  excavator_description      original model (DAE meshes), unchanged
    v2  excavator_v2_description   new model from Excavator_rig_packed.blend

Every launch file takes ``excavator_model:=v1|v2``. When the argument is not
given, the ``AUWO_EXCAVATOR_MODEL`` environment variable is used, and when that
is not set either, ``v2``.
"""
from excavator_models.registry import (  # noqa: F401
    DEFAULT_MODEL,
    ENV_VAR,
    MODELS,
    apply_sim_patches,
    arm_limits,
    default_model,
    describe,
    load_profile,
    moveit_file,
    normalize_model,
    process_xacro,
    resolve_package_uris,
)
