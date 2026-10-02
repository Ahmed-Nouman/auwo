"""Model registry: which description package, xacro, controllers and poses belong to a model.

Each description package ships ``config/model_profile.yaml``. This module finds the
package through the ament index, loads the profile and adds absolute paths, so
launch files and nodes never hard-code a model.
"""
import copy
import os
import re

import yaml
from ament_index_python.packages import get_package_share_directory

ENV_VAR = 'AUWO_EXCAVATOR_MODEL'
DEFAULT_MODEL = 'v2'

# model key -> description package
MODELS = {
    'v1': 'excavator_description',
    'v2': 'excavator_v2_description',
}

ALIASES = {
    'old': 'v1', 'legacy': 'v1', 'original': 'v1', 'excavator_description': 'v1',
    'new': 'v2', 'excavator_v2_description': 'v2',
}

MOVEIT_PACKAGE = 'excavator_moveit_config'

_cache = {}


def default_model():
    """Model used when no excavator_model argument is given."""
    return normalize_model(os.environ.get(ENV_VAR, '') or DEFAULT_MODEL)


def normalize_model(name):
    """'v1'/'old'/... -> 'v1'. Empty -> default. Unknown -> ValueError."""
    key = (name or '').strip().lower()
    if not key:
        return default_model()
    key = ALIASES.get(key, key)
    if key not in MODELS:
        raise ValueError(
            f'Unknown excavator model "{name}". Use one of: '
            f'{", ".join(sorted(MODELS))} (aliases: {", ".join(sorted(ALIASES))})')
    return key


def load_profile(name=None):
    """Return the profile dict of a model, with absolute paths added.

    Added keys: model, package, share, xacro_path, controllers_path,
    control_urdf_path (or None).
    """
    key = normalize_model(name)
    if key in _cache:
        return copy.deepcopy(_cache[key])
    package = MODELS[key]
    share = get_package_share_directory(package)
    with open(os.path.join(share, 'config', 'model_profile.yaml')) as f:
        prof = yaml.safe_load(f) or {}
    prof['model'] = key
    prof['package'] = package
    prof['share'] = share
    prof['xacro_path'] = os.path.join(share, prof['xacro'])
    prof['controllers_path'] = os.path.join(share, prof['controllers'])
    cu = prof.get('control_urdf')
    prof['control_urdf_path'] = os.path.join(share, cu) if cu else None
    prof.setdefault('extra_controllers', [])
    prof.setdefault('linkage', {'enabled': False})
    prof.setdefault('sim_patches', {})
    prof.setdefault('resolve_mesh_uris', False)
    _cache[key] = prof
    return copy.deepcopy(prof)


def arm_limits(profile):
    """(lower, upper) lists for [body, boom, stick, bucket] as used by the UI nodes."""
    lim = profile['ui_limits']
    joints = profile['arm_joints']
    lower = [float(lim[j][0]) for j in joints]
    upper = [float(lim[j][1]) for j in joints]
    return lower, upper


def process_xacro(profile, mappings=None):
    """Expand the model's xacro and return the URDF XML string."""
    import xacro  # imported lazily so the registry works without xacro for simple queries
    maps = {k: str(v) for k, v in (mappings or {}).items()}
    doc = xacro.process_file(profile['xacro_path'], mappings=maps)
    return doc.toprettyxml(indent='  ')


def resolve_package_uris(xml, profile, scheme='file', force=False):
    """Replace package://<description package>/ with an absolute path.

    scheme='file' -> file:///abs/share/...  (RViz / MoveIt RViz)
    scheme='path' -> /abs/share/...         (Gazebo resolved URDF)
    Only applied when the profile asks for it (resolve_mesh_uris) or force=True.
    """
    if not (force or profile.get('resolve_mesh_uris', False)):
        return xml
    prefix = 'file://' if scheme == 'file' else ''
    return xml.replace(f'package://{profile["package"]}/',
                       prefix + profile['share'].rstrip('/') + '/')


def _zero_joint_rpy(xml, joint_name):
    marker = f'name="{joint_name}"'
    if marker not in xml:
        return xml
    start = xml.index(marker)
    end = xml.index('</joint>', start) + len('</joint>')
    block = re.sub(r'rpy="[^"]*"', 'rpy="0 0 0"', xml[start:end])
    return xml[:start] + block + xml[end:]


def _zero_link_visual_rpy(xml, link_name):
    marker = f'<link name="{link_name}">'
    if marker not in xml:
        return xml
    start = xml.index(marker)
    end = xml.index('</link>', start) + len('</link>')
    parts = xml[start:end].split('<visual>')
    for i in range(1, len(parts)):
        vend = parts[i].index('</visual>')
        parts[i] = re.sub(r'rpy="[^"]*"', 'rpy="0 0 0"', parts[i][:vend]) + parts[i][vend:]
    return xml[:start] + '<visual>'.join(parts) + xml[end:]


def apply_sim_patches(xml, profile):
    """Simulation-only URDF fixes that a model declares in its profile.

    v1: zero the 180 deg yaw on base_to_base_link and the bucket visual flip
        (identical to the patches previously hard-coded in gazebo.launch.py).
    v2: nothing to patch.
    """
    patches = profile.get('sim_patches', {}) or {}
    if patches.get('zero_base_yaw'):
        xml = _zero_joint_rpy(xml, 'base_to_base_link')
    if patches.get('zero_bucket_visual_rpy'):
        xml = _zero_link_visual_rpy(xml, 'bucket')
    return xml


def moveit_file(profile, key):
    """Absolute path of a MoveIt config file (srdf, joint_limits, controllers) for a model."""
    share = get_package_share_directory(MOVEIT_PACKAGE)
    return os.path.join(share, profile['moveit'][key])


def describe(name=None):
    p = load_profile(name)
    lines = [
        f'model            : {p["model"]}  ({p.get("label", "")})',
        f'package          : {p["package"]}',
        f'xacro            : {p["xacro_path"]}',
        f'controllers      : {p["controllers_path"]}',
        f'extra controllers: {", ".join(p["extra_controllers"]) or "-"}',
        f'linkage node     : {"yes" if p["linkage"].get("enabled") else "no"}',
        f'sim patches      : {", ".join(k for k, v in p["sim_patches"].items() if v) or "-"}',
    ]
    return '\n'.join(lines)
