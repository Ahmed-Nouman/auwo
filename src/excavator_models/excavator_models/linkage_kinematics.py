"""Kinematics of the v2 excavator hydraulic cylinders and bucket four-bar linkage.

Generated from Excavator_rig_packed.blend (rig rest pose = all joints zero).
No ROS dependency, so it can also be used in Isaac Sim, tests or offline tools.

Joint sign convention (same as the previous excavator_urdf.xacro, axis 0 1 0):
    boom_rotation    negative = boom up,    positive = boom down
    stick_rotation   negative = stick out,  positive = stick in
    bucket_rotation  negative = dump/open,  positive = curl/close
    blade_rotation   negative = blade up,   positive = blade down
Internally the solver works with counter-clockwise angles in the X-Z plane
(X forward, Z up), i.e. the negated joint values.
"""
import math

# Pin positions (x, z) [m], machine frame at rest
PIN = {
    'swing': (-0.057418, 0.505486),
    'boom': (0.598947, 0.8292),
    'stick': (2.501762, 1.627077),
    'bucket': (3.094816, 0.483256),
    'bucket_E': (3.14382, 0.390104),
    'rocker': (3.035549, 0.614827),
    'hlink': (3.250051, 0.556168),
    'boom_cyl_barrel': (0.803491, 0.629663),
    'boom_cyl_rod': (1.49746, 1.490968),
    'stick_cyl_barrel': (1.503804, 1.844038),
    'stick_cyl_rod': (2.540497, 1.912884),
    'bucket_cyl_barrel': (2.832452, 1.550923),
    'bucket_cyl_rod': (3.250051, 0.556168),
    'blade': (0.38843, 0.238619),
    'blade_cyl_barrel': (0.426574, 0.358725),
    'blade_cyl_rod': (0.885368, 0.183309),
}

# Cylinder pin-to-pin lengths [m]: rest, fully retracted, fully extended (estimated from meshes)
CYL = {
    'boom': dict(L0=1.106091, Lmin=0.8090, Lmax=1.2660),
    'stick': dict(L0=1.038977, Lmin=0.7990, Lmax=1.3500),
    'bucket': dict(L0=1.078854, Lmin=0.7420, Lmax=1.2710),
    'blade': dict(L0=0.491185, Lmin=0.3150, Lmax=0.6010),
}

# Actuated joint limits [rad], identical to excavator_v2_description/urdf/excavator.urdf.xacro
LIMITS = {
    'boom_rotation': (-0.598, 1.037),
    'stick_rotation': (-0.878, 1.047),
    'bucket_rotation': (-1.944, 1.290),
    'blade_rotation': (-0.349, 0.401),
}

ACTUATED = ['body_rotation', 'boom_rotation', 'stick_rotation', 'bucket_rotation', 'blade_rotation']

# Order used by linkage_controller (config/controllers.yaml)
LINKAGE_JOINTS = [
    'boom_cyl_barrel_joint', 'boom_cyl_rod_joint',
    'stick_cyl_barrel_joint', 'stick_cyl_rod_joint',
    'bucket_rocker_joint', 'bucket_hlink_joint',
    'bucket_cyl_barrel_joint', 'bucket_cyl_rod_joint',
    'blade_cyl_barrel_joint', 'blade_cyl_rod_joint',
]


def clamp(name, value):
    lo, hi = LIMITS.get(name, (-math.inf, math.inf))
    return min(max(value, lo), hi)


def _rot(p, c, a):
    dx, dz = p[0] - c[0], p[1] - c[1]
    ca, sa = math.cos(a), math.sin(a)
    return (c[0] + dx * ca - dz * sa, c[1] + dx * sa + dz * ca)


def _ang(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def _dist(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _cylinder(name, barrel, rod_rest, rod_now):
    beta = _wrap(_ang(barrel, rod_now) - _ang(barrel, rod_rest))
    length = _dist(barrel, rod_now)
    c = CYL[name]
    ok = c['Lmin'] - 1e-6 <= length <= c['Lmax'] + 1e-6
    return beta, length - c['L0'], length, ok


def _circle_intersect(c0, r0, c1, r1, ref_sign):
    d = _dist(c0, c1)
    if d > r0 + r1 or d < abs(r0 - r1) or d == 0:
        return None
    a = (r0 * r0 - r1 * r1 + d * d) / (2 * d)
    h = math.sqrt(max(r0 * r0 - a * a, 0.0))
    ex, ez = (c1[0] - c0[0]) / d, (c1[1] - c0[1]) / d
    mx, mz = c0[0] + a * ex, c0[1] + a * ez
    for s in (1, -1):
        p = (mx - s * h * ez, mz + s * h * ex)
        cross = (c1[0] - c0[0]) * (p[1] - c0[1]) - (c1[1] - c0[1]) * (p[0] - c0[0])
        if (cross >= 0) == (ref_sign >= 0):
            return p
    return None


def _solve_ccw(boom, stick, bucket, blade):
    """Counter-clockwise angles in, counter-clockwise passive angles out."""
    J, L, valid = {}, {}, True

    q_rod = _rot(PIN['boom_cyl_rod'], PIN['boom'], boom)
    b, q, L['boom'], ok = _cylinder('boom', PIN['boom_cyl_barrel'], PIN['boom_cyl_rod'], q_rod)
    J['boom_cyl_barrel_joint'], J['boom_cyl_rod_joint'] = b, q
    valid &= ok

    q_rod = _rot(PIN['stick_cyl_rod'], PIN['stick'], stick)
    b, q, L['stick'], ok = _cylinder('stick', PIN['stick_cyl_barrel'], PIN['stick_cyl_rod'], q_rod)
    J['stick_cyl_barrel_joint'], J['stick_cyl_rod_joint'] = b, q
    valid &= ok

    A, E, C, D = PIN['bucket'], PIN['bucket_E'], PIN['rocker'], PIN['hlink']
    E1 = _rot(E, A, bucket)
    ref = (E[0] - C[0]) * (D[1] - C[1]) - (E[1] - C[1]) * (D[0] - C[0])
    D1 = _circle_intersect(C, _dist(C, D), E1, _dist(D, E), ref)
    if D1 is None:
        return J, L, False
    rocker = _wrap(_ang(C, D1) - _ang(C, D))
    J['bucket_rocker_joint'] = rocker
    J['bucket_hlink_joint'] = _wrap(_ang(D1, E1) - _ang(D, E) - rocker)
    b, q, L['bucket'], ok = _cylinder('bucket', PIN['bucket_cyl_barrel'], PIN['bucket_cyl_rod'], D1)
    J['bucket_cyl_barrel_joint'], J['bucket_cyl_rod_joint'] = b, q
    valid &= ok

    q_rod = _rot(PIN['blade_cyl_rod'], PIN['blade'], blade)
    b, q, L['blade'], ok = _cylinder('blade', PIN['blade_cyl_barrel'], PIN['blade_cyl_rod'], q_rod)
    J['blade_cyl_barrel_joint'], J['blade_cyl_rod_joint'] = b, q
    valid &= ok
    return J, L, valid


def solve(boom_rotation=0.0, stick_rotation=0.0, bucket_rotation=0.0, blade_rotation=0.0):
    """Joint values (URDF convention) in.

    Returns (passive joint positions in URDF convention, cylinder lengths [m], valid).
    Revolute passive joints use axis 0 1 0 like the actuated joints; prismatic
    *_cyl_rod_joint values are the extension relative to the rest length.
    """
    J, L, ok = _solve_ccw(-boom_rotation, -stick_rotation, -bucket_rotation, -blade_rotation)
    out = {k: (v if k.endswith('_rod_joint') else -v) for k, v in J.items()}
    return out, L, ok
