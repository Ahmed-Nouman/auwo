"""
sync_sensor_tf.py — read the lidar poses out of the Isaac stage and write them
into the sensor YAML, so the static TF matches where the sensors actually are.

Paste into Isaac Sim's Script Editor (cleared) with the excavator scene open
and run. By default it only REPORTS; set WRITE = True to update the YAML.

WHY THIS EXISTS
auwo.launch.py builds `body -> lidar_boom` from config/sensors/<name>.yaml and
nothing else on the ROS side knows where the sensor is - Isaac publishes no
transform for it. So the stage and that file have to agree, and until now
nothing checked that they did. They silently disagreed for a whole evening:
the YAML still held v1's puck pose, complete with a 180 degree flip that only
made sense on a machine whose arm pointed the other way, and the point cloud
landed half a turn from the truth.

Moving a sensor in the viewport and rerunning this closes that gap.

WHAT IT COMPUTES
Not the prim's local transform - that is relative to whatever prim happens to
sit above it, which may be a wrapper (the menu builds Xform/Looks/mesh/sensor)
or several nested Xforms. TF needs the pose relative to a URDF LINK, because
links are the only prims that exist as frames on the ROS side.

So it walks up from each sensor to the nearest ancestor whose name is a link
in the URDF, and computes

    relative = sensor_world * inverse(link_world)

which is correct through any depth of nesting.

LIDARS ONLY, DELIBERATELY
A USD camera looks down -Z with +Y up; a ROS optical frame has +Z forward and
-Y up. Emitting camera poses without that conversion would write plausible
numbers that are wrong by a fixed rotation - the same kind of silent error this
script exists to prevent. Cameras are listed but not written. An RTX lidar
needs no such conversion: +X forward, +Z up, same as ROS.
"""

import math
import os
import re

import omni.usd
from pxr import Gf, UsdGeom

# ----------------------------------------------------------------- settings
YAML_PATH = os.path.expanduser(
    "~/Desktop/auwo_ws/src/auwo_bringup/config/sensors/v2_airy_boom.yaml")

WRITE = True          # True actually edits the YAML; False just reports

# Prim name -> ROS frame id, for a sensor whose frameId on the ROS2 RTX Lidar
# Helper node differs from the prim that produces the cloud.
#
# EMPTY ON PURPOSE, and it should stay that way. Every lidar's prim name and
# frameId are the same, so the prim name IS the frame name and there is
# nothing to map.
#
# This existed briefly because the Avia's prim was `lidar_avia` while its
# frameId was `avia_lidar`. Both frames would exist, TF would resolve, nothing
# would error - and a cloud stamped with one name while the transform carried
# the other simply could not be placed. Renaming the frameId to match the prim
# removed the problem rather than translating it, which is why this is empty.
#
# Only add an entry for a sensor whose frameId genuinely cannot be changed.
FRAME_OVERRIDES = {}

# Prim names that are URDF links, i.e. that exist as TF frames. A sensor's
# pose is reported relative to the nearest of these above it.
LINKS = {"base_link", "base", "body", "boom", "stick", "bucket", "blade",
         "sensor_imu_link", "sensor_lidar_link"}

# ------------------------------------------------------------------ helpers
stage = omni.usd.get_context().get_stage()
MPU = UsdGeom.GetStageMetersPerUnit(stage) or 1.0
cache = UsdGeom.XformCache()


# OmniLidar prims the menu creates are all called "sensor" - the useful name
# is on the wrapper above them. Anything in here means "look one level up".
GENERIC = {"sensor", "lidar", "Lidar", "OmniLidar"}


def frame_name(prim):
    """The TF frame this sensor should publish as.

    The menu builds   lidar_boom/          Xform   <- the name you chose
                        Looks              Scope
                        os0_mesh           Mesh
                        sensor             OmniLidar
    so the OmniLidar itself is called "sensor" on EVERY lidar in the scene.
    Using that as the frame name gives two frames called "sensor" and matches
    nothing in the YAML, which is exactly what happened.
    """
    name = prim.GetName()
    if name in GENERIC:
        parent = prim.GetParent()
        if parent and str(parent.GetPath()) != "/":
            name = parent.GetName()
    # FRAME_OVERRIDES has the last word: the prim name is what the stage calls
    # this sensor, but the frame is whatever frameId the graph stamps clouds
    # with, and only that name is useful to TF.
    return FRAME_OVERRIDES.get(name, name)


def nearest_link(prim):
    """The closest ancestor that is a URDF link, i.e. a real TF frame."""
    p = prim.GetParent()
    while p and str(p.GetPath()) != "/":
        if p.GetName() in LINKS:
            return p
        p = p.GetParent()
    return None


def pose_relative_to(prim, link):
    """(xyz_m, quat_xyzw) of `prim` expressed in `link`'s frame."""
    w_s = cache.GetLocalToWorldTransform(prim)
    w_l = cache.GetLocalToWorldTransform(link)
    rel = w_s * w_l.GetInverse()

    t = rel.ExtractTranslation()
    xyz = (t[0] * MPU, t[1] * MPU, t[2] * MPU)

    # scale/shear would corrupt the extracted rotation
    q = rel.RemoveScaleShear().ExtractRotationQuat().GetNormalized()
    im = q.GetImaginary()
    quat = (im[0], im[1], im[2], q.GetReal())
    if quat[3] < 0:                       # keep w positive, same rotation
        quat = tuple(-v for v in quat)
    return xyz, quat


def block_of(text, frame):
    """The text of one '- frame: <name>' block, or None."""
    start = re.search(r'^(\s*)-\s+frame:\s*%s\s*$' % re.escape(frame),
                      text, re.M)
    if not start:
        return None
    indent = start.group(1)
    rest = text[start.end():]
    nxt = re.search(r'^%s-\s+frame:|^\S' % re.escape(indent), rest, re.M)
    end = start.end() + (nxt.start() if nxt else len(rest))
    return text[start.end():end]


def read_block(text, frame):
    """The xyz / quat_xyzw currently in the YAML, for the drift report."""
    block = block_of(text, frame)
    if block is None:
        return None, None

    def grab(key):
        m = re.search(r'^\s*%s:\s*\[([^\]]*)\]' % key, block, re.M)
        if not m:
            return None
        try:
            return [float(x) for x in m.group(1).split(",")]
        except ValueError:
            return None

    return grab("xyz"), grab("quat_xyzw")


def quat_angle_deg(a, b):
    """Angle between two quaternions, in degrees.

    abs() on the dot product because q and -q are the same rotation - without
    it an identical orientation can report 180 degrees of drift.
    """
    d = min(1.0, abs(sum(x * y for x, y in zip(a, b))))
    return math.degrees(2.0 * math.acos(d))


def patch_block(text, frame, xyz, quat, parent=None, extra=None):
    """Replace xyz / quat_xyzw inside one '- frame: <name>' block.

    Deliberately a text edit, not a YAML round-trip: these files carry the
    reasoning behind every number, and a load/dump would throw all of it away.
    """
    start = re.search(r'^(\s*)-\s+frame:\s*%s\s*$' % re.escape(frame), text, re.M)
    if not start:
        return text, False
    indent = start.group(1)
    rest = text[start.end():]
    nxt = re.search(r'^%s-\s+frame:|^\S' % re.escape(indent), rest, re.M)
    end = start.end() + (nxt.start() if nxt else len(rest))
    block = text[start.end():end]

    def fmt(v):
        return "[%s]" % ", ".join("%.6f" % x for x in v)

    new, hits = block, 0

    # The PARENT matters as much as the numbers. Reparenting the sensor from
    # the cab to the boom changes both, and a stale `parent:` would attach a
    # correct offset to the wrong frame - silently, since TF would still
    # resolve. That is the same failure mode as v1's pose on the v2 machine.
    if parent:
        new, _ = re.subn(r'^(\s*parent:\s*)\S+',
                         lambda m: m.group(1) + parent, new, count=1, flags=re.M)

    for key, val in (("xyz", fmt(xyz)), ("quat_xyzw", fmt(quat))):
        new, n = re.subn(r'^(\s*%s:\s*)\[[^\]]*\]' % key,
                         lambda m: m.group(1) + val, new, count=1, flags=re.M)
        hits += n
    for key, val in (extra or {}).items():
        new, _ = re.subn(r'^(\s*%s:\s*)[-\d.eE+]+' % re.escape(key),
                         lambda m: m.group(1) + ("%.4f" % val),
                         new, count=1, flags=re.M)
    return text[:start.end()] + new + text[end:], hits == 2


# --------------------------------------------------------------------- scan
base_link_prim = None
lidars, strays, cameras = [], [], []
for prim in stage.Traverse():
    t = str(prim.GetTypeName())
    path = str(prim.GetPath())
    if prim.GetName() == "base_link" and base_link_prim is None:
        base_link_prim = prim
    if t == "OmniLidar":
        # A lidar under /Graph or anywhere outside the robot is a leftover -
        # the ROS2 OmniGraphs generator makes one next to the graph it builds.
        # It has no link above it, so it can never have a correct transform,
        # and if the render product happens to point at it the published cloud
        # comes from a sensor floating in space.
        (lidars if nearest_link(prim) else strays).append(prim)
    elif t == "Camera" and not prim.GetName().startswith("OmniverseKit_"):
        cameras.append(prim)

if not lidars and not strays:
    raise RuntimeError("no OmniLidar prims in the stage")

print("stage units: %g m per unit\n" % MPU)
print("%-14s %-10s %-26s %-30s %s"
      % ("frame", "parent", "xyz in parent (m)", "quat xyzw", "height"))
print("-" * 100)

found = []
for prim in lidars:
    link = nearest_link(prim)
    if link is None:
        print("%-14s  NO URDF LINK ABOVE IT - %s" % (prim.GetName(), prim.GetPath()))
        continue
    xyz, quat = pose_relative_to(prim, link)

    # Height above the MACHINE's ground plane, not world Z. base_link sits on
    # the ground by definition in the URDF; the excavator prim may be placed
    # anywhere in the world, so world Z says nothing useful. (An earlier
    # version printed world Z here and reported a sensor 4.7 m underground.)
    height = None
    if base_link_prim is not None:
        z_s = cache.GetLocalToWorldTransform(prim).ExtractTranslation()[2]
        z_b = cache.GetLocalToWorldTransform(base_link_prim).ExtractTranslation()[2]
        height = (z_s - z_b) * MPU

    frame = frame_name(prim)
    print("%-14s %-10s (%7.4f,%8.4f,%7.4f)  (%7.4f,%7.4f,%7.4f,%7.4f)  %s"
          % (frame, link.GetName(), xyz[0], xyz[1], xyz[2],
             quat[0], quat[1], quat[2], quat[3],
             "%.3f m" % height if height is not None else "?"))
    found.append((frame, link.GetName(), xyz, quat, height, prim))

# WHICH LIDAR IS ACTUALLY PUBLISHING?
# The graph's Isaac Create Render Product node holds a cameraPrim relationship,
# and THAT is the sensor whose rays become the point cloud - regardless of what
# else exists in the scene or how neatly it is placed.
print("\nrender products in the graph:")
any_rp = False
for prim in stage.Traverse():
    rel = prim.GetRelationship("inputs:cameraPrim")
    if not rel:
        continue
    targets = rel.GetTargets()
    any_rp = True
    print("   %s" % prim.GetPath())
    if not targets:
        print("      cameraPrim: NOT SET - this graph renders nothing")
    for t in targets:
        tp = stage.GetPrimAtPath(t)
        mark = ""
        if tp and nearest_link(tp) is None:
            mark = "   <-- OUTSIDE THE ROBOT, its transform cannot be right"
        print("      cameraPrim: %s%s" % (t, mark))
        if tp:
            print("      publishes as frame: %s" % frame_name(tp))
if not any_rp:
    print("   none found")

if strays:
    print("\nSTRAY LIDARS (no URDF link above them - delete unless in use):")
    for p in strays:
        print("   %s" % p.GetPath())

if cameras:
    print("\ncameras found but NOT written (need the ROS optical-frame "
          "rotation, see the note at the top):")
    for c in cameras:
        print("   %s" % c.GetPath())

# ---------------------------------------------------------------- the yaml
print("\n" + "=" * 100)
if not os.path.isfile(YAML_PATH):
    print("YAML not found: %s" % YAML_PATH)
    print("fix YAML_PATH, or paste these blocks in by hand:")
    for frame, parent, xyz, quat, h, _ in found:
        print("""
  - frame: %s
    parent: %s
    topic: /%s/points
    xyz: [%s]
    quat_xyzw: [%s]
    height_above_ground_m: %.4f""" % (
            frame, parent, frame,
            ", ".join("%.6f" % v for v in xyz),
            ", ".join("%.6f" % v for v in quat), h))
else:
    text = open(YAML_PATH).read()

    # DRIFT REPORT, so a run with WRITE = False is a real audit rather than a
    # wall of lines to eyeball. "Differs" is not useful on its own: 2 mm is
    # rounding, 1.8 m and 180 deg is v1's pose still sitting on a v2 machine.
    # The magnitude is what tells you which you are looking at.
    print("DRIFT  stage vs YAML")
    print("%-16s %-12s %-12s %s"
          % ("frame", "translation", "rotation", ""))
    print("-" * 60)
    any_drift = False
    for frame, parent, xyz, quat, h, _ in found:
        old_xyz, old_quat = read_block(text, frame)
        if old_xyz is None and old_quat is None:
            print("%-16s %s" % (frame, "no block in the YAML"))
            continue
        dt = dq = None
        if old_xyz and len(old_xyz) == 3:
            dt = math.sqrt(sum((a - b) ** 2 for a, b in zip(old_xyz, xyz)))
        if old_quat and len(old_quat) == 4:
            dq = quat_angle_deg(old_quat, quat)
        note = ""
        if (dt is not None and dt > 0.001) or (dq is not None and dq > 0.1):
            any_drift = True
            note = "<-- the YAML is wrong"
        print("%-16s %-12s %-12s %s"
              % (frame,
                 "%.4f m" % dt if dt is not None else "?",
                 "%.2f deg" % dq if dq is not None else "?",
                 note))
    if not any_drift:
        print("\nevery frame agrees with the stage.")
    print()

    updated, changed = text, []
    for frame, parent, xyz, quat, h, _ in found:
        updated, ok = patch_block(updated, frame, xyz, quat, parent,
                                  {"height_above_ground_m": h})
        if ok:
            changed.append(frame)
        else:
            print("no '- frame: %s' block in the YAML - add it by hand" % frame)

    if updated == text:
        print("YAML already matches the stage - nothing to change.")
    elif WRITE:
        open(YAML_PATH, "w").write(updated)
        print("UPDATED %s" % YAML_PATH)
        print("   frames written: %s" % ", ".join(changed))
        print("\n   colcon build, relaunch, then check:")
        print("   ros2 run tf2_ros tf2_echo base_link %s" % changed[0])
    else:
        print("WOULD UPDATE %s  (set WRITE = True to apply)" % YAML_PATH)
        print("   frames: %s\n" % ", ".join(changed))
        for line in updated.splitlines():
            if any(k in line for k in
                   ("xyz:", "quat_xyzw:", "- frame:", "parent:")):
                print("   %s" % line)

print("\nCHECK THE FRAME NAMES against `frameId` on each ROS2 RTX Lidar Helper")
print("node. A name that matches no frameId publishes a transform nothing is")
print("stamped with, and a cloud whose frameId has no transform cannot be")
print("placed - neither case raises an error anywhere.")
if FRAME_OVERRIDES:
    print("\nFRAME_OVERRIDES in effect (prim -> frameId):")
    for k, v in FRAME_OVERRIDES.items():
        print("   %-16s -> %s" % (k, v))
    print("Remove an entry once the prim and its frameId share one name.")
print("\n   ros2 run tf2_ros tf2_echo base_link <frameId>")
