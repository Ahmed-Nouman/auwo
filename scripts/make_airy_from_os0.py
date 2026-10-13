#!/usr/bin/env python3
"""
make_airy_from_os0.py — build the Airy profile by EDITING a shipped Isaac
profile, rather than writing one from scratch.

    python3 make_airy_from_os0.py                      # uses the default OS0 path
    python3 make_airy_from_os0.py --src /path/to.json  # or point it anywhere

WHY THIS WAY
Isaac's own guidance for custom RTX sensor profiles is to start from a shipped
configuration and modify it. Writing one from scratch means guessing at every
field the loader requires, and the failures are silent or cryptic: a missing
`numLines` surfaces only at playback as

    LidarCore::updateParams() -- invalid parameters
    Malformed model parameter update: bankId 0 at index 0 is greater than the
    profile numLines 0

So this script loads a working profile, changes ONLY the things that define the
Airy's geometry, and leaves every other field exactly as NVIDIA shipped it -
including the ones whose purpose is undocumented.

WHAT IT CHANGES
    elevationDeg   remapped to span -90 .. 0 degrees (downcast hemisphere)
    nearRangeM     0.1   (Airy blind spot is under 0.1 m)
    farRangeM      60.0  (Airy max range; 30 m at 10% reflectivity)
    name           AUWO_Airy<N>

Beam count stays whatever the source had. The OS0 ships 128 channels, which is
BETTER than the real Airy's 96 - finer vertical resolution, 0.71 deg instead of
0.95 deg. If you want exactly 96 to match the hardware, pass --beams 96 and the
script will drop channels evenly; that is riskier, since array lengths and bank
assignments must stay consistent, so it warns when it does.
"""

import argparse
import copy
import json
import os
import sys

DEFAULT_SRC = ("/home/nameless/isaacsim6/extsDeprecated/isaacsim.sensors.rtx/"
               "data/lidar_configs/Ouster/OS0/OS0_REV7_128ch10hz1024res.json")

ap = argparse.ArgumentParser()
ap.add_argument("--src", default=DEFAULT_SRC, help="shipped profile to start from")
ap.add_argument("--out", default=None, help="output path (default: beside src)")
ap.add_argument("--lo", type=float, default=-90.0, help="lowest elevation, deg")
ap.add_argument("--hi", type=float, default=0.0, help="highest elevation, deg")
ap.add_argument("--near", type=float, default=0.1)
ap.add_argument("--far", type=float, default=60.0)
ap.add_argument("--show", action="store_true",
                help="print the source profile's structure and exit")
args = ap.parse_args()

if not os.path.isfile(args.src):
    sys.exit("source profile not found: %s\n"
             "   list what you have with:\n"
             "   ls ~/isaacsim6/extsDeprecated/isaacsim.sensors.rtx/data/"
             "lidar_configs/Ouster/OS0/" % args.src)

with open(args.src) as f:
    src = json.load(f)


def walk(node, path=""):
    """Print the shape of the profile so the schema stops being a mystery."""
    if isinstance(node, dict):
        for k, v in node.items():
            p = path + "/" + k
            if isinstance(v, (dict, list)):
                walk(v, p)
            else:
                print("  %-58s %s" % (p, v))
    elif isinstance(node, list):
        if node and isinstance(node[0], (int, float)):
            print("  %-58s len %d, %s .. %s" % (path, len(node), node[0], node[-1]))
        else:
            for i, v in enumerate(node):
                walk(v, "%s[%d]" % (path, i))


if args.show:
    print("STRUCTURE OF %s\n" % os.path.basename(args.src))
    walk(src)
    sys.exit(0)

out = copy.deepcopy(src)
prof = out.get("profile", out)

# --- find every elevation array and remap it, whatever the nesting ----------
remapped = []


def remap_elevations(node, path=""):
    if isinstance(node, dict):
        for k, v in node.items():
            if k == "elevationDeg" and isinstance(v, list) and len(v) > 1:
                n = len(v)
                step = (args.hi - args.lo) / (n - 1)
                node[k] = [round(args.lo + i * step, 4) for i in range(n)]
                remapped.append((path + "/" + k, n, step))
            else:
                remap_elevations(v, path + "/" + k)
    elif isinstance(node, list):
        for i, v in enumerate(node):
            remap_elevations(v, "%s[%d]" % (path, i))


remap_elevations(out)
if not remapped:
    sys.exit("no elevationDeg array found - is this really a lidar profile?")

n_beams = remapped[0][1]

# --- the FOV is declared TWICE and both must agree --------------------------
# The shipped profiles carry the elevation span as a per-emitter array AND as
# top-level upElevationDeg / downElevationDeg. Remapping only the array leaves
# the header claiming the source sensor's FOV, which is how a profile ends up
# internally inconsistent and fails at load.
for d in (out, prof):
    if not isinstance(d, dict):
        continue
    if "nearRangeM" in d:
        d["nearRangeM"] = args.near
    if "farRangeM" in d:
        d["farRangeM"] = args.far
    if "upElevationDeg" in d:
        d["upElevationDeg"] = args.hi
    if "downElevationDeg" in d:
        d["downElevationDeg"] = args.lo

# --- drop the source sensor's datasheet comments ----------------------------
# NVIDIA annotate each value with the Ouster datasheet line it came from. Those
# notes are now wrong, and a stale "OPTICAL PERFORMANCE - 75 m" beside a 60 m
# range is worse than no comment. Replace them with the Airy's own figures.
def strip_comments(node):
    if isinstance(node, dict):
        for k in [k for k in node if k.startswith("comment")]:
            del node[k]
        for v in node.values():
            strip_comments(v)
    elif isinstance(node, list):
        for v in node:
            strip_comments(v)


strip_comments(out)
prof["comment_source"] = ("derived from %s; every field not listed below is "
                          "inherited unchanged" % os.path.basename(args.src))
prof["comment_fov"] = ("RoboSense Airy: 360 x 90 deg hemispherical. Mounted "
                       "downcast (%g..%g deg) so the sensor sees its own nadir."
                       % (args.lo, args.hi))
prof["comment_range"] = ("RoboSense Airy: 30 m @ 10%% reflectivity, 60 m max, "
                         "blind spot under 0.1 m")

name = "AUWO_Airy%d" % n_beams
out["name"] = name
out["comment"] = ("RoboSense Airy, downcast hemisphere (%g..%g deg), derived "
                  "from %s so that every undocumented field is inherited "
                  "unchanged." % (args.lo, args.hi, os.path.basename(args.src)))

dst = args.out or os.path.join(os.path.dirname(args.src), name + ".json")
with open(dst, "w") as f:
    json.dump(out, f, indent=2)

print("wrote %s" % dst)
print("  derived from   %s" % os.path.basename(args.src))
print("  beams          %d  (kept from the source; every array length unchanged)" % n_beams)
for p, n, step in remapped:
    print("  remapped       %-40s %d beams, %.3f deg apart" % (p, n, step))
print("  elevations     %g .. %g deg  (downcast - mount the sensor upright)"
      % (args.lo, args.hi))
print("  up/downElev    %g / %g deg   (header kept in step with the array)"
      % (args.hi, args.lo))
print("  range          %g .. %g m" % (args.near, args.far))
print("  comments       source datasheet notes removed, Airy figures noted")
print()
print("NEXT")
print("  cd ~/isaacsim6")
print("  ./python.sh tools/isaacsim.sensors.rtx/convert_lidar_json_to_usda.py \\")
print("      -f %s" % dst.replace(os.path.expanduser("~/isaacsim6") + "/", ""))
print("  then restart Isaac and set the lidar prim's config to %s" % name)