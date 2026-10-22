#!/usr/bin/env python3
"""
make_avia_profile.py — Isaac RTX lidar profile for the Livox Avia.

    python3 make_avia_profile.py --list              # show usable base profiles
    python3 make_avia_profile.py --src <shipped.json>
    python3 make_avia_profile.py --src <shipped.json> --mode repetitive

Specs taken from the Livox Avia User Manual (2022-04), Specifications page:

    FOV            non-repetitive  70.4 deg (H) x 77.2 deg (V)
                   repetitive      70.4 deg (H) x  4.5 deg (V)
    range @ 10%    190 m   (@100 klx); 230 m @ 20%, 320 m @ 80%
    point rate     240,000 /s first return  (480k dual, 720k triple)
    accuracy       1 sigma < 2 cm @ 20 m
    angular error  1 sigma < 0.05 deg
    beam divergence 0.03 deg (H) x 0.28 deg (V)
    wavelength     905 nm
    weight         498 g, IP67, built-in BMI088 IMU

READ THIS BEFORE TRUSTING THE RESULT
================================================================================
The Avia's defining feature is its NON-REPETITIVE scan: the beam traces a
rosette that leaves large gaps in any single frame and fills them in over
roughly half a second to a second. Isaac's profile format describes a FIXED
emitter pattern, so that cannot be represented. This profile lays the same
number of points on a UNIFORM GRID across the FOV.

That approximation is not neutral - it flatters the sensor. Every simulated
frame comes out evenly covered, where the real Avia's frames are full of holes
that only close with integration time. On a mast that is a fair trade. On the
BOOM it is not: the sensor is moving, so the real device never gets its
integration time, and the sim will show dig-zone coverage the hardware would
not deliver.

If the Avia ends up on the boom, treat this profile as an upper bound on
coverage and verify against the real sensor before believing the terrain map.

The REPETITIVE mode (--mode repetitive) does not have this problem: it is a
genuine repeating line scan, and a fixed grid over 70.4 x 4.5 deg is a fair
model of it. The cost is the vertical FOV collapsing from 77.2 to 4.5 deg.

--states N tries to model the fill-in properly by cycling N patterns, one per
frame. WHETHER ISAAC ACTUALLY CYCLES THEM IS UNVERIFIED. The format has an
emitterStateCount field and the profiles carry a list of states, which is
suggestive and nothing more; the README in the config folder documents only
how profiles are named, and it documents it WRONGLY for Isaac 6, so it is no
authority on anything. Do not trust --states until you have checked.

TWO WAYS TO CHECK
  1. --list reports emitterStateCount and the real number of states for every
     shipped profile. If any ships with more than one, the mechanism is real
     and its structure shows the intended use. If none does, nothing NVIDIA
     ships exercises this field.

  2. The empirical test, which is decisive either way. Load a --states N
     profile, press Play, and watch ONE frame at a time:

         ros2 topic echo /lidar_avia/points --once --field width

     Cycling states  -> successive frames differ in where the points fall, and
                        a frame holds ~1/N of the accumulated grid.
     Ignored         -> every frame identical. Isaac is using state 0 only, so
                        you are running at 1/N of the intended density. Switch
                        to --states 1.

     Visually: in the viewport, a cycling profile makes the cloud shimmer as
     gaps fill and reopen; an ignored one is dead still.
================================================================================
"""

import argparse
import copy
import glob
import json
import math
import os
import sys

AP = argparse.ArgumentParser()
AP.add_argument("--src", help="shipped Isaac profile .json to inherit from")
AP.add_argument("--list", action="store_true", help="list candidate base profiles")
AP.add_argument("--mode", choices=["non-repetitive", "repetitive"],
                default="non-repetitive")
AP.add_argument("--rate", type=float, default=10.0, help="frame rate, Hz")
AP.add_argument("--returns", choices=["first", "dual", "triple"], default="first")
AP.add_argument("--shape", choices=["ellipse", "rect"], default=None,
                help="FOV outline. Default: ellipse for non-repetitive (the "
                     "rosette is inscribed in one), rect for repetitive (a "
                     "line scan really is rectangular).")
AP.add_argument("--states", type=int, default=1,
                help="emitter states to cycle (see NON-REPETITIVE SCAN below). "
                     "1 = single fixed grid, the safe default.")
AP.add_argument("--show", action="store_true",
                help="dump the base profile's structure and exit - use this on "
                     "a shipped multi-state profile to check the semantics")
AP.add_argument("--out", default=None)
args = AP.parse_args()

# --- datasheet ------------------------------------------------------------
FOV = {"non-repetitive": (70.4, 77.2), "repetitive": (70.4, 4.5)}[args.mode]
PTS = {"first": 240000, "dual": 480000, "triple": 720000}[args.returns]
# 1 m is the Avia's near-field blind zone, not a conservative guess: "targets
# closer than 1 m cannot be measured". An earlier version of this file had
# 0.1 m copied from the Airy, which would let the sim return points the real
# sensor cannot produce - and on a machine whose own structure sits well
# inside 1 m of any mount, those phantom returns are exactly the ones that
# would make a bad mounting position look acceptable.
NEAR_M, FAR_M = 1.0, 190.0        # 190 m is the 10% reflectivity figure

# And between 1 and 3 m the manual warns the cloud "may show distortion".
# That band is NOT modelled here - Isaac returns clean points throughout - so
# anything the sim reports between 1 and 3 m is better than the hardware will
# do. With a dig zone starting at 2 m, that overlap is worth keeping in mind.
DISTORTION_NEAR, DISTORTION_FAR = 1.0, 3.0
REFL_RANGE = 190.0
WAVELENGTH_NM = 905.0
RANGE_ACC_M = 0.02                # 1 sigma < 2 cm @ 20 m
ANG_ERR_DEG = 0.05

# Confirmed location on this machine, from the install tree:
#   ~/isaacsim6/extsDeprecated/isaacsim.sensors.rtx/data/lidar_configs/
#
# ONLY isaacsim6. There is a second install at ~/isaacsim which is Isaac Sim 5,
# and its profiles must not be mixed into this list: a v5 profile may use
# fields the v6 loader has dropped, or omit ones it now requires, and the
# failure would surface at playback as another cryptic parameter error rather
# than at load. An earlier version of this script searched both and would have
# offered v5 profiles as valid bases.
ISAAC_ROOT = os.environ.get("ISAACSIM_ROOT") or os.path.expanduser("~/isaacsim6")

ISAAC = os.path.join(ISAAC_ROOT, "extsDeprecated", "isaacsim.sensors.rtx",
                     "data", "lidar_configs")


def find_config_dirs():
    """lidar_configs under the Isaac 6 install, with a fallback search."""
    if os.path.isdir(ISAAC):
        return [ISAAC]
    # layout moved - search this install only, never a sibling version
    dirs = []
    if os.path.isdir(ISAAC_ROOT):
        for dirpath, dirnames, _ in os.walk(ISAAC_ROOT):
            if os.path.basename(dirpath) == "lidar_configs":
                dirs.append(dirpath)
                dirnames[:] = []
    return dirs


if args.list or not args.src:
    dirs = find_config_dirs()
    if not dirs:
        print("No lidar_configs under %s" % ISAAC_ROOT)
        print("   (set ISAACSIM_ROOT if your Isaac 6 install is elsewhere -")
        print("    do NOT point it at an Isaac 5 install)")
        sys.exit(1)

    print("candidate base profiles (solid-state preferred, but see the note "
          "at the bottom):\n")
    total = 0
    for d in dirs:
        found = sorted(glob.glob(os.path.join(d, "**", "*.json"), recursive=True))
        print("%s   (%d json)" % (d, len(found)))
        for f in found:
            total += 1
            try:
                with open(f) as fh:
                    p = json.load(fh)
                prof = p.get("profile", p)
                st = prof.get("scanType", "?")
                n = prof.get("numberOfEmitters", prof.get("numberOfChannels", "?"))
                es = prof.get("emitterStates") or []
                # The declared count and the actual number of entries are
                # separate facts, and a disagreement between them says more
                # about the format than either alone.
                ns = "%d/%d" % (prof.get("emitterStateCount", -1), len(es))
                mark = ""
                if len(es) > 1:
                    mark = "  <== MULTI-STATE"
                elif "solid" in str(st).lower():
                    mark = "  (solid state)"
                print("   %-50s %-11s em=%-7s states=%-7s%s"
                      % (os.path.relpath(f, d), st, n, ns, mark))
            except Exception as e:
                print("   %-50s (unreadable: %s)" % (os.path.relpath(f, d), e))
        print()

    if not total:
        print("Directories exist but hold no .json - Isaac may ship only the")
        print("converted .usda here. Look for the JSON sources, or reuse the")
        print("profile the Airy was built from.")

    print("A ROTARY base is fine. Every emitter array, scanType, FOV and range")
    print("field is overwritten below; the base only supplies the undocumented")
    print("fields the loader needs. That is exactly why the Airy was built off")
    print("an Ouster OS0 and works.")
    print("\nstates=A/B is emitterStateCount / actual len(emitterStates).")
    print("If NO shipped profile shows B > 1, then nothing NVIDIA ships uses")
    print("multiple emitter states, and --states > 1 is an untested guess about")
    print("a field no example exercises. In that case use --states 1 and treat")
    print("its coverage as an UPPER BOUND, or verify by the empirical test in")
    print("the header. A profile with B > 1 is proof the mechanism works, and")
    print("its structure shows how the states are meant to differ.")
    if not args.src:
        print("\nrerun with --src <one of the above>")
    sys.exit(0)

if not os.path.isfile(args.src):
    sys.exit("not found: %s" % args.src)

with open(args.src) as f:
    src = json.load(f)

# --- inspect mode ----------------------------------------------------------
# The profile schema is undocumented. Before trusting --states, read a shipped
# multi-state profile with this and check that emitterStateCount really is the
# number of entries in emitterStates and that the states differ from each other.
if args.show:
    p = src.get("profile", src)
    print("STRUCTURE OF %s\n" % os.path.basename(args.src))
    for k, v in sorted(p.items()):
        if k == "emitterStates":
            print("  %-28s %d state(s)" % (k, len(v)))
            for i, st in enumerate(v):
                print("     state[%d]" % i)
                for kk, vv in st.items():
                    if isinstance(vv, list):
                        uniq = len(set(vv)) if len(vv) < 200000 else "?"
                        print("        %-20s len %-7d uniq %-7s  %s .. %s"
                              % (kk, len(vv), uniq,
                                 vv[0] if vv else "-", vv[-1] if vv else "-"))
                    else:
                        print("        %-20s %s" % (kk, vv))
        elif isinstance(v, list):
            print("  %-28s len %d, %s .. %s"
                  % (k, len(v), v[0] if v else "-", v[-1] if v else "-"))
        elif isinstance(v, dict):
            print("  %-28s {%s}" % (k, ", ".join(sorted(v))))
        else:
            print("  %-28s %s" % (k, v))
    sys.exit(0)

out = copy.deepcopy(src)
prof = out.get("profile", out)

if "emitterStates" not in prof:
    sys.exit("base profile has no 'emitterStates' key - its emitter data lives\n"
             "somewhere this script does not know about, so replacing the\n"
             "pattern would leave the original in place and the two would\n"
             "disagree. Run with --show to see its structure, and pick a base\n"
             "that does have emitterStates.")

# --- build the emitter pattern ---------------------------------------------
# The sensor emits PTS points per second no matter what. What --states changes
# is how they are DISTRIBUTED in time:
#
#   --states 1    all PTS/rate points in one fixed pattern, repeated every
#                 frame. Dense per frame, identical every frame. This is a
#                 repetitive sensor, which the Avia is not (in nonrep mode).
#
#   --states N    N patterns cycled one per frame, each holding PTS/rate points
#                 drawn from an N-times-denser grid, interleaved so successive
#                 frames fall in each other's gaps. Each frame is sparse; the
#                 union over N frames is the full grid. Points per second is
#                 identical either way - only the per-frame sparsity differs.
#
# N > 1 is the honest model of the non-repetitive scan, and the reason is the
# boom: a moving sensor only ever gets ONE frame, so the sim must show the
# sparse single-frame coverage rather than the integrated coverage. With
# --states 1 the sim shows coverage the hardware cannot deliver on a moving
# mount. Pick N = rate * integration_time, so N = 10 models the ~1 s fill at
# 10 Hz.
n_frame = int(PTS / args.rate)         # points in any single frame
N = max(1, args.states)

h_deg, v_deg = FOV

# FOV SHAPE. The datasheet gives 70.4 x 77.2 deg, and it is tempting to read
# that as a rectangle. It is not: the non-repetitive pattern is a rosette
# traced by Risley prisms, inscribed in an ELLIPSE with those axes. A
# rectangular grid illuminates the corners, and the corners are where the two
# shapes differ most - at (35.2, 38.6) a corner ray is about 52 deg off
# boresight against the ellipse's 38.6 deg maximum. A third too wide, and
# visible as a ground footprint noticeably broader than the spec.
#
# The repetitive mode IS a rectangle: a line scan sweeping 70.4 x 4.5 deg.
SHAPE = args.shape or ("ellipse" if args.mode == "non-repetitive" else "rect")
FILL = math.pi / 4.0 if SHAPE == "ellipse" else 1.0   # ellipse/rect area ratio

# Grid sizing, the ellipse mask and the point-count fit all happen in
# build_grid() below. Interleaving is by flat index: state = k mod N, where
# k counts through the masked list. build_grid keeps gcd(gx, N) == 1 so each
# row is offset differently and every state lands on a diagonal lattice across
# both axes, rather than degenerating into a vertical comb.

# HARD CONSTRAINT, learned from the loader rejecting the first attempt:
#   "Attribute ...s003:azimuthDeg is of length 24008, expecting length 24009
#    (omni:sensor:Core:numberOfEmitters)"
# EVERY state must be EXACTLY numberOfEmitters long. Not near-equal - equal.
# So the grid is truncated to a whole multiple of N and the remainder dropped;
# that is at most N-1 points out of a quarter million.
def build_grid(n_target_grid):
    """A masked angular grid of roughly n_target_grid points."""
    gy = max(1, int(round(math.sqrt(n_target_grid * v_deg / h_deg))))
    gx = max(1, int(round(n_target_grid / gy)))
    while math.gcd(gx, N) != 1 and N > 1:
        gx += 1
    out = []
    for j in range(gy):
        e = round(-v_deg / 2 + (v_deg * j / (gy - 1) if gy > 1 else v_deg / 2),
                  5)
        for i in range(gx):
            a = round(-h_deg / 2 + (h_deg * i / (gx - 1) if gx > 1 else 0.0), 5)
            if SHAPE == "ellipse":
                # normalised radius; > 1 is outside the FOV ellipse
                if (a / (h_deg / 2.0)) ** 2 + (e / (v_deg / 2.0)) ** 2 > 1.0:
                    continue
            out.append((a, e))
    return out, gx, gy


# Land on the datasheet point rate exactly rather than near it. How many grid
# points survive an elliptical mask is not simply area x density - it depends
# on where the lattice falls relative to the boundary - so the oversample
# factor is fitted rather than computed. Guessing it once left the sim at
# 237,210 pts/s against a 240,000 pts/s spec: only 1.2% off, but it is a
# headline number and there is no reason to be wrong about it.
target = n_frame * N
factor = 1.0 / FILL
pts, nx, ny = [], 1, 1
for _ in range(8):
    pts, nx, ny = build_grid(int(math.ceil(target * factor)))
    if len(pts) >= target:
        break
    factor *= float(target) / max(1, len(pts)) * 1.01

# Trim to exactly the target by dropping evenly spaced points. Truncating the
# list instead would cut whole rows off one edge and quietly reshape the FOV.
if len(pts) > target:
    step = len(pts) / float(target)
    pts = [pts[int(i * step)] for i in range(target)]

n_grid_full = len(pts)
n_per_state = n_grid_full // N
n_use = n_per_state * N
dropped = n_grid_full - n_use

states = [{"azimuthDeg": [], "elevationDeg": []} for _ in range(N)]
for k in range(n_use):
    a, e = pts[k]
    s = states[k % N]
    s["azimuthDeg"].append(a)
    s["elevationDeg"].append(e)

# The widest ray actually emitted, as an angle off boresight. This is the
# number to compare against what a point cloud looks like - a rectangle and an
# ellipse with the same quoted FOV differ here by about 14 degrees.
max_off_axis = max(
    math.degrees(math.acos(max(-1.0, min(1.0,
        math.cos(math.radians(a)) * math.cos(math.radians(e))))))
    for a, e in pts)

# fireTimeNs is deliberately all zero: a non-zero maxFireTime must fit inside
# one pattern interval, and a mismatch there is what produced
#   "maxFireTime N is greater than patternFiringRateHz converted to nanoseconds"
# on the first Airy attempt.
#
# channelId is 1-BASED. The loader rejects a 0:
#   "channelId 0 at index 0 is greater than the numberOfChannels 24009 or
#    less than 1"
# The Airy never hit this because it inherits NVIDIA's channelId array
# untouched; this script writes its own, so it has to know the convention.
#
# EVERY per-emitter array the loader wants must be present and the right
# length. Omitting one does NOT leave it out - convert_lidar_json_to_usda.py
# fills it with a default, at a default length of 128, and only on the first
# state, which surfaces at playback as:
#
#   Attribute omni:sensor:Core:emitterState:s001:focalSlope is of length 128,
#   expecting length 24059 (omni:sensor:Core:numberOfEmitters)
#
# for s001 alone while s002..s010 are fine. These are the optical corrections,
# which on real hardware are per-unit calibration. Writing them explicitly is
# what stops the converter from inventing them.
OPTICAL_DEFAULTS = {
    "distanceCorrectionM": 0.0,   # no per-emitter range bias to correct
    "focalDistM": 0.0,            # ideal sensor; no focus compensation
    "focalSlope": 0.0,
    "horOffsetM": 0.0,            # all emitters share one origin in this
    "vertOffsetM": 0.0,           # model - there is no per-beam offset data
}

# Fields with NO corresponding prim attribute. The converter reports each as
#   Field X found in profile, without corresponding prim attribute ...Skipping.
# so writing them is inert - they never reach the sensor. Dropped so the
# converter's output is not buried in warnings about things we chose to send.
# Note what is on this list: upElevationDeg, downElevationDeg, startAzimuthDeg,
# endAzimuthDeg and emitterStateCount among them. The FOV headers that
# make_airy_from_os0.py carefully keeps in step with the arrays are skipped
# too; only the per-emitter arrays actually define the pattern.
NO_PRIM_ATTRIBUTE = {
    "reportRateDiv", "minDistBetweenEchos", "effectiveApertureSize",
    "avgPowerW", "minReflectanceRange", "wavelengthNm", "reportRateBaseHz",
    "emitterStateCount", "startAzimuthDeg", "endAzimuthDeg",
    "upElevationDeg", "downElevationDeg",
}

src_state = {}
_src_states = src.get("profile", src).get("emitterStates") or []
if _src_states and isinstance(_src_states[0], dict):
    src_state = _src_states[0]

for s in states:
    n = len(s["azimuthDeg"])
    s["fireTimeNs"] = [0] * n
    s["channelId"] = list(range(1, n + 1))
    s["rangeId"] = [0] * n
    s["bank"] = [0] * n

    # Carry over any OTHER per-emitter array the base profile had, retiled to
    # this state's length. Done by inspection rather than a fixed list, so a
    # field this script has never heard of still comes through at the right
    # size instead of being defaulted by the converter.
    for k, v in src_state.items():
        if k in s or k in NO_PRIM_ATTRIBUTE or not isinstance(v, list) or not v:
            continue
        s[k] = [v[i % len(v)] for i in range(n)]

    # Then guarantee the ones that must exist even if the base lacked them.
    for k, d in OPTICAL_DEFAULTS.items():
        if k not in s:
            s[k] = [d] * n

# --- the 5 MiB wall --------------------------------------------------------
# rtx.hydra refuses to serialize a sensor whose attributes exceed 5 MiB:
#   Exceeded maximum attributes size of 5.0MiB: 9.095MiB
#   Failed to serialize sensor attributes for sensor '/OmniLidar'
# and then the sensor does not work at all. Every per-emitter array in every
# state counts toward it, so the budget is shared between resolution and the
# number of states. Measured against a real conversion: 10 states of 24059
# emitters came to 9.095 MiB, i.e. about 40 bytes per emitter per state.
BYTES_PER = 42                      # rounded up from the measured 39.6
MAX_ATTR_BYTES = 5 * 1024 * 1024
est = n_per_state * N * BYTES_PER
max_states = max(1, MAX_ATTR_BYTES // (n_per_state * BYTES_PER))

if est > MAX_ATTR_BYTES:
    sys.exit(
        "REFUSING TO WRITE - this would exceed Isaac's sensor attribute cap.\n"
        "   estimated   %.2f MiB  (%d emitters x %d states x ~%d bytes)\n"
        "   hard cap    5.00 MiB\n"
        "   Over the cap rtx.hydra fails to serialize the sensor and it does\n"
        "   not work at all - this is not a warning you can ignore.\n\n"
        "   At %d emitters per state the most that fits is --states %d.\n"
        "   But see the s002 note below: multi-state is not dependable here,\n"
        "   and --states 1 is the option that actually works.\n"
        % (est / 1048576.0, n_per_state, N, BYTES_PER, n_per_state, max_states))

counts = [len(s["azimuthDeg"]) for s in states]
prof["emitterStates"] = states

prof["scanType"] = "solidState"
prof["numberOfEmitters"] = n_per_state
prof["numberOfChannels"] = n_per_state
prof["emitterStateCount"] = N

# --- refuse to write a profile the loader will reject ----------------------
# Every one of these was a real rejection at playback. Checking here turns a
# cryptic runtime error into a failure at the point the mistake was made.
problems = []
if len(set(counts)) != 1:
    problems.append("states differ in length: %s" % sorted(set(counts)))
for idx, s in enumerate(states):
    n = len(s["azimuthDeg"])
    if n != n_per_state:
        problems.append("state %d has %d entries, numberOfEmitters is %d"
                        % (idx, n, n_per_state))
    bad = [k for k, v in s.items() if len(v) != n]
    if bad:
        problems.append("state %d arrays disagree: %s" % (idx, bad))
    if min(s["channelId"]) < 1:
        problems.append("state %d has channelId < 1" % idx)
    if max(s["channelId"]) > n_per_state:
        problems.append("state %d channelId exceeds numberOfChannels" % idx)
    if max(s["bank"]) >= max(prof.get("numLines", 1), 1):
        problems.append("state %d bank >= numLines" % idx)
if prof["emitterStateCount"] != len(states):
    problems.append("emitterStateCount != len(emitterStates)")
if problems:
    sys.exit("REFUSING TO WRITE - the loader would reject this:\n   "
             + "\n   ".join(problems))
# bank is all zero, so numLines only has to exceed 0 - but an inherited value
# that happens to be 0 would trip "bankId 0 is greater than numLines 0".
if prof.get("numLines", 0) < 1:
    prof["numLines"] = 1
prof["scanRateBaseHz"] = args.rate
prof["reportRateBaseHz"] = args.rate
if "patternFiringRateHz" in prof:
    prof["patternFiringRateHz"] = float(PTS)

prof["startAzimuthDeg"] = -h_deg / 2
prof["endAzimuthDeg"] = h_deg / 2
prof["upElevationDeg"] = v_deg / 2
prof["downElevationDeg"] = -v_deg / 2
prof["nearRangeM"] = NEAR_M
prof["farRangeM"] = FAR_M
prof["minReflectance"] = 0.1
prof["minReflectanceRange"] = REFL_RANGE
prof["rangeAccuracyM"] = RANGE_ACC_M
prof["azimuthErrorStd"] = ANG_ERR_DEG
prof["elevationErrorStd"] = ANG_ERR_DEG
if "wavelengthNm" in prof:
    prof["wavelengthNm"] = WAVELENGTH_NM


def strip_comments(node):
    """Drop the source sensor's datasheet notes - they describe another part."""
    if isinstance(node, dict):
        for k in [k for k in node if k.startswith("comment")]:
            del node[k]
        for v in node.values():
            strip_comments(v)
    elif isinstance(node, list):
        for v in node:
            strip_comments(v)


strip_comments(out)
tag = "NonRep" if args.mode == "non-repetitive" else "Rep"
name = "AUWO_Avia_%s" % tag + ("_%dst" % N if N > 1 else "")
out["name"] = name
prof["comment_source"] = ("derived from %s; fields not listed below are "
                          "inherited unchanged" % os.path.basename(args.src))
prof["comment_fov"] = ("Livox Avia, %s scan: %.1f x %.1f deg"
                       % (args.mode, h_deg, v_deg))
if N > 1:
    prof["comment_approx"] = (
        "%d emitter states cycled one per frame. Each frame carries %d points "
        "on a sparse interleaved lattice; the union over %d frames (%.2f s at "
        "%.0f Hz) is the full %d-point grid. This models the Avia's "
        "non-repetitive fill-in: a STATIC mount accumulates the dense grid, a "
        "mount moving on the boom only ever gets one sparse frame."
        % (N, min(counts), N, N / args.rate, args.rate, n_grid))
else:
    prof["comment_approx"] = (
        "SINGLE fixed grid - every frame identical. Correct for the repetitive "
        "scan mode. For the non-repetitive mode it OVERSTATES single-frame "
        "coverage, because the real sensor's frames are full of gaps that only "
        "close with integration time. Use --states 10 for that.")

dst = args.out or os.path.join(os.path.dirname(os.path.abspath(args.src)),
                               name + ".json")
with open(dst, "w") as f:
    json.dump(out, f, indent=2)

# --- report ---------------------------------------------------------------
print("wrote %s\n" % dst)
print("  mode            %s" % args.mode)
print("  FOV             %.1f x %.1f deg  (%s)" % (h_deg, v_deg, SHAPE))
print("  widest ray      %.1f deg off boresight%s"
      % (max_off_axis,
         "" if SHAPE == "ellipse" else
         "  <-- a rect reaches past the spec in its corners"))
print("  states          %d  (%s)"
      % (N, "one fixed pattern, every frame identical" if N == 1
         else "cycled one per frame, %.2f s to fill" % (N / args.rate)))
print("  per frame       %d points  (all %d states exactly equal, as required)"
      % (n_per_state, N))
if N > 1:
    print("  accumulated     %d points over %d frames  (%d az x %d el grid)"
          % (n_use, N, nx, ny))
    if dropped:
        print("  dropped         %d point(s) to make the grid divisible by %d"
              % (dropped, N))
print("  pitch, filled   %.3f x %.3f deg" % (h_deg / max(nx - 1, 1),
                                             v_deg / max(ny - 1, 1)))
print("  pitch, 1 frame  %.3f x %.3f deg  <-- what a MOVING sensor gets"
      % ((h_deg / max(nx - 1, 1)) * math.sqrt(N),
         (v_deg / max(ny - 1, 1)) * math.sqrt(N)))
print("  point rate      %d /s at %.0f Hz  (unchanged by --states)"
      % (PTS, args.rate))
print("  attr size       ~%.2f MiB of 5.00 MiB cap" % (est / 1048576.0))
if N > 1:
    print()
    print("  !! s002 IS NOT SUPPORTED. The converter reports every field of")
    print("     emitter state index 1 as having no prim attribute:")
    print("        Field emitterState[1].azimuthDeg found in profile, without")
    print("        corresponding prim attribute ...s002:azimuthDeg. Skipping.")
    print("     s001 and s003..s010 are accepted; s002 alone is dropped. So a")
    print("     %d-state profile silently loses the %d points assigned to it,"
          % (N, n_per_state))
    print("     and the pattern has a hole in it that nothing reports again.")
    print()
    print("     Multi-state also forces MULTI-TICK, which warns:")
    print("        'Multi-tick is enabled but motion BVH is not active.")
    print("         This is not supported.'")
    print()
    print("     USE --states 1. To model the non-repetitive fill-in, decimate")
    print("     the published cloud in a ROS node instead: same effect, under")
    print("     your control, no attribute cap and no missing slots.")
print("  range           %.1f .. %.0f m  (190 m @ 10%% reflectivity)"
      % (NEAR_M, FAR_M))
print("  derived from    %s" % os.path.basename(args.src))

print("\n  GROUND COVERAGE from a 2.0 m boom mount:")
H = 2.0
nadir = H * math.tan(math.radians(v_deg / 2))
print("     aimed straight down : 0 .. %.2f m   (dig zone is 2 - 4 m)" % nadir)
c = (math.degrees(math.atan(H / 2.0)) + math.degrees(math.atan(H / 4.0))) / 2
lo, hi = c - v_deg / 2, c + v_deg / 2
near = H / math.tan(math.radians(hi)) if hi > 0 else 0.0
print("     tilted %.1f deg down : %.2f m .. %s"
      % (c, near, "horizon" if lo <= 0 else "%.2f m" % (H / math.tan(math.radians(lo)))))
print("     horizontally it is a %.1f deg wedge - NOT surround coverage" % h_deg)

print("\n  NEXT")
in_place = os.path.abspath(dst).startswith(os.path.abspath(ISAAC_ROOT))
if not in_place:
    print("     This file is NOT under the Isaac install, so the converter")
    print("     cannot see it. Move it into a lidar_configs subfolder first,")
    print("     or rerun with --out pointing there.")
print("     cd %s" % ISAAC_ROOT)
print("     ./python.sh tools/isaacsim.sensors.rtx/convert_lidar_json_to_usda.py \\")
print("         -f %s" % os.path.abspath(dst))
print("     # writes %s" % (os.path.splitext(os.path.abspath(dst))[0] + ".usd"))
print()
print("     Then attach by usd_path - NOT by config name. Edit")
print("     add_airy_by_usd_path.py: point AIRY_USD at the .usd above and set")
print("     NAME, then run it in the Script Editor with the timeline STOPPED.")
print("     Verify it printed ~%d emitters; %s means the base profile loaded"
      % (max(counts), src.get("profile", src).get("numberOfEmitters", "?")))
print("     and yours did not.")