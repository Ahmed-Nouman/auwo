#!/usr/bin/env python3
"""
fix_missing_textures.py — comment out .mtl texture maps whose file is absent.

    python3 fix_missing_textures.py ~/Desktop/auwo_ws/src/excavator_v2_description/meshes
    python3 fix_missing_textures.py <dir> --apply      # actually write

WHY
excavator_v2_description's .mtl files were exported from Blender with a full
PBR set:

    map_Kd    Excavator_BaseColor.png     present
    map_Bump  Excavator_Normal.png        MISSING
    map_Ns    Excavator_Roughness.png     MISSING
    map_refl  Excavator_Metallic.png      MISSING

Only the base colour was committed. RViz does not care - Assimp/Ogre ignore
maps it cannot find - so the model looks right there. Foxglove renders through
three.js, which fetches every referenced texture via foxglove_bridge and fails
the whole material when one cannot be retrieved, so the mesh never appears.
That asymmetry is why the machine rendered in one viewer and not the other.

THIS IS THE WORKAROUND, NOT THE FIX
The right fix is to get the three missing PNGs from whoever exported the model
and commit them; the material is then what the artist intended. Commenting the
lines out only makes the .mtl honest about what is actually shipped - the model
renders with its base colour and no surface detail.

Every file is backed up to <name>.mtl.bak before being touched, and the script
reports before it writes unless --apply is given.
"""

import os
import re
import sys

MAP_KEYS = ("map_Kd", "map_Ka", "map_Ks", "map_Ns", "map_d", "map_Bump",
            "map_bump", "bump", "map_refl", "refl", "norm", "map_Pr", "map_Pm")


def texture_of(line):
    """The filename a map line points at, ignoring its option flags.

    'map_Bump -bm 1.000000 Excavator_Normal.png' -> 'Excavator_Normal.png'
    Options come as '-flag value', so the filename is simply the last token.
    """
    parts = line.split()
    if not parts or parts[0] not in MAP_KEYS:
        return None
    return parts[-1] if len(parts) > 1 else None


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    apply_it = "--apply" in sys.argv
    if not args:
        sys.exit(__doc__)
    d = os.path.expanduser(args[0])
    if not os.path.isdir(d):
        sys.exit("not a directory: %s" % d)

    mtls = sorted(f for f in os.listdir(d) if f.endswith(".mtl"))
    if not mtls:
        sys.exit("no .mtl files in %s" % d)

    total_missing = 0
    missing_names = set()

    for name in mtls:
        path = os.path.join(d, name)
        with open(path) as f:
            lines = f.readlines()

        out, changed = [], 0
        for line in lines:
            tex = texture_of(line.strip())
            if tex and not os.path.isfile(os.path.join(d, tex)):
                missing_names.add(tex)
                changed += 1
                out.append("# MISSING, commented out by fix_missing_textures.py: "
                           + line.lstrip())
            else:
                out.append(line)

        if changed:
            total_missing += changed
            print("%-28s %d missing map(s)" % (name, changed))
            if apply_it:
                bak = path + ".bak"
                if not os.path.exists(bak):
                    with open(bak, "w") as f:
                        f.writelines(lines)
                with open(path, "w") as f:
                    f.writelines(out)

    print()
    if not total_missing:
        print("every referenced texture is present - nothing to do.")
        return

    print("missing textures: %s" % ", ".join(sorted(missing_names)))
    print("%d map line(s) across %d file(s)" % (total_missing, len(mtls)))
    if apply_it:
        print("\nWRITTEN. Originals kept as *.mtl.bak")
        print("Rebuild so the install/ copies are updated:")
        print("   colcon build --symlink-install --packages-select "
              "excavator_v2_description")
    else:
        print("\nreport only - rerun with --apply to write.")
        print("Ask for the real textures first if they can be found; this only")
        print("makes the .mtl match what is actually in the repo.")


if __name__ == "__main__":
    main()