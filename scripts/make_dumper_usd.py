#!/usr/bin/env python3
"""Build env/dumper/dumper.usda from the dumper_description OBJ (same mesh as Gazebo/MoveIt).

The asset is a static collider (triangle-mesh collision, so the dump bed stays open); place
it in the scene by hand. One Mesh per OBJ material, UsdPreviewSurface colours from the .mtl. Frame: +X towards the dump bed, origin on the ground under the track centre.

Needs Isaac Sim's USD (pxr); run with scripts/isaac_usd_python.sh:
  scripts/isaac_usd_python.sh scripts/make_dumper_usd.py
"""
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OBJ = os.path.join(REPO, 'src', 'dumper_description', 'models', 'dumper', 'meshes', 'dumper.obj')
OUT = os.path.join(REPO, 'env', 'dumper', 'dumper.usda')


def read_mtl(path):
    colors, name = {}, None
    for line in open(path):
        tok = line.split()
        if not tok:
            continue
        if tok[0] == 'newmtl':
            name = tok[1]
        elif tok[0] == 'Kd' and name:
            colors[name] = tuple(float(c) for c in tok[1:4])
    return colors


def read_obj(path):
    """Return (mtllib, {material: (points, normals, face_counts, face_indices, face_normals)})."""
    v, vn, mtllib, mat = [], [], None, 'default'
    parts = {}
    for line in open(path):
        tok = line.split()
        if not tok:
            continue
        if tok[0] == 'v':
            v.append(tuple(float(c) for c in tok[1:4]))
        elif tok[0] == 'vn':
            vn.append(tuple(float(c) for c in tok[1:4]))
        elif tok[0] == 'mtllib':
            mtllib = tok[1]
        elif tok[0] == 'usemtl':
            mat = tok[1]
        elif tok[0] == 'f':
            p = parts.setdefault(mat, {'map': {}, 'pts': [], 'counts': [], 'idx': [], 'nrm': []})
            corners = [c.split('/') for c in tok[1:]]
            p['counts'].append(len(corners))
            for c in corners:
                vi = int(c[0]) - 1
                if vi not in p['map']:
                    p['map'][vi] = len(p['pts'])
                    p['pts'].append(v[vi])
                p['idx'].append(p['map'][vi])
                p['nrm'].append(vn[int(c[2]) - 1] if len(c) > 2 and c[2] else (0.0, 0.0, 1.0))
    return mtllib, parts


def main():
    mtllib, parts = read_obj(OBJ)
    colors = read_mtl(os.path.join(os.path.dirname(OBJ), mtllib)) if mtllib else {}

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    stage = Usd.Stage.CreateNew(OUT)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)

    root = UsdGeom.Xform.Define(stage, '/dumper')
    stage.SetDefaultPrim(root.GetPrim())
    root.GetPrim().SetDocumentation('Tracked mini dumper, ~3.15 x 0.92 x 1.45 m (from Dumper_packed.blend)')

    looks = UsdGeom.Scope.Define(stage, '/dumper/Looks')
    for i, (mat, p) in enumerate(sorted(parts.items())):
        safe = Sdf.Path.MakeValidIdentifier(mat) if hasattr(Sdf.Path, 'MakeValidIdentifier') else \
            ''.join(ch if ch.isalnum() else '_' for ch in mat)
        mesh = UsdGeom.Mesh.Define(stage, f'/dumper/geometry/{safe}')
        mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*q) for q in p['pts']]))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray(p['counts']))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(p['idx']))
        mesh.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(*n) for n in p['nrm']]))
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        rgb = Gf.Vec3f(*colors.get(mat, (0.5, 0.5, 0.5)))
        mesh.CreateDisplayColorAttr([rgb])

        material = UsdShade.Material.Define(stage, f'/dumper/Looks/{safe}')
        shader = UsdShade.Shader.Define(stage, f'/dumper/Looks/{safe}/PreviewSurface')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(rgb)
        metal = mat.lower().startswith('metal')
        shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(0.8 if metal else 0.0)
        shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(0.35 if metal else 0.5)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)

        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(
            UsdPhysics.Tokens.none)
        print(f'{mat}: {len(p["pts"])} points, {len(p["counts"])} faces')

    stage.GetRootLayer().Save()
    print('wrote', OUT)


if __name__ == '__main__':
    sys.exit(main())
