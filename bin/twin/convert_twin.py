#!/usr/bin/env python3
"""Convert the scanned lab twin (Isaac USD from build_isaac_scene.py) into MuJoCo MJCF with the Unitree G1.

    venv/bin/python convert_twin.py <hall_scene.usda> <g1 folder with g1_29dof.xml + meshes> <out dir>

What the USD holds (walk5 build, 2026-11-08): /World/Ground (one quad at z = 0), /World/Static (one collision mesh made of
axis-aligned 10 cm columns from the LiDAR map: 8 points and 5 faces per column, no bottom face), /World/Fixed and
/World/Movable (Cube prims with translate + scale and a semantics class). Everything is axis-aligned boxes, so it maps 1:1
onto MuJoCo box geoms. That matters: MuJoCo collides a mesh through its convex hull, so the Static mesh loaded as one
mesh would become one solid block over the whole hall. (Joining neighbouring columns was tried: no two neighbours share a height, so
nothing merges; 6168 box geoms step at about 0.7 ms on one CPU core.)
"""
import argparse, shutil
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom

COLORS = {"static": "0.62 0.62 0.60 1", "chair": "0.20 0.35 0.65 1", "table": "0.62 0.45 0.30 1", "counter": "0.55 0.40 0.30 1",
          "tv screen": "0.10 0.10 0.12 1", "plant": "0.25 0.55 0.25 1", "window": "0.55 0.75 0.90 0.5", "door": "0.50 0.35 0.25 1",
          "refrigerator": "0.85 0.85 0.88 1", "stage": "0.35 0.30 0.28 1", "shelf": "0.60 0.50 0.35 1", "stairs": "0.80 0.55 0.20 1"}


def static_columns(mesh):
    pts = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float64)
    assert len(pts) % 8 == 0, "Static mesh is not made of 8-point columns"
    c = pts.reshape(-1, 8, 3)
    lo, hi = c.min(1), c.max(1)
    # every column must be an axis-aligned box: each coordinate takes exactly its min or max
    ok = np.all((np.isclose(c, lo[:, None]) | np.isclose(c, hi[:, None])), axis=(1, 2))
    assert ok.all(), f"{(~ok).sum()} columns are not axis-aligned boxes"
    return lo, hi


def box(name, lo, hi, rgba, cls):
    c, h = (lo + hi) / 2, (hi - lo) / 2
    return (f'    <geom name="{name}" class="{cls}" type="box" pos="{c[0]:.4f} {c[1]:.4f} {c[2]:.4f}" '
            f'size="{h[0]:.4f} {h[1]:.4f} {h[2]:.4f}" rgba="{rgba}"/>')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("usd"); ap.add_argument("g1"); ap.add_argument("out")
    a = ap.parse_args()
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    shutil.copy(Path(a.g1, "g1_29dof.xml"), out / "g1_29dof.xml")
    if not (out / "meshes").exists():
        shutil.copytree(Path(a.g1, "meshes"), out / "meshes")

    st = Usd.Stage.Open(a.usd)
    geoms, ground, n_cols, objs = [], None, 0, {}
    for prim in st.Traverse():
        t = prim.GetTypeName()
        if t == "Mesh" and prim.GetName() == "Ground":
            p = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get())
            ground = (p.min(0), p.max(0))
        elif t == "Mesh" and prim.GetName() == "Static":
            lo, hi = static_columns(UsdGeom.Mesh(prim)); n_cols = len(lo)
            geoms += [box(f"static_{i:05d}", l, h, COLORS["static"], "static") for i, (l, h) in enumerate(zip(lo, hi))]
        elif t == "Cube":
            xf = UsdGeom.Xformable(prim)
            assert [o.GetOpName() for o in xf.GetOrderedXformOps()] == ["xformOp:translate", "xformOp:scale"], prim.GetPath()
            size = prim.GetAttribute("size").Get() or 2.0
            T = np.array(prim.GetAttribute("xformOp:translate").Get()); S = np.array(prim.GetAttribute("xformOp:scale").Get())
            half = 0.5 * size * S
            cls = (prim.GetAttribute("semantics:labels:class").Get() or ["object"])[0]
            group = prim.GetParent().GetName().lower()   # fixed / movable
            objs[cls] = objs.get(cls, 0) + 1
            geoms.append(box(f"{group}_{prim.GetName()}", T - half, T + half, COLORS.get(cls, "0.7 0.7 0.7 1"), group))
    g_lo, g_hi = ground
    gc, gh = (g_lo + g_hi) / 2, (g_hi - g_lo) / 2
    (out / "scene.xml").write_text(f"""<mujoco model="Lab hall twin (G1 LiDAR map, walk5)">
  <!-- convert_twin.py from {Path(a.usd).name}: {n_cols} static 10 cm columns,
       {sum(objs.values())} labelled boxes {dict(sorted(objs.items()))}. Origin = robot start, z up, metres.
       Collision geometry only (boxes); no textures. The G1 stands at the start pose; it has no controller. -->
  <include file="g1_29dof.xml"/>
  <option timestep="0.002"><flag speculative="enable"/></option>
  <statistic center="-3 1 1" extent="14"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.35 0.35 0.35" specular="0 0 0"/>
    <global azimuth="135" elevation="-35"/>
    <map zfar="200"/>
  </visual>
  <default>
    <default class="static"><geom contype="1" conaffinity="1" group="2"/></default>
    <default class="fixed"><geom contype="1" conaffinity="1" group="1"/></default>
    <default class="movable"><geom contype="1" conaffinity="1" group="1"/></default>
  </default>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.35 0.45 0.55" rgb2="0.05 0.05 0.08" width="512" height="3072"/>
    <texture type="2d" name="floor" builtin="checker" mark="edge" rgb1="0.40 0.41 0.43" rgb2="0.34 0.35 0.37" markrgb="0.25 0.25 0.25" width="300" height="300"/>
    <material name="floor" texture="floor" texuniform="true" texrepeat="1 1" reflectance="0.05"/>
  </asset>
  <worldbody>
    <light pos="-3 1 8" dir="0 0 -1" directional="true" castshadow="true"/>
    <geom name="ground" type="box" material="floor" pos="{gc[0]:.3f} {gc[1]:.3f} -0.05" size="{gh[0]:.3f} {gh[1]:.3f} 0.05"/>
{chr(10).join(geoms)}
  </worldbody>
  <keyframe>
    <key name="start" qpos="0 0 0.793 1 0 0 0{' 0' * 29}"/>
  </keyframe>
</mujoco>
""")
    print(f"{n_cols} static columns; labelled {objs}; ground {np.round(g_lo, 2)} .. {np.round(g_hi, 2)}")


if __name__ == "__main__":
    main()
