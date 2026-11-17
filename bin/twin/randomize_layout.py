#!/usr/bin/env python3
"""Random furniture layouts for the scene from build_isaac_scene.py: the static layer stays, the movable boxes get new places.

    python3 randomize_layout.py <scene_dir> [--seeds 0 1 2 3 4 5 6 7] [--keep 0.6 1.0] [--margin 0.15] [--spawn-clear 1.0]

Per seed: a random share (--keep) of the movable objects of the recording day is placed on the placeable floor
(layout_space.npz), each at a random spot, turned by 0 or 90 deg, not overlapping anything that stays or each other
(--margin apart), and not within --spawn-clear of the spawn spot of the report.
Writes <scene_dir>/layouts/layout_<seed>.usda (sublayer = ../hall_static.usda, adds /World/Movable),
layout_<seed>.json (same keys as scene_report.json, so check_isaac_scene.py and render_scene_preview.py take it) and
<scene_dir>/layouts_topdown.png.
"""
import argparse, json, os
import numpy as np
from scene_usd import PAL, cube_group, header

ap = argparse.ArgumentParser()
ap.add_argument("scene")
ap.add_argument("--seeds", type=int, nargs="*", default=list(range(8)))
ap.add_argument("--keep", type=float, nargs=2, default=[0.6, 1.0], help="share of the movable objects used, drawn per layout")
ap.add_argument("--margin", type=float, default=0.15, help="free space kept around each placed object (m)")
ap.add_argument("--spawn-clear", type=float, default=1.0, help="radius around the spawn spot that stays empty (m)")
ap.add_argument("--tries", type=int, default=300)
a = ap.parse_args()
rep = json.load(open(os.path.join(a.scene, "scene_report.json")))
sp = np.load(os.path.join(a.scene, "layout_space.npz"))
lo, cell, placeable0 = sp["lo"], float(sp["cell"]), sp["placeable"]
nx, ny = placeable0.shape
fixed = [o for o in rep["objects"] if not o["movable"]]
movable = [o for o in rep["objects"] if o["movable"]]
if rep["spawn_xy"]:
    ii, jj = np.mgrid[0:nx, 0:ny]
    placeable0 = placeable0 & (np.hypot(lo[0] + (ii + 0.5) * cell - rep["spawn_xy"][0], lo[1] + (jj + 0.5) * cell - rep["spawn_xy"][1]) > a.spawn_clear)
os.makedirs(os.path.join(a.scene, "layouts"), exist_ok=True)
mg = int(np.ceil(a.margin / cell))

layouts = []
for seed in a.seeds:
    rng = np.random.default_rng(seed)
    free = placeable0.copy()
    n_use = int(round(rng.uniform(*a.keep) * len(movable)))
    use = [movable[i] for i in rng.choice(len(movable), n_use, replace=False)]
    use.sort(key=lambda o: -(o["max"][0] - o["min"][0]) * (o["max"][1] - o["min"][1]))  # big ones first
    placed, cnt = [], {}
    for o in use:
        size = np.array(o["max"]) - np.array(o["min"])
        for _ in range(a.tries):
            sx, sy = (size[1], size[0]) if rng.random() < 0.5 else (size[0], size[1])
            cand = np.argwhere(free)
            if len(cand) == 0:
                break
            ci, cj = cand[rng.integers(len(cand))]
            c = lo + (np.array([ci, cj]) + 0.5) * cell
            i0, j0 = np.floor((c - [sx / 2, sy / 2] - lo) / cell).astype(int); i1, j1 = np.ceil((c + [sx / 2, sy / 2] - lo) / cell).astype(int)
            if i0 < 0 or j0 < 0 or i1 > nx or j1 > ny or not free[i0:i1, j0:j1].all():
                continue
            free[max(i0 - mg, 0):i1 + mg, max(j0 - mg, 0):j1 + mg] = False
            cnt[o["class"]] = cnt.get(o["class"], 0) + 1
            placed.append({"class": o["class"], "class_id": o["class_id"], "movable": True, "name": "%s_%02d" % (o["class"].replace(" ", "_"), cnt[o["class"]]),
                           "min": [round(c[0] - sx / 2, 3), round(c[1] - sy / 2, 3), o["min"][2]], "max": [round(c[0] + sx / 2, 3), round(c[1] + sy / 2, 3), o["max"][2]]})
            break
    usd = header("Lab hall, random furniture layout, seed %d (randomize_layout.py)" % seed, ["../hall_static.usda"]) + ['def Xform "World"', '{'] + cube_group("Movable", placed) + ['}', '']
    open(os.path.join(a.scene, "layouts", "layout_%d.usda" % seed), "w").write("\n".join(usd))
    # test points: the scene's own, with the movable one replaced by an object of this layout
    tests = [t for t in rep["test_points"] if not t["what"].startswith("movable box")]
    big = [o for o in placed if o["class"] in ("table", "chair") and min(o["max"][0] - o["min"][0], o["max"][1] - o["min"][1]) >= 0.45]
    if big:
        o = max(big, key=lambda o: (o["max"][0] - o["min"][0]) * (o["max"][1] - o["min"][1]))
        tests.append({"what": "movable box " + o["name"], "xy": [round((o["min"][0] + o["max"][0]) / 2, 2), round((o["min"][1] + o["max"][1]) / 2, 2)], "surface_z": o["max"][2]})
    per_class = dict(rep["objects_per_class"])
    for cn in rep["movable_classes"]:
        if cn in per_class:
            per_class[cn] = cnt.get(cn, 0)
    lay = {"seed": seed, "static_layer": "../hall_static.usda", "hall_box_scene_m": rep["hall_box_scene_m"], "spawn_xy": rep["spawn_xy"], "look_from_start": rep["look_from_start"],
           "movable_wanted": n_use, "movable_placed": len(placed), "objects_per_class": {k: v for k, v in per_class.items() if v}, "test_points": tests, "objects": fixed + placed}
    json.dump(lay, open(os.path.join(a.scene, "layouts", "layout_%d.json" % seed), "w"), indent=1)
    layouts.append(lay)
    print("seed %d: placed %d of %d movable objects %s" % (seed, len(placed), n_use, cnt), flush=True)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
panels = [("as recorded on 2026-11-02 (%d movable)" % len(movable), rep["objects"])] + [("seed %d (%d movable)" % (l["seed"], l["movable_placed"]), l["objects"]) for l in layouts]
ncol = 3; nrow = int(np.ceil(len(panels) / ncol))
fig, axs = plt.subplots(nrow, ncol, figsize=(7 * ncol, 5.3 * nrow), squeeze=False)
ext = [lo[0], lo[0] + nx * cell, lo[1], lo[1] + ny * cell]
for ax, (title, objs) in zip(axs.ravel(), panels):
    ax.imshow(np.where(sp["placeable"], 1.0, np.nan).T, origin="lower", extent=ext, cmap="Greens", vmin=0, vmax=5)
    ax.imshow(np.where(sp["static"], sp["static_height"], np.nan).T, origin="lower", extent=ext, cmap="Greys", vmin=-0.8, vmax=2.4)
    for o in objs:
        col = PAL[o["class_id"] % len(PAL)]
        ax.add_patch(Rectangle(o["min"][:2], o["max"][0] - o["min"][0], o["max"][1] - o["min"][1], fc=(*col, 0.9 if o["movable"] else 0.25), ec="k" if o["movable"] else col, lw=0.6))
    if rep["spawn_xy"]:
        ax.plot(*rep["spawn_xy"], "m^", ms=7)
    ax.set_title(title, fontsize=11); ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
for ax in axs.ravel()[len(panels):]:
    ax.axis("off")
fig.suptitle("Furniture layouts: movable objects in full colour with black edge, fixed objects pale, static columns grey, placement area green, triangle = spawn spot", fontsize=12)
plt.tight_layout(); plt.savefig(os.path.join(a.scene, "layouts_topdown.png"), dpi=70)
