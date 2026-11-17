#!/usr/bin/env python3
"""Turn the labelled LiDAR map into a static USD scene for Isaac Sim / Isaac Lab (collision geometry, not a pretty mesh).

    python3 build_isaac_scene.py <labelled_map.ply> <out_dir> [--traj trajectory.csv] [--cams cameras.npz] [--movable chair table ...]

Scene frame: walls axis-aligned (the map is yawed ~30 deg in DLIO's odom frame), long side of the hall along x, floor
flattened to z = 0 (a plane is fitted to the floor, which removes the map's slow height drift), origin = map origin
(the robot's start), metres, z up. The map -> scene transform is written to scene_report.json.
The scene is split into what stays and what gets moved around:
  hall_static.usda    /World/Ground  one quad, collider
                           /World/Static  everything without an object label, as columns: every 10 cm cell that has
                                          points between 0.15 and 2.2 m gets a box from the floor up to its highest point
                                          (conservative: the space under a table top is filled). Cells on the robot's /
                                          the camera's own path are kept free.
                           /World/Fixed   one box per labelled object of a class that stays put (window, door, ...)
  hall_scene.usda     the static layer + /World/Movable = the movable classes (--movable) where they stood on the
                           day of the recording. A blob of chairs is cut into chair-sized boxes.
  layout_space.npz         top-down grids for randomize_layout.py: where a movable object may be put.
All boxes are colliders with the class in the prim name and in `semantics:labels:class`. People and the robot itself
are left out. Also writes scene_report.json and scene_topdown.png.
"""
import argparse, json, os
import numpy as np
from scipy import ndimage
from sklearn.cluster import DBSCAN
from scene_usd import PAL, cube_group, f3, header

ap = argparse.ArgumentParser()
ap.add_argument("ply"); ap.add_argument("out")
ap.add_argument("--traj", help="trajectory CSV of the LiDAR walk (t,x,y,z,...): its cells are kept free")
ap.add_argument("--cams", help="cameras.npz of the video: the trusted camera path is kept free")
ap.add_argument("--movable", nargs="*", default=["chair", "table", "counter", "tv screen", "plant", "banner", "lectern"], help="classes that get moved around (counters and tv screens are on wheels)")
ap.add_argument("--part-margin", type=float, default=0.3, help="short columns this close to a movable object count as part of it (m)")
ap.add_argument("--chair-unit", type=float, default=0.6, help="a chair cluster is cut into boxes of about this size (m)")
ap.add_argument("--cell", type=float, default=0.1); ap.add_argument("--hmin", type=float, default=0.15); ap.add_argument("--hmax", type=float, default=2.2)
ap.add_argument("--min-voxels", type=int, default=4, help="4 cm voxels a column needs to count as occupied")
ap.add_argument("--clear", type=float, default=0.35, help="radius kept free around the walked paths (m)")
ap.add_argument("--min-points", type=int, default=40, help="smallest object, in 5 cm voxels")
ap.add_argument("--wall-peak", type=float, default=0.3, help="an outer wall = a histogram peak of at least this share of the strongest one")
ap.add_argument("--confirm", help="labelled map of a second walk in the same frame: a static column must also be occupied there (glass reflections, people and moved things do not repeat)")
ap.add_argument("--confirm-slop", type=int, default=1, help="cells of alignment slop when confirming with the second map")
ap.add_argument("--confirm-grow", type=int, default=1, help="unconfirmed cells this close to a confirmed one are kept (wall continuity)")
ap.add_argument("--reachable", action="store_true", help="keep only the floor the robot can reach from its walked path without crossing a static column or a fixed object (glass partitions block), plus the columns that face it; what lies beyond the glass is dropped")
ap.add_argument("--seed-traj", nargs="*", default=[], help="more trajectory CSVs (same frame) as seeds for --reachable")
ap.add_argument("--reach-close", type=int, default=5, help="--reachable: close wall gaps up to this many cells before the flood fill, then thicken the barrier by one cell (0 = off)")
ap.add_argument("--edit", help="json with decisions made by a person who knows the hall: drop_boxes [[x0,y0,x1,y1],...] (static cells inside go), rooms {y_below, x:[x0,x1], glass_height} (everything below y_below is glass-walled rooms the robot never enters: dropped, a thin glass wall is added along y_below), add_objects [{class,min,max,movable}], chair_size [sx,sy] (every chair box gets this footprint about its centre)")
ap.add_argument("--drop-islands", action="store_true", help="with --reachable: free-standing static blobs inside the hall (unlabelled furniture) leave the static layer; written to static_islands_dropped.json")
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)

# boxes of these classes go down to the floor; the tv screens stand on wheeled stands, so they are floor-standing too
FLOOR_STANDING = {"chair", "table", "plant", "cabinet", "counter", "stage", "shelf", "refrigerator", "lectern", "stairs", "tv screen"}
SKIP = {"person", "humanoid robot"}  # not part of the static room

def load_labelled(path):
    """x y z intensity class r g b PLY from project_labels.py -> xyz, class, class names."""
    with open(path, "rb") as f:
        names = None
        while True:
            l = f.readline()
            if l.startswith(b"comment classes:"):
                names = [s.split("=", 1)[1] for s in l.decode()[len("comment classes:"):].strip().split(", ")]
            if l.startswith(b"element vertex"):
                n = int(l.split()[-1])
            if l.strip() == b"end_header":
                break
        d = np.fromfile(f, dtype=[("xyz", "<f4", 3), ("i", "<f4"), ("c", "u1"), ("rgb", "u1", 3)], count=n)
    return d["xyz"].astype(np.float64), d["c"], names

def rot2(th):
    return np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])

X, label, classes = load_labelled(a.ply)

# ---- floor: height mode, then a plane through the low points of 0.5 m cells
# (lower bound from a percentile, not the minimum: a few stray points far below the floor would otherwise shift the window)
zlo = np.percentile(X[:, 2], 0.5)
hist, edges = np.histogram(X[:, 2], bins=np.arange(zlo, zlo + 2.0, 0.02))
z0 = edges[hist.argmax()] + 0.01
low = X[np.abs(X[:, 2] - z0) < 0.35]
key = np.floor(low[:, :2] / 0.5).astype(np.int64)
order = np.lexsort((key[:, 1], key[:, 0])); key, low = key[order], low[order]
start = np.r_[0, np.where((np.diff(key, axis=0) != 0).any(1))[0] + 1, len(key)]
cells = np.array([[*low[s:e, :2].mean(0), np.percentile(low[s:e, 2], 20)] for s, e in zip(start[:-1], start[1:]) if e - s >= 30])
m = np.ones(len(cells), bool)
for _ in range(4):
    coef = np.linalg.lstsq(np.c_[cells[m, :2], np.ones(m.sum())], cells[m, 2], rcond=None)[0]
    m = np.abs(np.c_[cells[:, :2], np.ones(len(cells))] @ coef - cells[:, 2]) < 0.06
tilt = float(np.degrees(np.arctan(np.hypot(coef[0], coef[1]))))
h = X[:, 2] - (X[:, :2] @ coef[:2] + coef[2])
print("floor: mode z %.2f, plane tilt %.2f deg (height change over 20 m: %.2f m), %d of %d cells on the plane" % (z0, tilt, 20 * np.tan(np.radians(tilt)), m.sum(), len(cells)))

# ---- yaw: the angle at which the wall points pile up most sharply on the x and y axes
band = (h > 0.3) & (h < a.hmax)
B = np.unique(np.floor(X[band][:, :2] / 0.04).astype(np.int32), axis=0) * 0.04
def sharp(th):
    r = B @ rot2(-th).T
    return sum((np.bincount(np.floor((r[:, k] - r[:, k].min()) / 0.05).astype(int)).astype(float) ** 2).sum() for k in (0, 1))
ths = np.radians(np.arange(0, 90, 0.25))
yaw = ths[int(np.argmax([sharp(t) for t in ths]))]
r = B @ rot2(-yaw).T
if np.ptp(np.percentile(r[:, 1], [2, 98])) > np.ptp(np.percentile(r[:, 0], [2, 98])):
    yaw += np.pi / 2  # long side along x
print("yaw of the hall in the map frame: %.2f deg" % np.degrees(yaw))
def to_scene(P):
    """map frame -> scene frame (xyz or xy rows)."""
    out = np.array(P, dtype=float)
    out[:, :2] = P[:, :2] @ rot2(-yaw).T
    if out.shape[1] > 2:
        out[:, 2] = P[:, 2] - (P[:, :2] @ coef[:2] + coef[2])
    return out
S = to_scene(X)

# ---- crop to the hall: start from the box around what is known to be inside (floor the LiDAR saw, labelled objects,
# the walked paths), then move each side out to the outermost wall within 3 m. With the walls on the axes a wall is a
# strong peak in the x or y histogram of the wall-height points; what lies beyond was seen through glass.
paths = []
if a.traj:
    paths.append(to_scene(np.loadtxt(a.traj, delimiter=",", skiprows=1, usecols=(1, 2))[::20]))
if a.cams:
    c = np.load(a.cams)
    paths.append(to_scene(c["center"][c["trusted"]][:, :2]))
known = [S[np.abs(S[:, 2]) < 0.08][:, :2], S[(label > 0) & ~np.isin(label, [i for i, c in enumerate(classes) if c in SKIP])][:, :2]] + paths
lo = np.min([np.percentile(k, 0.5, axis=0) for k in known if len(k)], axis=0)
hi = np.max([np.percentile(k, 99.5, axis=0) for k in known if len(k)], axis=0)
Bs = to_scene(B)
for k in (0, 1):
    hk, ek = np.histogram(Bs[:, k], bins=np.arange(Bs[:, k].min(), Bs[:, k].max() + 0.1, 0.1))
    walls = ek[np.where(hk >= a.wall_peak * hk.max())[0]] + 0.05
    below, above = walls[(walls < lo[k]) & (walls > lo[k] - 3.0)], walls[(walls > hi[k]) & (walls < hi[k] + 3.0)]
    lo[k] = (below.min() if len(below) else lo[k]) - 0.5
    hi[k] = (above.max() if len(above) else hi[k]) + 0.5
inside = ((S[:, :2] >= lo) & (S[:, :2] <= hi)).all(1)
nx, ny = np.ceil((hi - lo) / a.cell).astype(int)
print("hall box in the scene frame: x %.1f..%.1f, y %.1f..%.1f (%.1f x %.1f m), %d of %d points inside" % (lo[0], hi[0], lo[1], hi[1], *(hi - lo), inside.sum(), len(S)))

def cells_of(o):
    """Grid slice covered by an object's footprint."""
    i0, j0 = np.floor((np.array(o["min"][:2]) - lo) / a.cell).astype(int)
    i1, j1 = np.ceil((np.array(o["max"][:2]) - lo) / a.cell).astype(int)
    return slice(max(i0, 0), min(i1, nx)), slice(max(j0, 0), min(j1, ny))

# ---- objects: cluster each class in the scene frame, one box per cluster
objects = []
for ci_, cn in enumerate(classes):
    if ci_ == 0 or cn in SKIP:
        continue
    P = S[inside & (label == ci_)]
    if len(P) < a.min_points:
        continue
    v = (np.unique(np.floor(P / 0.05).astype(np.int32), axis=0) + 0.5) * 0.05
    cl = DBSCAN(eps=0.12, min_samples=6).fit_predict(v)
    for j in range(cl.max() + 1):
        Q = v[cl == j]
        if len(Q) < a.min_points:
            continue
        b0, b1 = np.percentile(Q, 2, axis=0), np.percentile(Q, 98, axis=0)
        if cn in FLOOR_STANDING:
            b0[2] = 0.0  # labels near the floor were dropped, the object still stands on it
        b1 = np.maximum(b1, b0 + 0.05)
        # chairs standing together come out as one blob: cut it into chair-sized boxes, keep those that hold points
        n = np.maximum(1, np.round((b1[:2] - b0[:2]) / a.chair_unit).astype(int)) if cn == "chair" else np.array([1, 1])
        step = (b1[:2] - b0[:2]) / n
        gap = 0.03 if n.prod() > 1 else 0.0
        for i in range(n[0]):
            for k in range(n[1]):
                c0, c1 = b0.copy(), b1.copy()
                c0[:2] = b0[:2] + step * [i, k] + gap
                c1[:2] = b0[:2] + step * [i + 1, k + 1] - gap
                nq = int(((Q[:, :2] >= c0[:2]) & (Q[:, :2] <= c1[:2])).all(1).sum())
                if nq >= (8 if n.prod() > 1 else 1):
                    objects.append({"class": cn, "class_id": ci_, "movable": cn in a.movable, "split_from_cluster": bool(n.prod() > 1),
                                    "min": c0.round(3).tolist(), "max": c1.round(3).tolist(), "voxels_5cm": nq})
edit = json.load(open(a.edit)) if a.edit else {}
if edit.get("chair_size"):
    sx, sy = edit["chair_size"]
    for o in objects:
        if o["class"] == "chair":
            cx, cy = (o["min"][0] + o["max"][0]) / 2, (o["min"][1] + o["max"][1]) / 2
            o["min"][:2] = [round(cx - sx / 2, 3), round(cy - sy / 2, 3)]; o["max"][:2] = [round(cx + sx / 2, 3), round(cy + sy / 2, 3)]
    print("edit: %d chair boxes set to %.2f x %.2f m" % (sum(o["class"] == "chair" for o in objects), sx, sy))
for o in edit.get("add_objects", []):
    objects.append({"class": o["class"], "class_id": classes.index(o["class"]) if o["class"] in classes else -1, "movable": o.get("movable", o["class"] in a.movable),
                    "split_from_cluster": False, "min": o["min"], "max": o["max"], "voxels_5cm": 0, "added_by_hand": True})
if edit.get("add_objects"):
    print("edit: %d objects added by hand" % len(edit["add_objects"]))
objects.sort(key=lambda o: (o["class"], -o["voxels_5cm"]))
cnts = {}
for o in objects:
    cnts[o["class"]] = cnts.get(o["class"], 0) + 1
    o["name"] = "%s_%02d" % (o["class"].replace(" ", "_"), cnts[o["class"]])
if edit.get("drop_objects"):
    n0 = len(objects); objects = [o for o in objects if o["name"] not in set(edit["drop_objects"])]
    print("edit: %d labelled objects dropped by name" % (n0 - len(objects)))
fixed, movable = [o for o in objects if not o["movable"]], [o for o in objects if o["movable"]]
print("objects:", cnts, "| fixed %d, movable %d" % (len(fixed), len(movable)))

# ---- static columns from the points without an object label
st = S[inside & (label == 0) & (S[:, 2] > a.hmin) & (S[:, 2] < a.hmax)]
vox = np.unique(np.floor(st / 0.04).astype(np.int32), axis=0)
ci = np.floor((vox[:, :2] * 0.04 + 0.02 - lo) / a.cell).astype(int)
ok = (ci >= 0).all(1) & (ci < [nx, ny]).all(1)
ci, vz = ci[ok], vox[ok, 2] * 0.04 + 0.04
cnt = np.zeros((nx, ny), int); np.add.at(cnt, (ci[:, 0], ci[:, 1]), 1)
top = np.zeros((nx, ny)); np.maximum.at(top, (ci[:, 0], ci[:, 1]), vz)
occ = cnt >= a.min_voxels
# a second walk of the same hall: glass reflections, people and things that were moved do not repeat, walls do
n_unconfirmed = 0
if a.confirm:
    X2, label2, classes2 = load_labelled(a.confirm)
    S2 = to_scene(X2)
    st2 = S2[(label2 == 0) & (S2[:, 2] > a.hmin) & (S2[:, 2] < a.hmax)]
    vox2 = np.unique(np.floor(st2 / 0.04).astype(np.int32), axis=0)
    ci2 = np.floor((vox2[:, :2] * 0.04 + 0.02 - lo) / a.cell).astype(int)
    ok2 = (ci2 >= 0).all(1) & (ci2 < [nx, ny]).all(1)
    cnt2 = np.zeros((nx, ny), int); np.add.at(cnt2, (ci2[ok2, 0], ci2[ok2, 1]), 1)
    occ2 = ndimage.binary_dilation(cnt2 >= a.min_voxels, structure=np.ones((3, 3)), iterations=a.confirm_slop)
    confirmed = occ & occ2
    if a.confirm_grow:
        confirmed |= occ & ndimage.binary_dilation(confirmed, structure=np.ones((3, 3)), iterations=a.confirm_grow)
    n_unconfirmed = int((occ & ~confirmed).sum()); occ = confirmed
    print("confirm: %d of %d occupied cells kept (%d not seen in %s)" % (occ.sum(), occ.sum() + n_unconfirmed, n_unconfirmed, os.path.basename(a.confirm)))
# free the cells the robot and the camera went through (people and the robot itself leave points along the walk)
n_clear = 0
if paths:
    walked = np.zeros((nx, ny), bool)
    pj = np.floor((np.vstack(paths) - lo) / a.cell).astype(int)
    pj = pj[(pj >= 0).all(1) & (pj < [nx, ny]).all(1)]
    walked[pj[:, 0], pj[:, 1]] = True
    rr = int(round(a.clear / a.cell))
    yy, xx = np.ogrid[-rr:rr + 1, -rr:rr + 1]
    walked = ndimage.binary_dilation(walked, structure=(xx ** 2 + yy ** 2 <= rr ** 2))
    n_clear = int((occ & walked).sum()); occ &= ~walked
# unlabelled leftovers of a movable object (its feet, the side the camera did not see) would stay behind when it is
# moved: every column inside its footprint belongs to it, whatever its height (points above a chair come from people,
# the robot or the wall behind, and a static column inside a chair is wrong either way); within --part-margin of it,
# only columns that are not taller than it
n_part = 0
for o in movable:
    sl = cells_of({"min": [o["min"][0] - a.part_margin, o["min"][1] - a.part_margin], "max": [o["max"][0] + a.part_margin, o["max"][1] + a.part_margin]})
    part = occ[sl] & (top[sl] <= o["max"][2] + 0.2)
    n_part += int(part.sum()); occ[sl] &= ~part
    sl = cells_of(o)
    n_part += int(occ[sl].sum()); occ[sl] = False
lab_, n = ndimage.label(occ, structure=np.ones((3, 3)))
small = occ & np.isin(lab_, np.where(ndimage.sum(occ, lab_, np.arange(n + 1)) < 3)[0])  # specks
occ &= ~small
# the robot's side of the glass: flood-fill the free cells from the walked paths; static columns and fixed objects
# (doors, windows, glass fronts) are barriers. Columns not touching the reachable floor are beyond the glass or ghosts.
n_beyond = 0
if a.reachable:
    barrier = occ.copy()
    for o in fixed:
        barrier[cells_of(o)] = True
    barrier_raw = barrier.copy()
    # LiDAR walls have gaps (glass, thin partitions): close them before filling so the fill does not leak into the rooms
    if a.reach_close:
        barrier = ndimage.binary_closing(barrier, structure=np.ones((a.reach_close, a.reach_close)), border_value=1) | barrier
        barrier = ndimage.binary_dilation(barrier, structure=np.ones((3, 3)))
    seeds = list(paths)
    for p in a.seed_traj:
        seeds.append(to_scene(np.loadtxt(p, delimiter=",", skiprows=1, usecols=(1, 2))[::20]))
    sd = np.floor((np.vstack(seeds) - lo) / a.cell).astype(int)
    sd = sd[(sd >= 0).all(1) & (sd < [nx, ny]).all(1)]
    free_lab, nfree = ndimage.label(~barrier, structure=np.ones((3, 3)))
    ids = np.unique(free_lab[sd[:, 0], sd[:, 1]]); ids = ids[ids > 0]
    reach = np.isin(free_lab, ids)
    rd = 2 if a.reach_close else 1  # the thickened barrier keeps the reachable floor one cell away from the columns
    facing = occ & ndimage.binary_dilation(reach, structure=np.ones((3, 3)), iterations=rd)
    n_beyond = int((occ & ~facing).sum()); occ = facing
    print("reachable: %d free cells (%.0f m2) from %d seed cells in %d region(s); %d static cells beyond the walls/glass dropped, %d kept" % (reach.sum(), reach.sum() * a.cell ** 2, len(sd), len(ids), n_beyond, occ.sum()))
    if a.drop_islands:
        # a wall has unreachable space behind it; a free-standing island inside the hall (furniture nobody labelled,
        # people) has reachable floor all around. Islands leave the static layer and are written out for labelling.
        behind = ~reach & ~barrier  # unreachable floor beyond the (thickened) barrier; the ring around an island is barrier, not 'behind'
        border = np.zeros_like(occ); border[[0, -1], :] = True; border[:, [0, -1]] = True
        wall = occ & (ndimage.binary_dilation(behind, structure=np.ones((3, 3)), iterations=rd + 1) | border)
        isl = occ & ~wall
        il, ni = ndimage.label(isl, structure=np.ones((3, 3)))
        islands = []
        for k in range(1, ni + 1):
            ii, jj = np.where(il == k)
            islands.append({"min": [float(lo[0] + ii.min() * a.cell), float(lo[1] + jj.min() * a.cell)], "max": [float(lo[0] + (ii.max() + 1) * a.cell), float(lo[1] + (jj.max() + 1) * a.cell)],
                            "cells": int(len(ii)), "top_m": float(top[ii, jj].max())})
        json.dump(islands, open(os.path.join(a.out, "static_islands_dropped.json"), "w"), indent=1)
        print("islands: %d cells in %d free-standing blobs inside the hall dropped from the static layer (static_islands_dropped.json); %d wall cells kept" % (isl.sum(), ni, wall.sum()))
        occ = wall
# decisions by a person who knows the hall (--edit)
glass_boxes = []
if edit.get("drop_boxes") or edit.get("rooms") or edit.get("glass_lines"):
    cx = lo[0] + (np.arange(nx) + 0.5) * a.cell; cy = lo[1] + (np.arange(ny) + 0.5) * a.cell
    CX, CY = np.meshgrid(cx, cy, indexing="ij")
    keep = np.zeros_like(occ)   # cells a person marked as real: never dropped by the rules below
    for x0, x1, y0, y1 in edit.get("keep_boxes", []):
        keep |= (CX > x0) & (CX < x1) & (CY > y0) & (CY < y1)
    n_drop = 0
    for x0, y0, x1, y1 in edit.get("drop_boxes", []):
        m = occ & ~keep & (CX > x0) & (CX < x1) & (CY > y0) & (CY < y1)
        n_drop += int(m.sum()); occ &= ~m
    # glass lines from the Twin Fixer: {a, b, drop_side, height}. Cells on the drop side, more than 0.3 m from the line and
    # within the segment's extent, go; a thin glass wall is added along the line (as one box per 0.5 m for slanted lines)
    n_glass = 0
    for g in edit.get("glass_lines", []):
        (ax, ay), (bx, by), (sx, sy) = g["a"], g["b"], g["drop_side"]
        dx, dy = bx - ax, by - ay; L = np.hypot(dx, dy) or 1.0
        side = lambda X, Y: ((X - ax) * dy - (Y - ay) * dx) / L
        s0 = np.sign(side(sx, sy))
        t = ((CX - ax) * dx + (CY - ay) * dy) / (L * L)
        m = occ & ~keep & (t > -0.05) & (t < 1.05) & (np.sign(side(CX, CY)) == s0) & (np.abs(side(CX, CY)) > 0.3)
        n_glass += int(m.sum()); occ &= ~m
        h = g.get("height", 2.2); n_seg = max(1, int(np.ceil(L / 0.5)))
        for k in range(n_seg):
            p0 = np.array([ax, ay]) + np.array([dx, dy]) * k / n_seg; p1 = np.array([ax, ay]) + np.array([dx, dy]) * (k + 1) / n_seg
            glass_boxes.append([min(p0[0], p1[0]) - 0.05, max(p0[0], p1[0]) + 0.05, min(p0[1], p1[1]) - 0.05, max(p0[1], p1[1]) + 0.05, 0.0, h])
    if edit.get("glass_lines"):
        print("edit: %d static cells dropped behind %d glass line(s), %d glass boxes added" % (n_glass, len(edit["glass_lines"]), len(glass_boxes)))
    rooms = edit.get("rooms")
    n_rooms = 0
    if rooms:
        yb = rooms["y_below"]; xr = rooms.get("x", [-1e9, 1e9])
        m = occ & (CY < yb - 0.25) & (CX > xr[0]) & (CX < xr[1])   # the rooms; the 25 cm band at the glass stays
        n_rooms = int(m.sum()); occ &= ~m
        glass_boxes.append([xr[0], xr[1], yb - 0.1, yb, 0.0, rooms.get("glass_height", 2.2)])
    print("edit: %d static cells dropped in drop_boxes, %d dropped inside the rooms, %d glass wall(s) added" % (n_drop, n_rooms, len(glass_boxes)))
Hq = np.where(occ, np.ceil(top / 0.1).astype(int), 0)  # column height in 0.1 m steps
print("static: %d occupied cells of %d (%d freed on the walked paths, %d inside movable objects, %d specks dropped)" % (occ.sum(), nx * ny, n_clear, n_part, small.sum()))

# merge equal-height cells into boxes: runs along y, then identical runs of neighbouring x rows
boxes, open_ = [], {}
for i in range(nx + 1):
    runs = {}
    if i < nx:
        row = Hq[i]; j = 0
        while j < ny:
            if row[j]:
                j0 = j
                while j < ny and row[j] == row[j0]:
                    j += 1
                runs[(j0, j, row[j0])] = True
            else:
                j += 1
    for k_, i0 in list(open_.items()):
        if k_ not in runs:
            boxes.append((i0, i, k_[0], k_[1], k_[2])); del open_[k_]
    for k_ in runs:
        open_.setdefault(k_, i)
static = np.array([[lo[0] + i0 * a.cell, lo[0] + i1 * a.cell, lo[1] + j0 * a.cell, lo[1] + j1 * a.cell, 0.0, hq * 0.1] for i0, i1, j0, j1, hq in boxes] + glass_boxes)
print("static: %d boxes" % len(static))

# ---- where a movable object may be put: floor the LiDAR saw (plus where the movable objects stand now), away from
# everything that stays
fl = np.floor((S[inside & (np.abs(S[:, 2]) < 0.08)][:, :2] - lo) / a.cell).astype(int)
fl = fl[(fl >= 0).all(1) & (fl < [nx, ny]).all(1)]
fcnt = np.zeros((nx, ny), int); np.add.at(fcnt, (fl[:, 0], fl[:, 1]), 1)
floor_seen = fcnt >= 2
blocked = occ.copy()
for o in fixed:
    blocked[cells_of(o)] = True
for o in movable:
    floor_seen[cells_of(o)] = True
floor_seen = ndimage.binary_closing(floor_seen, iterations=2)
placeable = floor_seen & ~ndimage.binary_dilation(blocked, iterations=1)
print("layout space: %.0f m2 of floor seen, %.0f m2 placeable" % (floor_seen.sum() * a.cell ** 2, placeable.sum() * a.cell ** 2))

# ---- test points for check_isaac_scene.py: a ball dropped there has to rest on that surface
taken = blocked.copy()
for o in movable:
    taken[cells_of(o)] = True
dist = ndimage.distance_transform_edt(~taken) * a.cell
tests, spawn = [], None
if paths:
    pj = np.floor((np.vstack(paths) - lo) / a.cell).astype(int)
    pj = pj[(pj >= 0).all(1) & (pj < [nx, ny]).all(1)]
    b = pj[dist[pj[:, 0], pj[:, 1]].argmax()]
    spawn = (lo + (b + 0.5) * a.cell).round(2).tolist()
    tests.append({"what": "free floor", "xy": spawn, "surface_z": 0.0})
def clear_above(o):
    """Nothing else (another object box, a static column) over the middle of this box, where the ball is dropped."""
    c = np.array([(o["min"][0] + o["max"][0]) / 2, (o["min"][1] + o["max"][1]) / 2])
    i, j = np.floor((c - lo) / a.cell).astype(int)
    return not occ[max(i - 2, 0):i + 3, max(j - 2, 0):j + 3].any() and not any(
        q is not o and (np.array(q["min"][:2]) - 0.2 <= c).all() and (c <= np.array(q["max"][:2]) + 0.2).all() for q in objects)
for grp, nm in ((fixed, "fixed box "), (movable, "movable box ")):
    big = [o for o in grp if o["class"] in ("counter", "table", "chair") and min(o["max"][0] - o["min"][0], o["max"][1] - o["min"][1]) >= 0.45 and clear_above(o)]
    if big:
        o = max(big, key=lambda o: o["voxels_5cm"])
        tests.append({"what": nm + o["name"], "xy": [round((o["min"][0] + o["max"][0]) / 2, 2), round((o["min"][1] + o["max"][1]) / 2, 2)], "surface_z": o["max"][2]})
blocks = [s_ for s_ in static if min(s_[1] - s_[0], s_[3] - s_[2]) >= 0.3 and 0.3 <= s_[5] <= 1.5]
if blocks:
    s_ = max(blocks, key=lambda s_: (s_[1] - s_[0]) * (s_[3] - s_[2]))
    tests.append({"what": "static block %.1f m high" % s_[5], "xy": [round((s_[0] + s_[1]) / 2, 2), round((s_[2] + s_[3]) / 2, 2)], "surface_z": float(s_[5])})
chairs = [o for o in objects if o["class"] == "chair"]
look = np.mean([[(o["min"][0] + o["max"][0]) / 2, (o["min"][1] + o["max"][1]) / 2] for o in chairs], axis=0).round(2).tolist() if chairs else ((lo + hi) / 2).round(2).tolist()

# ---- USD (ASCII): static layer, and the layout of the recording day on top of it
pts, counts, idx = [], [], []
for x0, x1, y0, y1, z0_, z1 in static:
    b = len(pts)
    pts += [(x0, y0, z0_), (x1, y0, z0_), (x1, y1, z0_), (x0, y1, z0_), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    for q in ((4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)):  # top + 4 sides, normals outwards
        counts.append(4); idx += [b + k for k in q]
g0, g1 = lo - 1.0, hi + 1.0
usd = header("Lab hall, what stays: static collision columns from the G1 LiDAR map of 2026-11-02 + fixed labelled objects (build_isaac_scene.py)") + [
    'def Xform "World"', '{',
    '    def Mesh "Ground" (prepend apiSchemas = ["PhysicsCollisionAPI"])', '    {',
    '        int[] faceVertexCounts = [4]', '        int[] faceVertexIndices = [0, 1, 2, 3]',
    '        point3f[] points = [%s]' % ", ".join(f3(p) for p in ((g0[0], g0[1], 0), (g1[0], g0[1], 0), (g1[0], g1[1], 0), (g0[0], g1[1], 0))),
    '        color3f[] primvars:displayColor = [(0.32, 0.33, 0.35)]', '        uniform token subdivisionScheme = "none"', '    }', '',
    '    def Mesh "Static" (prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"])', '    {',
    '        uniform token physics:approximation = "none"',
    '        int[] faceVertexCounts = [%s]' % ", ".join(map(str, counts)),
    '        int[] faceVertexIndices = [%s]' % ", ".join(map(str, idx)),
    '        point3f[] points = [%s]' % ", ".join(f3(p) for p in pts),
    '        color3f[] primvars:displayColor = [(0.72, 0.72, 0.70)]', '        uniform token subdivisionScheme = "none"', '    }', ''] + cube_group("Fixed", fixed) + ['}', '']
open(os.path.join(a.out, "hall_static.usda"), "w").write("\n".join(usd))
usd = header("Lab hall as recorded on 2026-11-02: static layer + the movable objects where they stood (build_isaac_scene.py)", ["./hall_static.usda"]) + [
    'def Xform "World"', '{'] + cube_group("Movable", movable) + ['}', '']
open(os.path.join(a.out, "hall_scene.usda"), "w").write("\n".join(usd))
np.savez_compressed(os.path.join(a.out, "layout_space.npz"), lo=lo, hi=hi, cell=a.cell, placeable=placeable, blocked=blocked, static=occ, static_height=Hq * 0.1)

report = {"source": a.ply, "frame": {"yaw_map_to_scene_deg": float(-np.degrees(yaw)), "floor_plane_map_z=ax+by+c": [float(c) for c in coef], "floor_tilt_deg": tilt,
                                     "note": "scene_xy = R(yaw_map_to_scene) * map_xy, scene_z = map_z - (a*map_x + b*map_y + c); origin = map origin"},
          "hall_box_scene_m": {"min": lo.round(2).tolist(), "max": hi.round(2).tolist()}, "cell_m": a.cell, "column_height_range_m": [a.hmin, a.hmax],
          "static_cells": int(occ.sum()), "static_boxes": len(static), "static_vertices": len(pts), "cells_freed_on_paths": n_clear, "cells_counted_to_movable_objects": n_part,
          "movable_classes": a.movable, "placeable_m2": float(placeable.sum() * a.cell ** 2), "spawn_xy": spawn,
          "objects_per_class": cnts, "test_points": tests, "look_from_start": look, "objects": objects}
json.dump(report, open(os.path.join(a.out, "scene_report.json"), "w"), indent=1)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
fig, ax = plt.subplots(figsize=(17, 11))
ext = [lo[0], lo[0] + nx * a.cell, lo[1], lo[1] + ny * a.cell]
ax.imshow(np.where(placeable, 1.0, np.nan).T, origin="lower", extent=ext, cmap="Greens", vmin=0, vmax=4)
ax.imshow(np.where(occ, Hq * 0.1, np.nan).T, origin="lower", extent=ext, cmap="Greys", vmin=-0.8, vmax=2.4)
for p, st_, nm in zip(paths, ("b-", "c-"), ("LiDAR walk (kept free)", "video camera, trusted part (kept free)")):
    ax.plot(p[:, 0], p[:, 1], st_, lw=0.8, label=nm)
seen = set()
for o in objects:
    col = PAL[o["class_id"] % len(PAL)]
    ax.add_patch(Rectangle(o["min"][:2], o["max"][0] - o["min"][0], o["max"][1] - o["min"][1], fc=(*col, 0.55), ec="k" if o["movable"] else col, lw=1, ls="--" if o["movable"] else "-",
                           label=None if o["class"] in seen else "%s (%d)%s" % (o["class"], cnts[o["class"]], ", movable" if o["movable"] else "")))
    seen.add(o["class"])
ax.plot(0, 0, "r*", ms=12, label="origin = robot start")
if spawn:
    ax.plot(*spawn, "m^", ms=9, label="open spot (spawn / drop test)")
ax.set_aspect("equal"); ax.legend(fontsize=8, loc="upper left", bbox_to_anchor=(1.0, 1.0)); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title("Isaac scene, top view: grey = static columns (darker = higher), solid = fixed objects, dashed = movable objects, green = where movable objects may be put")
plt.tight_layout(); plt.savefig(os.path.join(a.out, "scene_topdown.png"), dpi=75)
print("wrote", os.path.join(a.out, "hall_static.usda"), "%.1f MB" % (os.path.getsize(os.path.join(a.out, "hall_static.usda")) / 1e6), "+ hall_scene.usda, layout_space.npz")
