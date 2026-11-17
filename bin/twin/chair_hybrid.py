#!/usr/bin/env python3
"""Chair asset from a mesh_from_points.py mesh: scanned seat, back and armrests + four straight legs with castors fitted
to the scanned leg cores (thin legs are the part pose blur smears most, and the floor cut leaves them short).

    python3 chair_hybrid.py <okay2_chair.ply> <out_dir> --points <okay2_chair_points.ply> --name okay2_chair [--cut 0.40]

The mesh is mesh_from_points.py output; its _report.json (next to it) gives the turn from the raw points to the object
frame. Legs come from the raw points, since sparse legs can drop out of the mesh: in each quadrant around the seat
centre and each 2 cm slice between --leg-low and --cut, the densest 1 cm cell (smoothed) is the leg core; a line through
the cores (refit without cores > 2 cm off) runs from the seat underside down to the castor. Writes <name>.glb/.obj,
<name>_render.png (shaded, four views), <name>_report.json.
"""
import argparse
import json
import os

import cv2
import numpy as np
import open3d as o3d
import trimesh
from scipy import ndimage
from scipy.spatial import cKDTree

LEG = (32, 32, 35)

ap = argparse.ArgumentParser()
ap.add_argument("mesh"); ap.add_argument("out"); ap.add_argument("--points", required=True); ap.add_argument("--name", default="chair")
ap.add_argument("--cut", type=float, default=0.40, help="scanned surface kept above this height (m)")
ap.add_argument("--leg-low", type=float, default=0.08, help="lowest height used to fit the legs (m)")
ap.add_argument("--leg-r", type=float, default=0.012, help="leg tube radius (m)")
ap.add_argument("--castor-r", type=float, default=0.025, help="castor wheel radius (m)")
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)

m = trimesh.load(a.mesh, process=False)
keep = m.vertices[m.faces][:, :, 2].min(1) > a.cut
upper = m.copy(); upper.update_faces(keep); upper.remove_unreferenced_vertices()
parts = upper.split(only_watertight=False)
big = max(len(p.faces) for p in parts)
upper = trimesh.util.concatenate([p for p in parts if len(p.faces) > 0.02 * big])

# raw points in the object frame, as mesh_from_points.py turned the mesh
mr = json.load(open(os.path.splitext(a.mesh)[0] + "_report.json"))
P = np.asarray(o3d.io.read_point_cloud(a.points).points)
c0 = np.median(P[(P[:, 2] > mr.get("seat_band", [0.38, 0.52])[0]) & (P[:, 2] < mr.get("seat_band", [0.38, 0.52])[1]), :2], 0)
P = P[(np.linalg.norm(P[:, :2] - c0, axis=1) < 0.6) & (P[:, 2] > a.leg_low) & (P[:, 2] < a.cut - 0.02)] - [c0[0], c0[1], 0]
t = mr["yaw_to_object_rad"]
P = P @ np.array([[np.cos(t), -np.sin(t), 0], [np.sin(t), np.cos(t), 0], [0, 0, 1]]).T
P[:, :2] -= mr["seat_centre_shift_xy"]

legs, rep_legs, fits = [], [], []
scan_kd = cKDTree(upper.vertices)
for sx in (1, -1):
    for sy in (1, -1):
        Q = P[(np.sign(P[:, 0]) == sx) & (np.sign(P[:, 1]) == sy)]
        cores = []
        for z0 in np.arange(a.leg_low, a.cut - 0.02, 0.02):
            s_ = Q[(Q[:, 2] >= z0) & (Q[:, 2] < z0 + 0.02), :2]
            if len(s_) < 200:
                continue
            H, xe, ye = np.histogram2d(s_[:, 0], s_[:, 1], bins=[np.arange(-0.6, 0.61, 0.01)] * 2)
            i, j = np.unravel_index(np.argmax(ndimage.gaussian_filter(H, 1.5)), H.shape)
            cores.append([xe[i] + 0.005, ye[j] + 0.005, z0 + 0.01])
        q = np.array(cores)
        if len(q) < 4:
            print("quadrant", sx, sy, "has no leg"); continue
        for _ in range(3):
            c = q.mean(0); d = np.linalg.svd(q - c)[2][0]
            d = d if d[2] > 0 else -d
            off = np.linalg.norm(np.cross(q - c, d), axis=1)
            q = q[off < max(0.02, np.percentile(off, 80))]
        at = lambda z: c + d * (z - c[2]) / d[2]  # noqa: E731
        fits.append((sx, sy, at(a.cut + 0.03), at(2 * a.castor_r + 0.01), len(q)))
        print("leg", sx, sy, "top", np.round(fits[-1][2][:2], 3), "foot", np.round(fits[-1][3][:2], 3), len(q), "slices")

# the chair is mirror-symmetric: front legs share one shape, rear legs another (mean of the pair, y mirrored)
for sx, sy, _, _, n in fits:
    pair = [(t, f) for qx, _, t, f, _ in fits if qx == sx]
    top, foot = (np.mean([np.abs(p[k]) for p in pair], 0) * [sx, sy, 1] for k in (0, 1))
    d = (top - foot) / np.linalg.norm(top - foot)
    for z in np.arange(a.cut - 0.03, a.cut + 0.3, 0.005):  # run up the leg line until it is inside the scanned seat
        p_ = foot + d * (z - foot[2]) / d[2]
        if scan_kd.query(p_)[0] < a.leg_r:
            top = p_ + d * 0.015; break
    else:
        print("leg", sx, sy, "does not reach the scan")
    legs.append(trimesh.creation.cylinder(radius=a.leg_r, segment=[top, foot], sections=16))
    stem = trimesh.creation.cylinder(radius=0.012, segment=[foot, foot - [0, 0, 0.012]], sections=12)
    wheel = trimesh.creation.cylinder(radius=a.castor_r, height=0.022, sections=20)
    wheel.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
    wheel.apply_translation([foot[0], foot[1], a.castor_r])
    legs += [stem, wheel]
    tilt = np.degrees(np.arctan2(np.linalg.norm(top[:2] - foot[:2]), top[2] - foot[2]))
    rep_legs.append({"quadrant": [sx, sy], "top_xy": [round(float(x), 3) for x in top[:2]], "foot_xy": [round(float(x), 3) for x in foot[:2]],
                     "tilt_deg": round(float(tilt), 1), "fit_slices": int(n), "pair_averaged": len(pair) == 2})
frame = trimesh.util.concatenate(legs)
frame.visual.vertex_colors = np.tile(np.array(LEG + (255,), np.uint8), (len(frame.vertices), 1))

scene = trimesh.Scene()
scene.add_geometry(upper, geom_name="seat_back_arms_scan")
scene.add_geometry(frame, geom_name="legs_castors_fitted")
scene.export(os.path.join(a.out, a.name + ".glb"))
full = trimesh.util.concatenate([upper, frame])
full.export(os.path.join(a.out, a.name + ".obj"))

U = upper.vertices
up = upper.face_normals[:, 2] > 0.9
hz, he = np.histogram(upper.triangles_center[up, 2], bins=np.arange(a.cut, U[:, 2].max() + 0.02, 0.01), weights=upper.area_faces[up])
seat_z = float(he[np.argmax(hz)] + 0.005)
seat = U[np.abs(U[:, 2] - seat_z) < 0.02]
arm = U[(U[:, 2] > seat_z + 0.1) & (U[:, 2] < seat_z + 0.35) & (np.abs(U[:, 1]) > np.percentile(np.abs(seat[:, 1]), 95))]
ext = full.bounds[1] - full.bounds[0]
rep = {"seat_height_m": round(seat_z, 3), "top_height_m": round(float(U[:, 2].max()), 3),
       "armrest_height_m": round(float(np.percentile(arm[:, 2], 90)), 3) if len(arm) else None,
       "seat_depth_x_m": round(float(np.ptp(np.percentile(seat[:, 0], [2, 98]))), 3), "seat_width_y_m": round(float(np.ptp(np.percentile(seat[:, 1], [2, 98]))), 3),
       "overall_m": [round(float(x), 3) for x in ext], "legs": rep_legs, "cut_m": a.cut}
json.dump(rep, open(os.path.join(a.out, a.name + "_report.json"), "w"), indent=1)
print(json.dumps(rep))


def shade(meshes, yaw, pitch, size=520):
    cy, sy, cp, sp = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch)
    Rv = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]]) @ np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]]) @ np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    s = size / 1.2
    img = np.full((size, size, 3), 246, np.uint8)
    L = np.array([0.3, 0.5, 0.8]) / np.linalg.norm([0.3, 0.5, 0.8])
    tris, cols = [], []
    for mm in meshes:
        Q = (mm.vertices - [0, 0, 0.45]) @ Rv.T
        light = np.clip(0.35 + 0.65 * np.abs((mm.face_normals @ Rv.T) @ L), 0, 1)
        tris.append(Q[mm.faces]); cols.append(mm.visual.face_colors[:, :3] * light[:, None])
    T = np.concatenate(tris); Cc = np.concatenate(cols)
    for fi in np.argsort(T.mean(1)[:, 2]):
        pts = np.stack([size / 2 + T[fi, :, 0] * s, size / 2 - T[fi, :, 1] * s], 1).astype(np.int32)
        cv2.fillConvexPoly(img, pts, tuple(int(x) for x in Cc[fi][::-1]))
    return img


tiles = [shade([upper, frame], y, p) for y, p in ((0.4, 0.25), (1.9, 0.2), (3.3, 0.35), (4.7, 0.25))]
for t, label in zip(tiles, ["front", "side", "back", "other side"]):
    cv2.putText(t, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (60, 60, 60), 2)
cv2.imwrite(os.path.join(a.out, a.name + "_render.png"), np.hstack(tiles))
print("wrote", a.out)
