#!/usr/bin/env python3
"""Closed furniture mesh straight from a fuse_object.py *_points.ply (no TSDF, which carves thin legs away).

    python3 mesh_from_points.py <points.ply> <out_dir> --name okay2_chair [--radius 0.45] [--voxel 0.008] [--keep 0.35]

The raw D435i points of an object seen from all sides hold every part, blurred by pose error. Points are binned into
voxels; the voxel counts are smoothed and a voxel is kept above --keep of the peak density in its --window
neighbourhood, so the dense core of each part stays (thin legs too, which get fewer points) and the blur halo goes. The largest connected part is meshed with marching cubes (closed, watertight),
smoothed, coloured from the nearest points, turned so the back points to -x with the seat centre at (0, 0) (as in
furniture_models), and decimated. Writes <name>.glb/.obj/.ply, <name>_views.png, <name>_report.json.
"""
import argparse
import json
import os

import cv2
import numpy as np
import open3d as o3d
import trimesh
from scipy import ndimage
from skimage import measure

ap = argparse.ArgumentParser()
ap.add_argument("points"); ap.add_argument("out"); ap.add_argument("--name", default="object")
ap.add_argument("--radius", type=float, default=0.45, help="crop radius around the object centre (m)")
ap.add_argument("--floor-cut", type=float, default=0.03, help="drop points below this height (m); the parts cut there are extended down to the floor")
ap.add_argument("--close", type=int, default=6, help="morphological closing (voxels) to fill small holes and gaps")
ap.add_argument("--min-part", type=float, default=0.05, help="keep connected parts of at least this fraction of the largest")
ap.add_argument("--voxel", type=float, default=0.008)
ap.add_argument("--sigma", type=float, default=1.0, help="smoothing of the voxel counts (voxels)")
ap.add_argument("--keep", type=float, default=0.35, help="threshold as a fraction of the local peak density (within --window)")
ap.add_argument("--window", type=float, default=0.10, help="size of the local peak window (m); thin legs are judged against their own core")
ap.add_argument("--min-frac", type=float, default=0.03, help="absolute floor as a fraction of the 99th-percentile density (rejects specks)")
ap.add_argument("--seat-band", type=float, nargs=2, default=(0.38, 0.52), help="height band of the seat (m)")
ap.add_argument("--target-faces", type=int, default=80000)
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)

pc = o3d.io.read_point_cloud(a.points)
P = np.asarray(pc.points); C = np.asarray(pc.colors)
c = np.median(P[(P[:, 2] > a.seat_band[0]) & (P[:, 2] < a.seat_band[1]), :2], 0)
m = (P[:, 2] > a.floor_cut) & (np.linalg.norm(P[:, :2] - c, axis=1) < a.radius)
P, C = P[m] - [c[0], c[1], 0], C[m]
print("%d points in the crop" % len(P))

# voxel counts -> smoothed density -> solid
lo = P.min(0) - 3 * a.voxel
lo[2] = -2 * a.voxel  # grid reaches below the floor so the feet can be extended to z = 0
idx = np.floor((P - lo) / a.voxel).astype(int)
shape = idx.max(0) + 4
cnt = np.zeros(shape, np.float32)
np.add.at(cnt, tuple(idx.T), 1)
den = ndimage.gaussian_filter(cnt, a.sigma)
thr = a.min_frac * np.percentile(den[den > 0], 99)
peak = ndimage.maximum_filter(den, size=max(3, int(round(a.window / a.voxel)) | 1))
solid = (den > thr) & (den > a.keep * peak)
k_cut, k_floor = int(np.ceil((a.floor_cut - lo[2]) / a.voxel)), int(round(-lo[2] / a.voxel))
solid[..., :k_cut] = False
lab, n = ndimage.label(solid)
sizes = ndimage.sum(solid, lab, range(1, n + 1))
keep = 1 + np.flatnonzero(sizes >= a.min_part * sizes.max())
solid = np.isin(lab, keep)
if a.close:
    solid = ndimage.binary_closing(np.pad(solid, a.close), ndimage.generate_binary_structure(3, 1), a.close)[(slice(a.close, -a.close),) * 3]
solid = ndimage.binary_fill_holes(solid)
feet = solid[..., k_cut]
solid[..., k_floor:k_cut] |= feet[..., None]  # whatever reaches the cut stands on the floor
print("threshold %.1f points/voxel: %d components, kept %d (%.0f%% of the voxels), %d foot columns"
      % (thr, n, len(keep), 100 * sizes[keep - 1].sum() / sizes.sum(), feet.sum()))

v, f, _, _ = measure.marching_cubes(np.pad(solid, 1).astype(np.float32), 0.5)
v = (v - 1) * a.voxel + lo
mesh = trimesh.Trimesh(v, f[:, ::-1], process=True)
trimesh.smoothing.filter_taubin(mesh, iterations=10)
if len(mesh.faces) > a.target_faces:
    try:
        mesh = mesh.simplify_quadric_decimation(face_count=a.target_faces)
    except Exception as e:  # needs fast-simplification; the full mesh is fine too
        print("no decimation:", e)

# back -> -x, seat centre at (0, 0)
V = mesh.vertices
seat = V[(V[:, 2] > a.seat_band[0]) & (V[:, 2] < a.seat_band[1])]
back = V[V[:, 2] > a.seat_band[1] + 0.2]
d = back[:, :2].mean(0) - seat[:, :2].mean(0)
th = np.arctan2(d[1], d[0])
cs, sn = np.cos(np.pi - th), np.sin(np.pi - th)
R = np.array([[cs, -sn, 0], [sn, cs, 0], [0, 0, 1]])
V = V @ R.T; P = P @ R.T
seat = V[(V[:, 2] > a.seat_band[0]) & (V[:, 2] < a.seat_band[1])]
sc = (np.percentile(seat[:, :2], 2, 0) + np.percentile(seat[:, :2], 98, 0)) / 2
V[:, :2] -= sc; P[:, :2] -= sc
mesh.vertices = V

# colour: median of the nearest points (the halo is greyer than the core, so take the 5 nearest)
kd = o3d.geometry.KDTreeFlann(o3d.geometry.PointCloud(o3d.utility.Vector3dVector(P)))
col = np.zeros((len(V), 3))
for i, x in enumerate(V):
    _, nn, _ = kd.search_knn_vector_3d(x, 5)
    col[i] = np.median(C[np.asarray(nn)], 0)
mesh.visual.vertex_colors = np.hstack([(col * 255).astype(np.uint8), np.full((len(V), 1), 255, np.uint8)])

for ext in ("glb", "obj", "ply"):
    mesh.export(os.path.join(a.out, "%s.%s" % (a.name, ext)))

# report: size and the horizontal surfaces (z of upward faces with most area)
nz = mesh.face_normals[:, 2]
up = nz > 0.9
hz, he = np.histogram(mesh.triangles_center[up, 2], bins=np.arange(0, V[:, 2].max() + 0.02, 0.01), weights=mesh.area_faces[up])
top = he[np.argsort(hz)[::-1][:5]] + 0.005
ext3 = V.max(0) - V.min(0)
rep = {"faces": len(mesh.faces), "watertight": bool(mesh.is_watertight), "volume_m3": round(float(mesh.volume), 4) if mesh.is_watertight else None,
       "size_m": {"x_depth": round(float(ext3[0]), 3), "y_width": round(float(ext3[1]), 3), "z_height": round(float(ext3[2]), 3)},
       "horizontal_surfaces_m": sorted(round(float(z), 3) for z in top), "threshold_points_per_voxel": round(float(thr), 2),
       "voxel_m": a.voxel, "seat_band": list(a.seat_band), "yaw_to_object_rad": float(np.pi - th), "seat_centre_shift_xy": [float(x) for x in sc]}
json.dump(rep, open(os.path.join(a.out, a.name + "_report.json"), "w"), indent=1)
print(json.dumps(rep))


def shade(mesh, yaw, pitch, size=520):
    cy, sy, cp, sp = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch)
    Rv = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]]) @ np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]]) @ np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Q = (mesh.vertices - [0, 0, 0.45]) @ Rv.T
    s = size / 1.2
    img = np.full((size, size, 3), 246, np.uint8); zb = np.full((size, size), -1e9)
    N = mesh.face_normals @ Rv.T
    light = np.clip(0.35 + 0.65 * np.abs(N @ np.array([0.3, 0.5, 0.8]) / np.linalg.norm([0.3, 0.5, 0.8])), 0, 1)
    fc = mesh.visual.face_colors[:, :3].astype(float)
    order = np.argsort(Q[mesh.faces].mean(1)[:, 2])
    for fi in order:
        tri = Q[mesh.faces[fi]]
        pts = np.stack([size / 2 + tri[:, 0] * s, size / 2 - tri[:, 1] * s], 1).astype(np.int32)
        cv2.fillConvexPoly(img, pts, tuple(int(x) for x in (fc[fi] * light[fi])[::-1]))
    return img


tiles = [shade(mesh, y, p) for y, p in ((0.4, 0.25), (1.9, 0.2), (3.3, 0.35), (4.7, 0.25))]
cv2.imwrite(os.path.join(a.out, a.name + "_views.png"), np.hstack(tiles))
print("wrote", a.out)
