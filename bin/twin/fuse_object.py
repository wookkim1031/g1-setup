#!/usr/bin/env python3
"""Turn a head-camera (D435i) orbit around one piece of furniture into a coloured 3D mesh.

    python3 fuse_object.py <rec_dir> <trajectory.csv> <extrinsics.json> <out_dir> [--clock clock_offset.jsonl] [--name chair]
    python3 fuse_object.py <rec_dir> <poses>/trajectory.csv <poses>/extrinsics_identity.json <out_dir> --traj-skip 0   # hand-held, gemini_poses.py

The robot (on the gantry) circles the object with the object kept in the middle of the picture.
1. Camera pose per frame = DLIO pose at the frame time (robot clock -> laptop clock) x Mid-360 <- D435i extrinsics.
2. Floor = the largest level plane of a coarse fusion; object centre = median of the points at the image centres.
3. Depth outside a cylinder around the object (and within --floor-cut of the floor) is cut, so people, cable and floor drop out.
4. TSDF fusion (open3d, about 4 mm voxels in a cube around the object) with colour registered onto depth, marching cubes, small fragments removed.
5. The mesh is put into an object frame: floor at z = 0, centred, long side along x (minimum-area rectangle).
Writes <out>/<name>.ply (coloured mesh), <name>.glb, <name>_points.ply, <name>_views.png, <name>_report.json (size, heights).
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np
import open3d as o3d

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from d435i_common import Rec, Clock, Trajectory, fit_plane_ransac  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("rec"); ap.add_argument("traj"); ap.add_argument("extrinsics"); ap.add_argument("out")
ap.add_argument("--clock"); ap.add_argument("--name", default="object")
ap.add_argument("--dt", type=float, default=0.0, help="camera time offset (s), from the extrinsics check")
ap.add_argument("--step", type=int, default=2, help="use every n-th frame")
ap.add_argument("--max-pair-gap", type=float, default=0.03, help="skip colour frames whose nearest depth frame is further apart (s)")
ap.add_argument("--radius", type=float, default=0.55, help="keep depth within this distance of the object axis (m)")
ap.add_argument("--zmax", type=float, default=1.4, help="keep depth up to this height above the floor (m)")
ap.add_argument("--voxel", type=float, default=0.004)
ap.add_argument("--floor-cut", type=float, default=0.04, help="drop depth this close to the floor (m): floor noise, and thin legs are lost anyway")
ap.add_argument("--trunc", type=float, default=2.2, help="ignore depth beyond this (m)")
ap.add_argument("--traj-skip", type=float, default=3.0, help="seconds at the start of the trajectory to ignore (DLIO: 3; gemini_poses.py: 0)")
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)

rec = Rec(a.rec)
clock = Clock(a.clock or os.path.join(os.path.dirname(a.rec.rstrip("/")), "clock_offset.jsonl"))
traj = Trajectory(a.traj, skip=a.traj_skip)
T_sc = np.array(json.load(open(a.extrinsics))["T_sensor_from_depthcam"], float)
t_dd = clock.to_dd(rec.idx["c_trecv"]) + a.dt
frames = np.arange(0, len(rec), a.step)
frames = frames[rec.idx["pair_gap"][frames] <= a.max_pair_gap]  # colour and depth from the same moment only
M, ok = traj.pose(t_dd[frames])
frames, Tw = frames[ok], M[ok] @ T_sc  # world <- depth camera
print("%d frames with a pose (every %d-th)" % (len(frames), a.step), flush=True)

# ---- 2. floor and object centre from a coarse fusion
coarse, centres = [], []
for k in range(0, len(frames), 5):
    d = rec.depth(frames[k])
    if d.shape != (rec.depth_intr["h"], rec.depth_intr["w"]):  # not aligned to colour (driver without depth_registration)
        continue
    P, u, v = rec.depth_points(d, step=8, zmax=a.trunc)
    W = P @ Tw[k][:3, :3].T + Tw[k][:3, 3]
    coarse.append(W)
    c = (np.abs(u - d.shape[1] / 2) < 60) & (np.abs(v - d.shape[0] / 2) < 60)
    if c.sum() > 10:
        centres.append(np.median(W[c], 0))
coarse = np.concatenate(coarse)
low = coarse[coarse[:, 2] < np.percentile(coarse[:, 2], 40)]
n, d0, _ = fit_plane_ransac(low, thr=0.015)
if n[2] < 0:
    n, d0 = -n, -d0
floor_z = -d0 / n[2]  # floor height under the origin; the DLIO frame is gravity-aligned
print("floor normal (%.3f %.3f %.3f), tilt %.1f deg, z = %.3f m" % (*n, np.degrees(np.arccos(abs(n[2]))), floor_z), flush=True)
centres = np.array(centres)
above = coarse[(coarse @ n + d0 > 0.05) & (coarse @ n + d0 < a.zmax)]
cxy = np.median(centres[:, :2], 0)
for _ in range(3):  # pull the centre onto the object: median of the points above the floor near it
    near = above[np.linalg.norm(above[:, :2] - cxy, axis=1) < a.radius]
    if len(near) > 50:
        cxy = np.median(near[:, :2], 0)
print("object centre (%.3f, %.3f), %d image-centre points" % (*cxy, len(centres)), flush=True)

# ---- 3./4. masked RGB-D frames into a TSDF
k = rec.depth_intr
intr = o3d.camera.PinholeCameraIntrinsic(k["w"], k["h"], k["fx"], k["fy"], k["cx"], k["cy"])
# a fixed cube around the object (ScalableTSDFVolume extracts nothing in open3d 0.20 on Python 3.14)
L = max(2 * a.radius, a.zmax) + 0.1
res = int(np.ceil(L / a.voxel / 16)) * 16
vol = o3d.pipelines.integration.UniformTSDFVolume(length=L, resolution=res, sdf_trunc=4 * a.voxel,
                                                  color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
                                                  origin=np.array([cxy[0] - L / 2, cxy[1] - L / 2, floor_z - 0.05]))
print("TSDF cube %.2f m, %d^3 voxels of %.1f mm" % (L, res, 1000 * L / res), flush=True)
pts_all, col_all, used = [], [], 0
for k_, i in enumerate(frames):
    d = rec.depth(i)
    if d.shape != (rec.depth_intr["h"], rec.depth_intr["w"]):
        continue
    P, u, v = rec.depth_points(d, step=1, zmax=a.trunc)
    W = P @ Tw[k_][:3, :3].T + Tw[k_][:3, 3]
    h = W @ n + d0
    keep = (np.linalg.norm(W[:, :2] - cxy, axis=1) < a.radius) & (h > a.floor_cut) & (h < a.zmax)
    if keep.sum() < 500:
        continue
    col = rec.color(i)
    cu, cv_, _ = rec.to_color_pixels(P)
    inside = (cu >= 0) & (cu < col.shape[1] - 1) & (cv_ >= 0) & (cv_ < col.shape[0] - 1)
    keep &= inside
    dm = np.zeros_like(d)
    rgb = np.zeros(d.shape + (3,), np.uint8)
    ui, vi = u[keep].astype(int), v[keep].astype(int)
    dm[vi, ui] = d[vi, ui]
    rgb[vi, ui] = col[cv_[keep].astype(int), cu[keep].astype(int)][:, ::-1]
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(rgb), o3d.geometry.Image(dm), depth_scale=1.0 / rec.depth_scale, depth_trunc=a.trunc,
        convert_rgb_to_intensity=False)
    vol.integrate(rgbd, intr, np.linalg.inv(Tw[k_]))
    if k_ % 3 == 0:
        pts_all.append(W[keep][::4]); col_all.append(rgb[vi, ui][::4])
    used += 1
print("integrated %d frames" % used, flush=True)

mesh = vol.extract_triangle_mesh()
tri_c, n_tri, _ = (np.asarray(x) for x in mesh.cluster_connected_triangles())
big = n_tri >= max(200, 0.02 * n_tri.max())
mesh.remove_triangles_by_mask(~big[tri_c])
mesh.remove_unreferenced_vertices()
mesh.compute_vertex_normals()

# ---- 5. object frame: floor z = 0, centre of the footprint at the origin, long side along x
V = np.asarray(mesh.vertices)
R_level = np.eye(3)
if abs(n[2]) < 0.9999:
    ax = np.cross(n, [0, 0, 1.0]); s = np.linalg.norm(ax); c = n[2]
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    R_level = np.eye(3) + K + K @ K * ((1 - c) / s ** 2)
Vl = V @ R_level.T
Vl[:, 2] -= (np.array([0, 0, -d0 / n[2]]) @ R_level.T)[2]
rect = cv2.minAreaRect(Vl[:, :2].astype(np.float32))
(cx, cy), (w, l), ang = rect
if w < l:
    ang += 90; w, l = l, w
th = np.radians(-ang)
Rz = np.array([[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1]])
Vo = (Vl - [cx, cy, 0]) @ Rz.T
mesh.vertices = o3d.utility.Vector3dVector(Vo)
mesh.compute_vertex_normals()
T_obj = np.eye(4); T_obj[:3, :3] = Rz @ R_level; T_obj[:3, 3] = Rz @ (np.array([0, 0, -(np.array([0, 0, -d0 / n[2]]) @ R_level.T)[2]]) - [cx, cy, 0])

o3d.io.write_triangle_mesh(os.path.join(a.out, a.name + ".ply"), mesh)
pts = np.concatenate(pts_all) @ T_obj[:3, :3].T + T_obj[:3, 3]
pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts)); pc.colors = o3d.utility.Vector3dVector(np.concatenate(col_all) / 255.0)
o3d.io.write_point_cloud(os.path.join(a.out, a.name + "_points.ply"), pc)
try:
    import trimesh
    tm = trimesh.Trimesh(Vo, np.asarray(mesh.triangles), vertex_colors=(np.asarray(mesh.vertex_colors) * 255).astype(np.uint8), process=False)
    tm.export(os.path.join(a.out, a.name + ".glb"))
except ImportError:
    pass

# ---- report: size and the heights of horizontal surfaces (seat, table top)
nz = np.asarray(mesh.vertex_normals)[:, 2]
flat = Vo[nz > 0.9, 2]
hist, edges = np.histogram(flat, bins=np.arange(0.05, Vo[:, 2].max() + 0.02, 0.01))
peaks = [round(float(edges[j] + 0.005), 3) for j in np.argsort(hist)[::-1][:6] if hist[j] > 0.05 * hist.max()]
rep = {"frames_used": used, "vertices": len(Vo), "triangles": len(mesh.triangles),
       "size_m": {"x_long": round(float(np.ptp(Vo[:, 0])), 3), "y_short": round(float(np.ptp(Vo[:, 1])), 3), "z_height": round(float(Vo[:, 2].max()), 3)},
       "horizontal_surface_heights_m": sorted(peaks), "floor_tilt_deg": float(np.degrees(np.arccos(abs(n[2])))),
       "centre_world_xy": [float(cxy[0]), float(cxy[1])], "T_object_from_world": T_obj.tolist()}
json.dump(rep, open(os.path.join(a.out, a.name + "_report.json"), "w"), indent=1)
print(json.dumps({k: rep[k] for k in ("frames_used", "triangles", "size_m", "horizontal_surface_heights_m")}), flush=True)

# ---- views: front, side, top, oblique (point splats of the mesh vertices, painter's order)
def view(Rv, size=420):
    Q = Vo @ Rv.T
    C = np.asarray(mesh.vertex_colors)
    span = np.abs(Q[:, :2]).max() * 1.1
    s = size / (2 * span)
    o = np.argsort(Q[:, 2])
    img = np.full((size, size, 3), 245, np.uint8)
    u = (size / 2 + Q[o, 0] * s).astype(int); v = (size / 2 - Q[o, 1] * s).astype(int)
    for du in (0, 1):
        for dv in (0, 1):
            img[np.clip(v + dv, 0, size - 1), np.clip(u + du, 0, size - 1)] = (C[o][:, ::-1] * 255).astype(np.uint8)
    return img

def rot(yaw, pitch):
    cy_, sy = np.cos(yaw), np.sin(yaw); cp, sp = np.cos(pitch), np.sin(pitch)
    Ry = np.array([[cy_, -sy, 0], [sy, cy_, 0], [0, 0, 1]])
    Rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])
    return Rx @ np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]) @ Ry  # camera: x right, y up, z towards the viewer

Vo = Vo - [0, 0, Vo[:, 2].max() / 2]
tiles = [view(rot(0, 0)), view(rot(np.pi / 2, 0)), view(rot(np.pi / 4, 0.45)), view(np.eye(3))]
for t, label in zip(tiles, ["front", "side", "oblique", "top"]):
    cv2.putText(t, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (60, 60, 60), 2)
cv2.imwrite(os.path.join(a.out, a.name + "_views.png"), np.hstack(tiles))
print("wrote", a.out, flush=True)
