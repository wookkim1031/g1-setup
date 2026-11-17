#!/usr/bin/env python3
"""Post-process a DLIO run: trajectory CSV, loop-closure check, PLY export, top-down PNG, map statistics.

    python3 lidar_map_report.py <dlio_run_dir> <out_dir> [--name hall_walk]

Reads <run>/odom_bag (ros2 bag of /dlio/odom_node/odom) and <run>/map_0.05/dlio_map.pcd, <run>/map_0.02/dlio_map.pcd.
Writes <out>/<name>_trajectory.csv, <name>_map_0.02.ply, <name>_map_0.05.ply, <name>_topdown.png, <name>_report.json
"""
import argparse
import json
import os
import re

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from rosbags.rosbag2 import Reader  # noqa: E402
from rosbags.typesys import Stores, get_typestore  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("run"); ap.add_argument("out"); ap.add_argument("--name", default="hall_walk")
ap.add_argument("--dense", default=None, help="dense PLY (from dense_map.py) to render in the PNG instead of the keyframe map")
ap.add_argument("--rmax", type=float, default=25.0, help="PNG: ignore points farther than this from the trajectory centroid")
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)
rep = {}

# ---------- trajectory ----------
ts = get_typestore(Stores.ROS2_HUMBLE)
rows = []
with Reader(os.path.join(a.run, "odom_bag")) as r:
    conns = [c for c in r.connections if c.topic == "/dlio/odom_node/odom"]
    for conn, t, raw in r.messages(connections=conns):
        m = ts.deserialize_cdr(raw, conn.msgtype)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        v = m.twist.twist.linear
        rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z))
tr = np.array(rows)
tr = tr[tr[:, 0] > 1e9]  # drop DLIO's initial pose published with stamp 0
tr = tr[np.argsort(tr[:, 0])]
csv = os.path.join(a.out, f"{a.name}_trajectory.csv")
np.savetxt(csv, tr, delimiter=",", header="t,x,y,z,qx,qy,qz,qw,vx,vy,vz", comments="", fmt="%.6f")
xyz = tr[:, 1:4]
seg = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
path_len = float(seg.sum())
# loop closure: first pose vs last pose; also vs. mean of the last 2 s (robot standing still at the end)
t = tr[:, 0]
end_mask = t >= t[-1] - 2.0
start_mask = t <= t[0] + 2.0
p0 = xyz[start_mask].mean(axis=0); p1 = xyz[end_mask].mean(axis=0)
d_se = float(np.linalg.norm(xyz[-1] - xyz[0]))
d_se_avg = float(np.linalg.norm(p1 - p0))
d_xy = float(np.linalg.norm((xyz[-1] - xyz[0])[:2]))


def yaw(q):  # q = x,y,z,w
    x, y, z, w = q
    return np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


dyaw = float((yaw(tr[-1, 4:8]) - yaw(tr[0, 4:8]) + 180) % 360 - 180)
rep["trajectory"] = dict(n_poses=int(len(tr)), duration_s=float(t[-1] - t[0]), path_length_m=path_len,
                         start=xyz[0].round(3).tolist(), end=xyz[-1].round(3).tolist(),
                         start_end_dist_m=d_se, start_end_dist_xy_m=d_xy, start_end_dist_2s_mean_m=d_se_avg,
                         start_end_z_m=float(xyz[-1, 2] - xyz[0, 2]), start_end_yaw_deg=dyaw,
                         z_min=float(xyz[:, 2].min()), z_max=float(xyz[:, 2].max()),
                         mean_speed_mps=float(path_len / (t[-1] - t[0])))


# ---------- PCD ----------
def read_pcd(path):
    with open(path, "rb") as f:
        hdr = {}
        while True:
            line = f.readline().decode("ascii", "ignore").strip()
            k = line.split(" ")[0]
            hdr[k] = line.split(" ")[1:]
            if k == "DATA":
                break
        if hdr["DATA"][0] != "binary":
            raise SystemExit("only binary PCD supported")
        names, sizes, types, counts = hdr["FIELDS"], list(map(int, hdr["SIZE"])), hdr["TYPE"], list(map(int, hdr["COUNT"]))
        dt = np.dtype([(n, {"F": "f", "U": "u", "I": "i"}[ty] + str(s)) if c == 1 else (n, {"F": "f", "U": "u", "I": "i"}[ty] + str(s), (c,))
                       for n, s, ty, c in zip(names, sizes, types, counts)])
        n = int(hdr["POINTS"][0])
        arr = np.frombuffer(f.read(n * dt.itemsize), dtype=dt, count=n)
    return arr, names


def write_ply(path, arr, names):
    n = len(arr)
    has_i = "intensity" in names
    dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4")] + ([("intensity", "<f4")] if has_i else []))
    out = np.empty(n, dt)
    for k in dt.names:
        out[k] = arr[k]
    with open(path, "wb") as f:
        f.write(("ply\nformat binary_little_endian 1.0\ncomment DLIO map, Unitree G1 Livox Mid-360\n"
                 f"element vertex {n}\nproperty float x\nproperty float y\nproperty float z\n"
                 + ("property float intensity\n" if has_i else "") + "end_header\n").encode())
        f.write(out.tobytes())


maps = {}
for leaf in ("0.02", "0.05"):
    p = os.path.join(a.run, f"map_{leaf}", "dlio_map.pcd")
    if not os.path.exists(p):
        continue
    arr, names = read_pcd(p)
    pts = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float64)
    ok = np.isfinite(pts).all(axis=1)
    pts = pts[ok]; arr = arr[ok]
    write_ply(os.path.join(a.out, f"{a.name}_map_{leaf}.ply"), arr, names)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    maps[leaf] = dict(points=int(len(pts)), fields=names, min=lo.round(3).tolist(), max=hi.round(3).tolist(),
                      extent_m=(hi - lo).round(3).tolist(), floor_z_p1=float(np.percentile(pts[:, 2], 1)),
                      ceiling_z_p99=float(np.percentile(pts[:, 2], 99)))
    if leaf == "0.05":
        P = pts
rep["maps"] = maps

# ---------- top-down PNG ----------
map_label = f"DLIO keyframe map (leaf 0.05 m, {len(P):,} pts)"
if a.dense:
    with open(a.dense, "rb") as f:
        hdr = b""
        while not hdr.endswith(b"end_header\n"):
            hdr += f.readline()
        nv = int(re.search(rb"element vertex (\d+)", hdr).group(1))
        props = re.findall(rb"property float (\w+)", hdr)
        dense = np.frombuffer(f.read(nv * 4 * len(props)), dtype=np.dtype([(pn.decode(), "<f4") for pn in props]))
    P = np.stack([dense["x"], dense["y"], dense["z"]], axis=1).astype(np.float64)
    map_label = f"dense map, all deskewed scans (voxel 0.02 m, {len(P):,} pts)"
    rep["dense_png_source"] = a.dense
cen = xyz.mean(axis=0)
near = np.linalg.norm(P[:, :2] - cen[:2], axis=1) < a.rmax
P = P[near]
floor = np.percentile(P[:, 2], 1); ceil = np.percentile(P[:, 2], 99)
# panel 1: everything, colored by height; panel 2: wall slice (0.3 .. 1.9 m above the floor), density
fig, axs = plt.subplots(1, 2, figsize=(20, 10), dpi=120)
ax = axs[0]
idx = np.random.default_rng(0).permutation(len(P))[:3_000_000]
sc = ax.scatter(P[idx, 0], P[idx, 1], c=P[idx, 2], s=0.05, cmap="viridis", vmin=floor, vmax=ceil, rasterized=True, linewidths=0)
plt.colorbar(sc, ax=ax, fraction=0.03, pad=0.01, label="z [m] (odom frame)")
ax.set_title(f"{map_label} — all points by height")
ax = axs[1]
sl = (P[:, 2] > floor + 0.3) & (P[:, 2] < floor + 1.9)
res = 0.05
xe = np.arange(P[:, 0].min(), P[:, 0].max() + res, res); ye = np.arange(P[:, 1].min(), P[:, 1].max() + res, res)
H, _, _ = np.histogram2d(P[sl, 0], P[sl, 1], bins=[xe, ye])
ax.imshow(np.log1p(H.T), origin="lower", extent=[xe[0], xe[-1], ye[0], ye[-1]], cmap="Greys", aspect="equal")
ax.set_xlim(axs[0].get_xlim()); ax.set_ylim(axs[0].get_ylim())
ax.set_title(f"slice {0.3}–{1.9} m above floor (z_floor={floor:.2f}), log density, {res} m cells")
for ax in axs:
    ax.plot(xyz[:, 0], xyz[:, 1], "-", color="red", lw=1.0, label=f"trajectory ({path_len:.1f} m, {t[-1]-t[0]:.0f} s)")
    ax.plot(xyz[0, 0], xyz[0, 1], "o", color="lime", ms=9, mec="k", label="start")
    ax.plot(xyz[-1, 0], xyz[-1, 1], "X", color="orange", ms=10, mec="k", label=f"end (start→end {d_se:.2f} m)")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend(loc="upper right")
ext = P.max(axis=0) - P.min(axis=0)
fig.suptitle(f"{a.name}: Unitree G1 Livox Mid-360, DLIO (ros2 branch) — shown extent "
             f"{ext[0]:.1f} x {ext[1]:.1f} x {ext[2]:.1f} m (points within {a.rmax:g} m of the trajectory centroid)")
fig.tight_layout()
png = os.path.join(a.out, f"{a.name}_topdown.png")
fig.savefig(png)
rep["files"] = dict(csv=csv, png=png, ply=[os.path.join(a.out, f"{a.name}_map_{k}.ply") for k in maps])
json.dump(rep, open(os.path.join(a.out, f"{a.name}_report.json"), "w"), indent=2)
print(json.dumps(rep, indent=2))
