#!/usr/bin/env python3
"""Build a dense map from DLIO's deskewed scans (already in the odom frame) recorded in a ros2 bag.

    python3 dense_map.py <run_dir> <out_dir> --name <name> [--voxel 0.02] [--tmax <rel s>]

Reads <run>/odom_bag (/dlio/odom_node/pointcloud/deskewed), voxel-downsamples all scans together (one point per
voxel = mean of its points, intensity = mean) and writes <out>/<name>_dense_<voxel>.ply + .pcd and a stats JSON.
"""
import argparse, json, os
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

ap = argparse.ArgumentParser()
ap.add_argument("run"); ap.add_argument("out"); ap.add_argument("--name", required=True)
ap.add_argument("--voxel", type=float, default=0.02); ap.add_argument("--tmax", type=float, default=None)
ap.add_argument("--rmax", type=float, default=30.0, help="drop points farther than this from the sensor pose (m)")
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)
ts = get_typestore(Stores.ROS2_HUMBLE)

def pc2_xyzi(m):
    off = {f.name: f.offset for f in m.fields}
    buf = np.frombuffer(m.data.tobytes() if hasattr(m.data, "tobytes") else bytes(m.data), np.uint8).reshape(-1, m.point_step)
    xyz = np.stack([buf[:, off[c]:off[c] + 4].copy().view("<f4").ravel() for c in "xyz"], 1)
    inten = buf[:, off["intensity"]:off["intensity"] + 4].copy().view("<f4").ravel() if "intensity" in off else np.zeros(len(xyz), np.float32)
    return xyz, inten

# pass 1: odom poses (for range filtering) ; pass 2: clouds
poses = []
with Reader(os.path.join(a.run, "odom_bag")) as r:
    co = [c for c in r.connections if c.topic == "/dlio/odom_node/odom"]
    for conn, t, raw in r.messages(connections=co):
        m = ts.deserialize_cdr(raw, conn.msgtype)
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if st > 1e9:
            poses.append((st, m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z))
poses = np.array(sorted(poses)); t0 = poses[0, 0]

V = a.voxel
acc = {}  # voxel key -> [sum x, sum y, sum z, sum i, n]
keys_all, sums_all = [], []
n_scans = n_pts = 0
with Reader(os.path.join(a.run, "odom_bag")) as r:
    cc = [c for c in r.connections if c.topic == "/dlio/odom_node/pointcloud/deskewed"]
    print("deskewed msgs in bag:", sum(c.msgcount for c in cc), "frame:", end=" ", flush=True)
    for conn, t, raw in r.messages(connections=cc):
        m = ts.deserialize_cdr(raw, conn.msgtype)
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if n_scans == 0:
            print(m.header.frame_id, flush=True)
        if a.tmax is not None and st - t0 > a.tmax:
            break
        xyz, inten = pc2_xyzi(m)
        ok = np.isfinite(xyz).all(1)
        # sensor position at this stamp -> range filter
        k = np.searchsorted(poses[:, 0], st); k = min(max(k, 0), len(poses) - 1)
        ok &= np.linalg.norm(xyz - poses[k, 1:4], axis=1) < a.rmax
        xyz, inten = xyz[ok], inten[ok]
        idx = np.floor(xyz / V).astype(np.int64)
        keys_all.append(idx); sums_all.append(np.column_stack([xyz, inten]))
        n_scans += 1; n_pts += len(xyz)
        if n_scans % 500 == 0:
            print(f"  {n_scans} scans, {n_pts} pts", flush=True)
keys = np.concatenate(keys_all); vals = np.concatenate(sums_all).astype(np.float64)
# combine voxel index triple into one int64 key
kmin = keys.min(0); kk = keys - kmin; dims = kk.max(0) + 1
flat = (kk[:, 0] * dims[1] + kk[:, 1]) * dims[2] + kk[:, 2]
uniq, inv, cnt = np.unique(flat, return_inverse=True, return_counts=True)
out = np.zeros((len(uniq), 4)); np.add.at(out, inv, vals); out /= cnt[:, None]
P = out[:, :3].astype(np.float32); I = out[:, 3].astype(np.float32)
lo, hi = P.min(0), P.max(0)
stats = dict(scans=n_scans, raw_points=int(n_pts), voxel_m=V, points=int(len(P)), min=lo.round(3).tolist(), max=hi.round(3).tolist(),
             extent_m=(hi - lo).round(3).tolist(), floor_z_p1=float(np.percentile(P[:, 2], 1)), ceiling_z_p99=float(np.percentile(P[:, 2], 99)),
             points_per_voxel_median=float(np.median(cnt)))
print(json.dumps(stats, indent=1))
base = os.path.join(a.out, f"{a.name}_dense_{V:g}")
dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")])
arr = np.empty(len(P), dt); arr["x"], arr["y"], arr["z"], arr["intensity"] = P[:, 0], P[:, 1], P[:, 2], I
with open(base + ".ply", "wb") as f:
    f.write((f"ply\nformat binary_little_endian 1.0\ncomment dense DLIO map (all deskewed scans, voxel {V} m), Unitree G1 Livox Mid-360\n"
             f"element vertex {len(P)}\nproperty float x\nproperty float y\nproperty float z\nproperty float intensity\nend_header\n").encode()); f.write(arr.tobytes())
with open(base + ".pcd", "wb") as f:
    f.write((f"# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
             f"WIDTH {len(P)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(P)}\nDATA binary\n").encode()); f.write(arr.tobytes())
json.dump(stats, open(base + "_stats.json", "w"), indent=2)
print("wrote", base + ".ply", base + ".pcd")
