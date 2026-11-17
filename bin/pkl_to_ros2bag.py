#!/usr/bin/env python3
"""Convert a g1_lidar_rec.py pickle recording (cloud.pkl + imu.pkl) into a ROS 2 (Humble, sqlite3) bag.

    python3 pkl_to_ros2bag.py <rec_dir> <out_bag_dir>

- /livox/lidar  sensor_msgs/PointCloud2, fields x y z intensity(f32) ring(u8) time(f32, SECONDS rel. to header stamp)
  (the recording stores `time` in nanoseconds; DLIO's Velodyne path expects seconds), no-return (0,0,0) points dropped
- /livox/imu    sensor_msgs/Imu, acc converted g -> m/s^2 (x 9.80665), gyro rad/s, orientation unset (cov[0] = -1)
Bag timestamps = sensor header stamps. Prints per-stream statistics at the end.
"""
import pickle
import sys
import time as _time

import numpy as np
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

G = 9.80665
rec_dir, out = sys.argv[1], sys.argv[2]
RESTAMP = "--restamp-imu" in sys.argv  # replace jittery IMU header stamps by a 200 Hz linear fit (stamp_i = a + b*i)
ts = get_typestore(Stores.ROS2_HUMBLE)
T = ts.types
PointCloud2, PointField, Imu = T["sensor_msgs/msg/PointCloud2"], T["sensor_msgs/msg/PointField"], T["sensor_msgs/msg/Imu"]
Header, Time = T["std_msgs/msg/Header"], T["builtin_interfaces/msg/Time"]
Vector3, Quaternion = T["geometry_msgs/msg/Vector3"], T["geometry_msgs/msg/Quaternion"]


def stream(path):
    with open(path, "rb") as f:
        while True:
            try:
                yield pickle.load(f)
            except EOFError:
                return


def hdr(t, frame):
    sec = int(t)
    return Header(stamp=Time(sec=sec, nanosec=int(round((t - sec) * 1e9))), frame_id=frame)


# rosbags >= 0.10 writes metadata version 8 or 9; 8 keeps offered_qos_profiles as a string, which Humble can read
writer = Writer(out, version=8)  # default storage plugin: sqlite3
t0 = _time.time()
with writer as w:
    c_imu = w.add_connection("/livox/imu", Imu.__msgtype__, typestore=ts)
    c_pc = w.add_connection("/livox/lidar", PointCloud2.__msgtype__, typestore=ts)

    # ---- IMU ----
    n_imu, acc_n, imu_t = 0, [], []
    t_fit = None
    if RESTAMP:
        t_raw = np.array([r["t"] for r in stream(f"{rec_dir}/imu.pkl")])
        i = np.arange(len(t_raw))
        b, a = np.polyfit(i, t_raw, 1)
        res = (t_raw - (a + b * i)) * 1e3
        print(f"IMU restamp: fit rate {1 / b:.4f} Hz, residual ms min {res.min():.1f} p50 {np.median(res):.1f} "
              f"p99 {np.percentile(res, 99):.1f} max {res.max():.1f}", flush=True)
        t_fit = a + b * i
    cov_unknown = np.array([-1.0] + [0.0] * 8)
    cov_zero = np.zeros(9)
    for k, r in enumerate(stream(f"{rec_dir}/imu.pkl")):
        if t_fit is not None:
            r["t"] = float(t_fit[k])
        ax, ay, az = (v * G for v in r["acc"])
        gx, gy, gz = r["gyro"]
        m = Imu(header=hdr(r["t"], "livox_frame"),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0), orientation_covariance=cov_unknown,
                angular_velocity=Vector3(x=gx, y=gy, z=gz), angular_velocity_covariance=cov_zero,
                linear_acceleration=Vector3(x=ax, y=ay, z=az), linear_acceleration_covariance=cov_zero)
        w.write(c_imu, int(round(r["t"] * 1e9)), ts.serialize_cdr(m, Imu.__msgtype__))
        n_imu += 1
        acc_n.append((ax * ax + ay * ay + az * az) ** 0.5)
        imu_t.append(r["t"])
    imu_t = np.array(imu_t)
    print(f"IMU: {n_imu} msgs, {imu_t[-1] - imu_t[0]:.1f} s, |acc| mean {np.mean(acc_n):.3f} m/s^2, "
          f"max gap {np.diff(imu_t).max() * 1000:.0f} ms", flush=True)

    # ---- clouds ----
    fields = None
    n_pc, kept, total, cloud_t = 0, [], 0, []
    tmax = []
    for r in stream(f"{rec_dir}/cloud.pkl"):
        buf = np.frombuffer(r["data"], np.uint8).reshape(-1, r["point_step"])
        off = {name: (o, dt) for name, o, dt, c in r["fields"]}
        if fields is None:
            fields = [PointField(name=n, offset=o, datatype=dt, count=c) for n, o, dt, c in r["fields"]]
            point_step = r["point_step"]
            toff = off["time"][0]
        xyz = np.stack([buf[:, off[c][0]:off[c][0] + 4].copy().view("<f4").ravel() for c in "xyz"], axis=1)
        good = np.isfinite(xyz).all(axis=1) & (np.abs(xyz).sum(axis=1) > 1e-6)
        rows = buf[good].copy()
        # time: ns (float32) -> seconds (float32), relative to header stamp
        tt = rows[:, toff:toff + 4].copy().view("<f4").ravel() * np.float32(1e-9)
        rows[:, toff:toff + 4] = tt.astype("<f4").view(np.uint8).reshape(-1, 4)
        n = int(good.sum())
        m = PointCloud2(header=hdr(r["t"], "livox_frame"), height=1, width=n, fields=fields,
                        is_bigendian=False, point_step=point_step, row_step=point_step * n,
                        data=rows.reshape(-1), is_dense=True)
        w.write(c_pc, int(round(r["t"] * 1e9)), ts.serialize_cdr(m, PointCloud2.__msgtype__))
        n_pc += 1
        kept.append(n)
        total += len(xyz)
        cloud_t.append(r["t"])
        tmax.append(float(tt.max()) if n else 0.0)
        if n_pc % 500 == 0:
            print(f"  {n_pc} clouds, {_time.time() - t0:.0f} s", flush=True)
    kept = np.array(kept)
    cloud_t = np.array(cloud_t)
    print(f"CLOUD: {n_pc} msgs, {cloud_t[-1] - cloud_t[0]:.1f} s, points kept {kept.sum()} / {total} "
          f"({100 * kept.sum() / total:.1f} %), per frame median {np.median(kept):.0f} min {kept.min()} max {kept.max()}")
    print(f"  frames with <500 pts: {(kept < 500).sum()}; first frame with >=1000 pts: idx {int(np.argmax(kept >= 1000))} "
          f"(t+{cloud_t[int(np.argmax(kept >= 1000))] - cloud_t[0]:.1f} s); last frame with >=1000 pts: idx "
          f"{len(kept) - 1 - int(np.argmax(kept[::-1] >= 1000))}")
    print(f"  per-point time max (s) median {np.median(tmax):.4f}; IMU covers cloud span: "
          f"{imu_t[0] <= cloud_t[0] + 0.01 and imu_t[-1] >= cloud_t[-1] - 0.01}")
    print("  kept-per-frame profile (every 100th frame):", kept[::100].tolist())
print(f"done in {_time.time() - t0:.0f} s -> {out}")
