#!/usr/bin/env python3
"""Record the G1's Livox Mid-360 (point cloud + IMU) and Unitree SLAM odometry straight from DDS, no ROS needed.

Topics (CycloneDDS, domain 0, as the G1 publishes them):
    rt/utlidar/cloud_livox_mid360   sensor_msgs::msg::dds_::PointCloud2_
    rt/utlidar/imu_livox_mid360     sensor_msgs::msg::dds_::Imu_
    rt/unitree/slam_mapping/odom    nav_msgs::msg::dds_::Odometry_   (only if Unitree SLAM is running)

Output: one pickle stream per topic in <out>/, each record a dict with receive time, header stamp and the raw
fields (cloud: width, height, fields, point_step, row_step, is_bigendian, data bytes). Convert to a ROS 2 bag
later (rosbags library) for DLIO.

Usage (py310-robot env, cable on enp58s0):
    python g1_lidar_rec.py probe                 # 6 s: rates, points per frame, frame ids
    python g1_lidar_rec.py record --out ~/bags/<name>   # Ctrl-C stops; prints counts
"""
import argparse
import os
import pickle
import signal
import sys
import time
from dataclasses import dataclass

sys.path.insert(0, os.path.expanduser("~/unitree_sdk2_python"))
os.environ.setdefault(
    "CYCLONEDDS_URI",
    '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp58s0" priority="default" '
    'multicast="default"/></Interfaces></General></Domain></CycloneDDS>')

from cyclonedds.domain import DomainParticipant  # noqa: E402
from cyclonedds.idl import IdlStruct  # noqa: E402
from cyclonedds.idl.types import array, float64  # noqa: E402
from cyclonedds.sub import DataReader  # noqa: E402
from cyclonedds.topic import Topic  # noqa: E402
from cyclonedds.core import Qos, Policy  # noqa: E402

from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_  # noqa: E402
from unitree_sdk2py.idl.std_msgs.msg.dds_ import Header_  # noqa: E402
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Quaternion_, Vector3_  # noqa: E402
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_  # noqa: E402


@dataclass
class Imu_(IdlStruct, typename="sensor_msgs::msg::dds_::Imu_"):
    header: Header_
    orientation: Quaternion_
    orientation_covariance: array[float64, 9]
    angular_velocity: Vector3_
    angular_velocity_covariance: array[float64, 9]
    linear_acceleration: Vector3_
    linear_acceleration_covariance: array[float64, 9]


TOPICS = {
    "cloud": ("rt/utlidar/cloud_livox_mid360", PointCloud2_),
    "imu": ("rt/utlidar/imu_livox_mid360", Imu_),
    "odom": ("rt/unitree/slam_mapping/odom", Odometry_),
}


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def cloud_record(m, t):
    return {"t_recv": t, "t": stamp(m.header), "frame_id": m.header.frame_id, "height": m.height, "width": m.width,
            "fields": [(f.name, f.offset, f.datatype, f.count) for f in m.fields], "is_bigendian": m.is_bigendian,
            "point_step": m.point_step, "row_step": m.row_step, "is_dense": m.is_dense, "data": bytes(m.data)}


def imu_record(m, t):
    return {"t_recv": t, "t": stamp(m.header), "frame_id": m.header.frame_id,
            "q": (m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w),
            "gyro": (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z),
            "acc": (m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z)}


def odom_record(m, t):
    p, o = m.pose.pose.position, m.pose.pose.orientation
    v, w = m.twist.twist.linear, m.twist.twist.angular
    return {"t_recv": t, "t": stamp(m.header), "frame_id": m.header.frame_id, "child": m.child_frame_id,
            "p": (p.x, p.y, p.z), "q": (o.x, o.y, o.z, o.w), "v": (v.x, v.y, v.z), "w": (w.x, w.y, w.z)}


REC = {"cloud": cloud_record, "imu": imu_record, "odom": odom_record}


def readers(dp):
    qos = Qos(Policy.Reliability.BestEffort, Policy.History.KeepLast(50))
    out = {}
    for k, (name, typ) in TOPICS.items():
        out[k] = DataReader(dp, Topic(dp, name, typ), qos=qos)
    return out


def probe(seconds=6.0):
    dp = DomainParticipant(0)
    rd = readers(dp)
    n = {k: 0 for k in rd}
    info = {}
    t0 = time.time()
    while time.time() - t0 < seconds:
        for k, r in rd.items():
            for m in r.take(N=100):
                if not hasattr(m, "header"):
                    continue
                n[k] += 1
                if k not in info:
                    if k == "cloud":
                        info[k] = (f"frame_id={m.header.frame_id!r} {m.width}x{m.height} pts, point_step={m.point_step}, "
                                   f"fields={[(f.name, f.datatype) for f in m.fields]}, bytes={len(m.data)}")
                    elif k == "imu":
                        info[k] = (f"frame_id={m.header.frame_id!r} acc=({m.linear_acceleration.x:.2f},"
                                   f"{m.linear_acceleration.y:.2f},{m.linear_acceleration.z:.2f})")
                    else:
                        info[k] = f"frame_id={m.header.frame_id!r} child={m.child_frame_id!r}"
        time.sleep(0.005)
    for k in rd:
        print(f"{k:6s} {n[k]:5d} msgs in {seconds:.0f} s = {n[k] / seconds:6.1f} Hz   {info.get(k, '(nothing received)')}")


def record(out):
    os.makedirs(out, exist_ok=True)
    dp = DomainParticipant(0)
    rd = readers(dp)
    files = {k: open(os.path.join(out, f"{k}.pkl"), "ab") for k in rd}
    n = {k: 0 for k in rd}
    stop = []
    signal.signal(signal.SIGINT, lambda *a: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *a: stop.append(1))
    t0 = time.time()
    last = t0
    print(f"recording to {out} (Ctrl-C to stop)", flush=True)
    bad = 0
    while not stop:
        for k, r in rd.items():
            try:
                samples = r.take(N=100)
            except Exception as e:  # never die on a reader hiccup
                print(f"take() failed on {k}: {e!r}", flush=True)
                time.sleep(0.05)
                continue
            for m in samples:
                if not hasattr(m, "header"):      # InvalidSample (publisher restarted / disposed): skip
                    bad += 1
                    if bad <= 5 or bad % 100 == 0:
                        print(f"skipped invalid sample on {k} (#{bad})", flush=True)
                    continue
                try:
                    pickle.dump(REC[k](m, time.time()), files[k], protocol=pickle.HIGHEST_PROTOCOL)
                    n[k] += 1
                except Exception as e:
                    print(f"record failed on {k}: {e!r}", flush=True)
        if time.time() - last > 10:
            last = time.time()
            print(f"{last - t0:6.0f} s  " + "  ".join(f"{k} {n[k]}" for k in n), flush=True)
        time.sleep(0.002)
    for f in files.values():
        f.close()
    dur = time.time() - t0
    print(f"stopped after {dur:.0f} s: " + "  ".join(f"{k} {n[k]} ({n[k] / max(dur, 1e-6):.1f} Hz)" for k in n), flush=True)
    with open(os.path.join(out, "info.txt"), "w") as f:
        f.write(f"duration_s {dur:.1f}\n" + "".join(f"{k} {TOPICS[k][0]} {n[k]}\n" for k in n))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["probe", "record"])
    ap.add_argument("--out", default=None)
    ap.add_argument("--seconds", type=float, default=6.0)
    a = ap.parse_args()
    if a.mode == "probe":
        probe(a.seconds)
    else:
        record(a.out or os.path.expanduser(time.strftime("~/bags/g1_lidar_%Y-%m-%d_%H%M")))
