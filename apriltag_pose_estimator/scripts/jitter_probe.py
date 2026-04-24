#!/usr/bin/env python3
"""target_point_pose 서비스를 반복 호출해 응답 jitter (mm, deg) 통계를 출력."""

import argparse
import csv
import datetime as _dt
import math
import statistics
import sys
import time
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.node import Node

from apriltag_pose_estimator_msgs.srv import TargetPointPose


class _Tee:
    """Mirror writes to multiple streams (stdout + log file)."""

    def __init__(self, *streams):
        self.streams = streams

    def write(self, s):
        for st in self.streams:
            st.write(s)
            st.flush()

    def flush(self):
        for st in self.streams:
            st.flush()


def quat_to_rpy_deg(qx: float, qy: float, qz: float, qw: float) -> tuple:
    """Quaternion -> (roll, pitch, yaw) in degrees (intrinsic Z-Y-X)."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    deg = 180.0 / math.pi
    return roll * deg, pitch * deg, yaw * deg


class JitterProbe(Node):
    def __init__(self, service_name: str):
        super().__init__("jitter_probe")
        self.client = self.create_client(TargetPointPose, service_name)
        self.get_logger().info(f"waiting for service: {service_name}")
        if not self.client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"service not available: {service_name}")
        self.get_logger().info("service ready")

    def call_once(self) -> TargetPointPose.Response:
        req = TargetPointPose.Request()
        req.request_time = TimeMsg(sec=0, nanosec=0)  # always older than latest
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        return future.result()


def summarize(values: list, label: str, unit: str) -> str:
    if not values:
        return f"  {label:6s} (no data)"
    mean = statistics.fmean(values)
    std = statistics.pstdev(values) if len(values) > 1 else 0.0
    vmin = min(values)
    vmax = max(values)
    p2p = vmax - vmin
    return (
        f"  {label:6s} mean={mean:+10.4f} {unit}  std={std:8.4f}  "
        f"min={vmin:+10.4f}  max={vmax:+10.4f}  p2p={p2p:8.4f}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-n",
        "--count",
        type=int,
        default=100,
        help="number of service calls (default: 100)",
    )
    parser.add_argument(
        "-i",
        "--interval",
        type=float,
        default=0.1,
        help="seconds between calls (default: 0.1)",
    )
    parser.add_argument(
        "--service",
        default="/pose_estimator_node/target_point_pose",
        help="service name",
    )
    parser.add_argument(
        "--point-index",
        type=int,
        default=0,
        help="which target point in the response to track (default: 0)",
    )
    parser.add_argument(
        "--log",
        default=None,
        help="path to log file (default: /tmp/jitter_probe_<timestamp>.log)",
    )
    parser.add_argument(
        "--csv",
        default=None,
        help="path to raw CSV (default: alongside log file with .csv suffix)",
    )
    parser.add_argument(
        "--no-log",
        action="store_true",
        help="disable log/csv files entirely",
    )
    args = parser.parse_args()

    # ---------- Log/CSV setup ----------
    log_fp = None
    csv_fp = None
    csv_writer = None
    if not args.no_log:
        ts = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = Path(args.log) if args.log else Path(f"/tmp/jitter_probe_{ts}.log")
        csv_path = Path(args.csv) if args.csv else log_path.with_suffix(".csv")
        log_path.parent.mkdir(parents=True, exist_ok=True)
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        log_fp = open(log_path, "w", buffering=1)
        sys.stdout = _Tee(sys.__stdout__, log_fp)
        csv_fp = open(csv_path, "w", newline="")
        csv_writer = csv.writer(csv_fp)
        csv_writer.writerow(
            [
                "idx",
                "ok",
                "x_mm",
                "y_mm",
                "z_mm",
                "roll_deg",
                "pitch_deg",
                "yaw_deg",
                "message",
            ]
        )
        print(f"log : {log_path}")
        print(f"csv : {csv_path}")

    rclpy.init()
    probe = JitterProbe(args.service)

    xs, ys, zs = [], [], []
    rolls, pitches, yaws = [], [], []
    fail_count = 0

    print(
        f"calling {args.count} times, interval={args.interval}s, "
        f"point_index={args.point_index}"
    )
    t0 = time.time()
    for k in range(args.count):
        resp = probe.call_once()
        if resp is None or not resp.success:
            fail_count += 1
            msg = resp.message if resp is not None else "(timeout)"
            print(f"  [{k:3d}] FAIL: {msg}")
            if csv_writer is not None:
                csv_writer.writerow([k, 0, "", "", "", "", "", "", msg])
        else:
            poses = resp.poses.poses
            if args.point_index >= len(poses):
                fail_count += 1
                msg = f"point_index out of range (have {len(poses)})"
                print(f"  [{k:3d}] FAIL: {msg}")
                if csv_writer is not None:
                    csv_writer.writerow([k, 0, "", "", "", "", "", "", msg])
            else:
                p = poses[args.point_index]
                x_mm = p.position.x * 1000.0
                y_mm = p.position.y * 1000.0
                z_mm = p.position.z * 1000.0
                r, pi, ya = quat_to_rpy_deg(
                    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
                )
                xs.append(x_mm)
                ys.append(y_mm)
                zs.append(z_mm)
                rolls.append(r)
                pitches.append(pi)
                yaws.append(ya)
                if csv_writer is not None:
                    csv_writer.writerow(
                        [
                            k,
                            1,
                            f"{x_mm:.4f}",
                            f"{y_mm:.4f}",
                            f"{z_mm:.4f}",
                            f"{r:.4f}",
                            f"{pi:.4f}",
                            f"{ya:.4f}",
                            "",
                        ]
                    )
        time.sleep(args.interval)

    elapsed = time.time() - t0
    ok = len(xs)
    print()
    print(
        f"summary: {ok}/{args.count} ok, {fail_count} fail, " f"elapsed={elapsed:.2f}s"
    )
    print()
    print("position (mm):")
    print(summarize(xs, "x", "mm"))
    print(summarize(ys, "y", "mm"))
    print(summarize(zs, "z", "mm"))
    print()
    print("orientation (deg):")
    print(summarize(rolls, "roll", "deg"))
    print(summarize(pitches, "pitch", "deg"))
    print(summarize(yaws, "yaw", "deg"))

    probe.destroy_node()
    rclpy.shutdown()

    if log_fp is not None:
        sys.stdout = sys.__stdout__
        log_fp.close()
    if csv_fp is not None:
        csv_fp.close()


if __name__ == "__main__":
    main()
