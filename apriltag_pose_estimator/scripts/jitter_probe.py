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
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node

from apriltag_pose_estimator_msgs.srv import TargetPointPose


def _pv_to_py(pv):
    """ParameterValue -> Python native value."""
    t = pv.type
    if t == ParameterType.PARAMETER_BOOL:
        return pv.bool_value
    if t == ParameterType.PARAMETER_INTEGER:
        return pv.integer_value
    if t == ParameterType.PARAMETER_DOUBLE:
        return pv.double_value
    if t == ParameterType.PARAMETER_STRING:
        return pv.string_value
    if t == ParameterType.PARAMETER_BOOL_ARRAY:
        return list(pv.bool_array_value)
    if t == ParameterType.PARAMETER_INTEGER_ARRAY:
        return list(pv.integer_array_value)
    if t == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return list(pv.double_array_value)
    if t == ParameterType.PARAMETER_STRING_ARRAY:
        return list(pv.string_array_value)
    return None


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
    def __init__(self, service_name: str, group_name: str = ""):
        super().__init__("jitter_probe")
        self.client = self.create_client(TargetPointPose, service_name)
        self.group_name = group_name
        self.get_logger().info(f"waiting for service: {service_name}")
        if not self.client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"service not available: {service_name}")
        self.get_logger().info("service ready")

    def call_once(self) -> TargetPointPose.Response:
        req = TargetPointPose.Request()
        req.request_time = TimeMsg(sec=0, nanosec=0)  # always older than latest
        req.group_name = self.group_name  # "" → first/default group
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        return future.result()

    def fetch_params(self, target_node: str, names: list, timeout_sec: float = 3.0):
        """Query target_node for parameter values via standard ROS2 param service."""
        cli = self.create_client(GetParameters, f"/{target_node}/get_parameters")
        if not cli.wait_for_service(timeout_sec=timeout_sec):
            return None
        req = GetParameters.Request()
        req.names = names
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        res = future.result()
        if res is None:
            return None
        return {name: _pv_to_py(pv) for name, pv in zip(names, res.values)}


def _node_from_service(service_name: str) -> str:
    """Extract node name from a fully-qualified service name (first path segment)."""
    parts = service_name.strip("/").split("/")
    return parts[0] if parts else ""


def _print_node_config(params: dict, group_name: str, group_names: list) -> None:
    """Pretty-print the relevant subset of pose_estimator_node parameters (multi-group)."""
    print()
    print("node configuration:")
    print(f"  group_names    : {group_names}")
    print(f"  group          : {group_name}")
    print(f"  tag_size       : {params.get('tag_size')} m")
    print(f"  tag_family     : {params.get('tag_family')}")
    print(f"  marker_ids     : {params.get('marker_ids')}")
    print(f"  base_marker_id : {params.get('base_marker_id')}")
    offsets = params.get("marker_offsets")
    ids = params.get("marker_ids") or []
    if offsets is not None and len(ids) > 0 and len(offsets) == len(ids) * 6:
        print("  marker_offsets (T_base <- marker_i):")
        for i, mid in enumerate(ids):
            base = i * 6
            x, y, z = offsets[base + 0], offsets[base + 1], offsets[base + 2]
            r, p, yw = offsets[base + 3], offsets[base + 4], offsets[base + 5]
            print(
                f"    id={mid:<4d} t=({x:+.4f}, {y:+.4f}, {z:+.4f}) m"
                f"  rpy=({r:+.4f}, {p:+.4f}, {yw:+.4f}) rad"
            )
    else:
        print(f"  marker_offsets : {offsets}")
    print()


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
        "--group",
        default="",
        help="group name to query (default: '' → first/default group)",
    )
    parser.add_argument(
        "--log",
        default=None,
        help="path to log file (default: /ros2_ws/jitter_probe_<timestamp>.log)",
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
    parser.add_argument(
        "--node",
        default=None,
        help="node name to query for parameters "
        "(default: derived from --service path)",
    )
    parser.add_argument(
        "--no-params",
        action="store_true",
        help="skip querying node parameters at startup",
    )
    args = parser.parse_args()

    # ---------- Log/CSV setup ----------
    log_fp = None
    csv_fp = None
    csv_writer = None
    if not args.no_log:
        ts = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = (
            Path(args.log) if args.log else Path(f"/ros2_ws/jitter_probe_{ts}.log")
        )
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
    probe = JitterProbe(args.service, group_name=args.group)

    # ---------- Query node parameters (best-effort, multi-group) ----------
    if not args.no_params:
        target_node = args.node or _node_from_service(args.service)
        # Step 1: get group_names + common params
        common = probe.fetch_params(
            target_node, ["group_names", "tag_size", "tag_family"]
        )
        if common is None:
            print(f"warning: could not fetch parameters from /{target_node}")
        else:
            group_names = common.get("group_names") or []
            # Resolve target group: --group if given, else first group
            target_group = (
                args.group if args.group else (group_names[0] if group_names else "")
            )
            # Step 2: fetch group-scoped params using dotted-name
            group_params = {}
            if target_group:
                prefix = f"groups.{target_group}."
                group_params = (
                    probe.fetch_params(
                        target_node,
                        [
                            prefix + "marker_ids",
                            prefix + "base_marker_id",
                            prefix + "marker_offsets",
                        ],
                    )
                    or {}
                )
                # Strip prefix for display
                group_params = {k[len(prefix) :]: v for k, v in group_params.items()}
            print(f"queried node : /{target_node}")
            _print_node_config(
                {**common, **group_params},
                target_group,
                group_names,
            )

    xs, ys, zs = [], [], []
    rolls, pitches, yaws = [], [], []
    fail_count = 0

    print(
        f"calling {args.count} times, interval={args.interval}s, "
        f"group='{args.group or '(default)'}'"
    )
    t0 = time.time()
    frame_id_printed = False
    for k in range(args.count):
        resp = probe.call_once()
        if resp is not None and resp.success and not frame_id_printed:
            print(
                f"response frame_id: {resp.poses.header.frame_id}  "
                f"(poses count: {len(resp.poses.poses)})"
            )
            frame_id_printed = True
        if resp is None or not resp.success:
            fail_count += 1
            msg = resp.message if resp is not None else "(timeout)"
            print(f"  [{k:3d}] FAIL: {msg}")
            if csv_writer is not None:
                csv_writer.writerow([k, 0, "", "", "", "", "", "", msg])
        else:
            # multi-group v3.0.0+: PoseArray always contains exactly one pose (base marker)
            p = resp.poses.poses[0]
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
