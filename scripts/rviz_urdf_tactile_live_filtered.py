#!/usr/bin/env python3
import argparse
import logging
import math
import time
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
ROOT = THIS_DIR.parent

import rclpy
from geometry_msgs.msg import Point, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

logging.getLogger("RH56DFTP").setLevel(logging.ERROR)
logging.getLogger("pymodbus").setLevel(logging.ERROR)

from RH56DFTP.RH56DFTP_TCP import RH56DFTP_TCP
from tactile_noise_filter import TactileNoiseFilter, active_counts, positive_deltas
from rh56_tactile_common import (
    COLORS,
    LEFT_URDF,
    RIGHT_URDF,
    baseline_means,
    configure_hand_side,
    joint_values_from_positions,
    matmul,
    parse_urdf,
    quaternion_from_matrix,
    read_positions,
    read_tactile,
    rot_axis,
    tactile_points,
    transform,
)


def rgba(r, g, b, a):
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


class FilteredUrdfTactileLiveNode(Node):
    def __init__(self, args):
        super().__init__("rh56_urdf_tactile_live_filtered")
        self.args = args
        configure_hand_side(args.side)
        self.frame_id = args.frame
        self.client = RH56DFTP_TCP(args.ip, args.port)
        self.joints, _, _ = parse_urdf(Path(args.urdf))
        self.positions = [float(x) for x in args.positions.split(",")]
        if len(self.positions) != 6:
            raise ValueError("--positions must contain 6 comma-separated values")

        self.tf_pub = TransformBroadcaster(self)
        description_qos = QoSProfile(depth=1)
        description_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        description_qos.reliability = ReliabilityPolicy.RELIABLE
        self.robot_description_pub = self.create_publisher(String, "robot_description", description_qos)
        self.taxel_pub = self.create_publisher(MarkerArray, "rh56/tactile_contacts", 5)
        self.status_pub = self.create_publisher(String, "rh56/tactile_status", 5)
        self.robot_description = Path(args.urdf).read_text(encoding="utf-8")

        self.get_logger().info(f"Filtered baseline: keep {args.side} hand untouched for {args.baseline_sec:.1f}s")
        samples = []
        baseline_frames = max(1, int(args.baseline_sec * args.hz))
        for _ in range(baseline_frames):
            samples.append(read_tactile(self.client))
            time.sleep(1.0 / args.hz)

        self.baseline = baseline_means(samples)
        self.noise_filter = TactileNoiseFilter(
            samples,
            min_delta=args.threshold,
            sigma=args.sigma,
            idle_margin=args.idle_margin,
            min_component_size=args.min_component_size,
            min_component_size_3x3=args.min_component_size_3x3,
            temporal_frames=args.temporal_frames,
            strong_delta=args.strong_delta,
        )
        self.get_logger().info("Filtered baseline ready. RViz now shows only persistent connected contact clusters.")

        self.publish_robot_description()
        self.timer = self.create_timer(1.0 / args.hz, self.tick)

    @property
    def _child_joint_by_link(self):
        if not hasattr(self, "_child_joint_by_link_cache"):
            _, child_joint_by_link, _ = parse_urdf(Path(self.args.urdf))
            self._child_joint_by_link_cache = child_joint_by_link
        return self._child_joint_by_link_cache

    def current_positions(self):
        if self.args.read_pos_set:
            return read_positions(self.client, self.positions)
        return self.positions

    def publish_robot_description(self):
        msg = String()
        msg.data = self.robot_description
        self.robot_description_pub.publish(msg)

    def publish_tf(self, positions):
        joint_values = joint_values_from_positions(positions, self.joints)
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for joint in self.joints.values():
            m = transform(joint["xyz"], joint["rpy"])
            if joint["type"] == "revolute":
                m = matmul(m, rot_axis(joint["axis"], joint_values.get(joint["name"], 0.0)))
            msg = TransformStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = joint["parent"]
            msg.child_frame_id = joint["child"]
            msg.transform.translation.x = m[0][3]
            msg.transform.translation.y = m[1][3]
            msg.transform.translation.z = m[2][3]
            qw, qx, qy, qz = quaternion_from_matrix(m)
            msg.transform.rotation.w = qw
            msg.transform.rotation.x = qx
            msg.transform.rotation.y = qy
            msg.transform.rotation.z = qz
            transforms.append(msg)
        self.tf_pub.sendTransform(transforms)

    def publish_contacts(self, points):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        by_group = {}
        max_delta = max([p["delta"] for p in points] + [1.0])
        for p in points:
            by_group.setdefault(p["group"], []).append(p)

        marker_id = 0
        for group, pts in sorted(by_group.items()):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = "filtered_taxels"
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.SPHERE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = self.args.marker_size
            base = COLORS.get(group, (0.2, 0.2, 0.2))
            for p in pts:
                point = Point()
                point.x, point.y, point.z = p["p"]
                marker.points.append(point)
                if self.args.flat_colors:
                    marker.colors.append(rgba(base[0], base[1], base[2], 1.0))
                else:
                    k = min(1.0, math.sqrt(p["delta"] / max_delta))
                    marker.colors.append(rgba(
                        0.85 * (1 - k) + base[0] * k,
                        0.85 * (1 - k) + base[1] * k,
                        0.85 * (1 - k) + base[2] * k,
                        1.0,
                    ))
            markers.markers.append(marker)

        for stale_id in range(marker_id, 12):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = "filtered_taxels"
            marker.id = stale_id
            marker.action = Marker.DELETE
            markers.markers.append(marker)
        self.taxel_pub.publish(markers)

    def tick(self):
        positions = self.current_positions()
        self.publish_robot_description()
        self.publish_tf(positions)
        sample = read_tactile(self.client)
        raw_delta = positive_deltas(sample, self.noise_filter.stats)
        filtered_raw, filtered_delta, _debug = self.noise_filter.filter_sample(sample)
        points = tactile_points(filtered_raw, self.baseline, self.joints, self._child_joint_by_link, positions, 1.0)
        self.publish_contacts(points)

        raw_counts = active_counts(raw_delta, self.args.threshold)
        filtered_counts = active_counts(filtered_delta, 1.0)
        top = sorted(filtered_counts.items(), key=lambda item: item[1], reverse=True)[:6]
        msg = String()
        msg.data = (
            f"side={self.args.side} raw_taxels={sum(raw_counts.values())} "
            f"filtered_taxels={sum(filtered_counts.values())} top={top}"
        )
        self.status_pub.publish(msg)
        if self.args.print_status:
            self.get_logger().info(msg.data)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="192.168.123.211")
    parser.add_argument("--port", type=int, default=6000)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--urdf", default=None)
    parser.add_argument("--frame", default=None)
    parser.add_argument("--hz", type=float, default=8.0)
    parser.add_argument("--baseline-sec", type=float, default=3.0)
    parser.add_argument("--threshold", type=float, default=80.0)
    parser.add_argument("--sigma", type=float, default=6.0)
    parser.add_argument("--idle-margin", type=float, default=3.0)
    parser.add_argument("--min-component-size", type=int, default=2)
    parser.add_argument("--min-component-size-3x3", type=int, default=1)
    parser.add_argument("--temporal-frames", type=int, default=2)
    parser.add_argument("--strong-delta", type=float, default=1500.0)
    parser.add_argument("--marker-size", type=float, default=0.008)
    parser.add_argument("--positions", default="0,0,0,0,0,0")
    parser.add_argument("--read-pos-set", action="store_true")
    parser.add_argument("--flat-colors", action="store_true")
    parser.add_argument("--print-status", action="store_true")
    args = parser.parse_args()
    if args.urdf is None:
        args.urdf = str(LEFT_URDF if args.side == "left" else RIGHT_URDF)
    if args.frame is None:
        args.frame = f"{args.side}_wrist_yaw_link"

    rclpy.init()
    node = FilteredUrdfTactileLiveNode(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
