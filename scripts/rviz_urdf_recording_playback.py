#!/usr/bin/env python3
import argparse
import csv
import math
from collections import defaultdict, deque
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, String
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from rh56_tactile_common import (
    COLORS,
    RIGHT_URDF,
    SENSOR_TO_LINK,
    joint_values_from_positions,
    link_transform,
    matmul,
    parse_urdf,
    quaternion_from_matrix,
    rot_axis,
    transform,
)


def rgba(r, g, b, a):
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


def load_contact_csv(path):
    frames = defaultdict(list)
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                frame = int(float(row["frame"]))
                activation = float(row.get("activation", 0.0))
                p = (float(row["x_m"]), float(row["y_m"]), float(row["z_m"]))
            except (KeyError, TypeError, ValueError):
                continue
            frames[frame].append(
                {
                    "sensor": row.get("sensor", ""),
                    "group": row.get("group", "palm"),
                    "activation": activation,
                    "p": p,
                }
            )
    return [(frame, frames[frame]) for frame in sorted(frames)]


def infer_csv_path(source):
    source = Path(source).expanduser().resolve()
    if source.is_file():
        return source
    csv_path = source / "contact_points_right_urdf.csv"
    if not csv_path.exists():
        raise FileNotFoundError(f"No contact_points_right_urdf.csv in {source}")
    return csv_path


class UrdfRecordingPlaybackNode(Node):
    def __init__(self, args):
        super().__init__("rh56_urdf_recording_playback")
        self.args = args
        self.frame_id = args.frame
        self.joints, self.child_joint_by_link, _ = parse_urdf(Path(args.urdf))
        self.frames = load_contact_csv(infer_csv_path(args.source))
        if not self.frames:
            raise RuntimeError("No contact points loaded")

        self.tf_pub = StaticTransformBroadcaster(self)
        description_qos = QoSProfile(depth=1)
        description_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        description_qos.reliability = ReliabilityPolicy.RELIABLE
        self.robot_description_pub = self.create_publisher(String, "robot_description", description_qos)
        self.taxel_pub = self.create_publisher(MarkerArray, "rh56/tactile_contacts", 5)
        self.status_pub = self.create_publisher(String, "rh56/tactile_status", 5)
        self.robot_description = Path(args.urdf).read_text(encoding="utf-8")

        self.idx = 0
        self.history = deque(maxlen=max(0, args.history_frames))
        self.max_activation = max(p["activation"] for _, pts in self.frames for p in pts) or 1.0
        self.positions = [float(x) for x in args.positions.split(",")]
        if len(self.positions) != 6:
            raise ValueError("--positions must contain 6 comma-separated values")
        self.sensor_normals = self.compute_sensor_normals()

        self.get_logger().info(
            f"URDF RobotModel playback: {len(self.frames)} frames, fps={args.fps:.1f}, "
            f"source={infer_csv_path(args.source)}"
        )
        self.timer = self.create_timer(1.0 / args.fps, self.tick)
        self.publish_robot_description()
        self.publish_tf()

    def publish_robot_description(self):
        msg = String()
        msg.data = self.robot_description
        self.robot_description_pub.publish(msg)

    def publish_tf(self):
        joint_values = joint_values_from_positions(self.positions, self.joints)
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

    def compute_sensor_normals(self):
        joint_values = joint_values_from_positions(self.positions, self.joints)
        normals = {}
        for sensor, link in SENSOR_TO_LINK.items():
            tf = link_transform(link, self.child_joint_by_link, joint_values)
            nx, ny, nz = tf[0][2], tf[1][2], tf[2][2]
            norm = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
            normals[sensor] = (nx / norm, ny / norm, nz / norm)
        return normals

    def lifted_point(self, p):
        x, y, z = p["p"]
        nx, ny, nz = self.sensor_normals.get(p["sensor"], (0.0, 0.0, 1.0))
        lift = self.args.contact_lift
        return x + nx * lift, y + ny * lift, z + nz * lift

    def make_points_marker(self, ns, marker_id, pts, scale, alpha, stamp):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = scale
        for p in pts:
            point = Point()
            point.x, point.y, point.z = self.lifted_point(p)
            marker.points.append(point)
            base = COLORS.get(p["group"], (0.1, 0.1, 0.1))
            k = min(1.0, math.sqrt(max(0.0, p["activation"]) / self.max_activation))
            marker.colors.append(
                rgba(
                    0.92 * (1 - k) + base[0] * k,
                    0.92 * (1 - k) + base[1] * k,
                    0.92 * (1 - k) + base[2] * k,
                    alpha,
                )
            )
        return marker

    def make_halo_marker(self, ns, marker_id, pts, scale, alpha, stamp):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = scale
        for p in pts:
            point = Point()
            point.x, point.y, point.z = self.lifted_point(p)
            marker.points.append(point)
            marker.colors.append(rgba(1.0, 1.0, 1.0, alpha))
        return marker

    def publish_contacts(self):
        if self.idx >= len(self.frames):
            if not self.args.loop:
                return
            self.idx = 0
            self.history.clear()

        frame, pts = self.frames[self.idx]
        self.history.append((frame, pts))
        self.idx += 1

        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        marker_id = 0

        if self.args.history_frames:
            for age, (_, old_pts) in enumerate(reversed(list(self.history)[:-1]), start=1):
                fade = max(0.08, 0.38 * (1.0 - age / max(1, self.args.history_frames)))
                markers.markers.append(
                    self.make_points_marker("contact_history", marker_id, old_pts, self.args.marker_size * 0.65, fade, stamp)
                )
                marker_id += 1

        if not self.args.no_halo:
            markers.markers.append(self.make_halo_marker("contact_halo", marker_id, pts, self.args.marker_size * 1.8, 0.42, stamp))
            marker_id += 1
        markers.markers.append(self.make_points_marker("active_taxels", marker_id, pts, self.args.marker_size, 1.0, stamp))
        marker_id += 1

        for stale_id in range(marker_id, self.args.history_frames + 4):
            for ns in ("active_taxels", "contact_history", "contact_halo"):
                marker = Marker()
                marker.header.frame_id = self.frame_id
                marker.header.stamp = stamp
                marker.ns = ns
                marker.id = stale_id
                marker.action = Marker.DELETE
                markers.markers.append(marker)

        self.taxel_pub.publish(markers)

        by_sensor = defaultdict(int)
        for p in pts:
            by_sensor[p["sensor"]] += 1
        top = sorted(by_sensor.items(), key=lambda item: item[1], reverse=True)[:5]
        msg = String()
        msg.data = f"recorded_frame={frame} active_points={len(pts)} top={top}"
        self.status_pub.publish(msg)
        if self.args.print_status and (self.idx == 1 or self.idx % max(1, int(self.args.fps)) == 0):
            self.get_logger().info(msg.data)

    def tick(self):
        self.publish_contacts()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", help="Visualization/session folder or contact_points_right_urdf.csv")
    parser.add_argument("--urdf", default=str(RIGHT_URDF))
    parser.add_argument("--frame", default="right_wrist_yaw_link")
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--marker-size", type=float, default=0.006)
    parser.add_argument("--contact-lift", type=float, default=0.003, help="Meters to lift contact markers along sensor normal for RViz readability.")
    parser.add_argument("--no-halo", action="store_true", help="Show only colored contact points without white halo spheres.")
    parser.add_argument("--history-frames", type=int, default=12)
    parser.add_argument("--positions", default="0,0,0,0,0,0", help="Static POS_SET-style values for URDF joints.")
    parser.add_argument("--loop", action="store_true", default=True)
    parser.add_argument("--no-loop", dest="loop", action="store_false")
    parser.add_argument("--print-status", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = UrdfRecordingPlaybackNode(args)
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
