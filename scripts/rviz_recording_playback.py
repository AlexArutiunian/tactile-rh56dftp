#!/usr/bin/env python3
import argparse
import csv
import math
import time
from collections import defaultdict, deque
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from rh56_tactile_common import COLORS, RIGHT_URDF, link_transform, parse_urdf, quaternion_from_matrix


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
                x = float(row["x_m"])
                y = float(row["y_m"])
                z = float(row["z_m"])
            except (KeyError, TypeError, ValueError):
                continue
            frames[frame].append(
                {
                    "sensor": row.get("sensor", ""),
                    "group": row.get("group", "palm"),
                    "activation": activation,
                    "p": (x, y, z),
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


class RecordingPlaybackNode(Node):
    def __init__(self, args):
        super().__init__("rh56_tactile_recording_playback")
        self.args = args
        self.frame_id = args.frame
        self.joints, self.child_joint_by_link, self.visuals = parse_urdf(Path(args.urdf))
        self.frames = load_contact_csv(infer_csv_path(args.source))
        if not self.frames:
            raise RuntimeError("No contact points loaded")

        self.mesh_pub = self.create_publisher(MarkerArray, "rh56/right_hand_mesh", 1)
        self.taxel_pub = self.create_publisher(MarkerArray, "rh56/tactile_contacts", 5)
        self.status_pub = self.create_publisher(String, "rh56/tactile_status", 5)

        self.idx = 0
        self.history = deque(maxlen=max(0, args.history_frames))
        self.max_activation = max(p["activation"] for _, pts in self.frames for p in pts) or 1.0
        self.start_time = time.time()

        self.publish_mesh()
        self.get_logger().info(
            f"Loaded {len(self.frames)} frames from {infer_csv_path(args.source)}; "
            f"playing at {args.fps:.1f} fps, loop={args.loop}"
        )
        self.timer = self.create_timer(1.0 / args.fps, self.tick)

    def publish_mesh(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for idx, (link, mesh_path) in enumerate(sorted(self.visuals.items())):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = "right_hand_mesh"
            marker.id = idx
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            marker.mesh_resource = "file://" + str(mesh_path)
            marker.mesh_use_embedded_materials = False
            tf = link_transform(link, self.child_joint_by_link, {})
            marker.pose.position.x = tf[0][3]
            marker.pose.position.y = tf[1][3]
            marker.pose.position.z = tf[2][3]
            qw, qx, qy, qz = quaternion_from_matrix(tf)
            marker.pose.orientation.w = qw
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.scale.x = marker.scale.y = marker.scale.z = 1.0
            marker.color = rgba(0.68, 0.72, 0.78, 0.72)
            if "force_sensor" in link:
                marker.color = rgba(0.08, 0.09, 0.11, 0.95)
            markers.markers.append(marker)
        self.mesh_pub.publish(markers)

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
            point.x, point.y, point.z = p["p"]
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

    def tick(self):
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

        markers.markers.append(self.make_points_marker("active_taxels", marker_id, pts, self.args.marker_size, 0.98, stamp))
        marker_id += 1

        for stale_id in range(marker_id, self.args.history_frames + 3):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = "active_taxels" if stale_id == marker_id else "contact_history"
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", help="Visualization/session folder or contact_points_right_urdf.csv")
    parser.add_argument("--urdf", default=str(RIGHT_URDF))
    parser.add_argument("--frame", default="right_wrist_yaw_link")
    parser.add_argument("--fps", type=float, default=12.0)
    parser.add_argument("--marker-size", type=float, default=0.0045)
    parser.add_argument("--history-frames", type=int, default=10)
    parser.add_argument("--loop", action="store_true", default=True)
    parser.add_argument("--no-loop", dest="loop", action="store_false")
    parser.add_argument("--print-status", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = RecordingPlaybackNode(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
