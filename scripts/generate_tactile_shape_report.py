#!/usr/bin/env python3
import argparse
import csv
import json
import math
import random
import statistics
import struct
import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parent
DEFAULT_SESSION = ROOT / "scripts" / "tactile_grasp_dataset" / "roulette_grasp_01"
DEFAULT_URDF = ROOT / "urdf" / "inspire_hand" / "FTP_left_hand.urdf"
DEFAULT_OUT = ROOT / "tactile_shape_report_visualizations"

CONTROL_TO_JOINT = {
    0: "left_little_1_joint",
    1: "left_ring_1_joint",
    2: "left_middle_1_joint",
    3: "left_index_1_joint",
    4: "left_thumb_2_joint",
    5: "left_thumb_1_joint",
}

SENSOR_TO_LINK = {
    "small_tip_3x3": "left_little_force_sensor_3",
    "small_tip_12x8": "left_little_force_sensor_2",
    "small_pad_10x8": "left_little_force_sensor_1",
    "ring_tip_3x3": "left_ring_force_sensor_3",
    "ring_tip_12x8": "left_ring_force_sensor_2",
    "ring_pad_10x8": "left_ring_force_sensor_1",
    "middle_tip_3x3": "left_middle_force_sensor_3",
    "middle_tip_12x8": "left_middle_force_sensor_2",
    "middle_pad_10x8": "left_middle_force_sensor_1",
    "index_tip_3x3": "left_index_force_sensor_3",
    "index_tip_12x8": "left_index_force_sensor_2",
    "index_pad_10x8": "left_index_force_sensor_1",
    "thumb_tip_3x3": "left_thumb_force_sensor_4",
    "thumb_tip_12x8": "left_thumb_force_sensor_3",
    "thumb_middle_3x3": "left_thumb_force_sensor_2",
    "thumb_pad_12x8": "left_thumb_force_sensor_1",
    "palm_8x14": "left_palm_force_sensor",
}

SENSOR_PLANE_SIZE_M = {
    "3x3": (0.014, 0.014),
    "12x8": (0.018, 0.026),
    "10x8": (0.020, 0.030),
    "8x14": (0.052, 0.040),
}

COLORS = {
    "thumb": (207, 62, 54),
    "index": (30, 111, 159),
    "middle": (34, 139, 78),
    "ring": (143, 85, 181),
    "small": (210, 138, 16),
    "palm": (55, 55, 55),
}

THUMB_ASSUMED_LINKS = (
    ("left_thumb_force_sensor_4", "thumb_final_tip_contact_assumption", 0.018, 0.026, 5, 5),
)

HAND_SIDE = "left"
WRIST_LINK = "left_wrist_yaw_link"
CONTACT_POINTS_FILENAME = "contact_points_left_urdf.csv"
SENSOR_LAYOUT_FILENAME = "01_left_hand_sensor_layout.png"


def configure_hand_side(side):
    global HAND_SIDE, WRIST_LINK, CONTACT_POINTS_FILENAME, SENSOR_LAYOUT_FILENAME
    global CONTROL_TO_JOINT, SENSOR_TO_LINK, THUMB_ASSUMED_LINKS
    if side not in {"left", "right"}:
        raise ValueError(f"Unsupported hand side: {side}")
    HAND_SIDE = side
    WRIST_LINK = f"{side}_wrist_yaw_link"
    CONTACT_POINTS_FILENAME = f"contact_points_{side}_urdf.csv"
    SENSOR_LAYOUT_FILENAME = f"01_{side}_hand_sensor_layout.png"
    CONTROL_TO_JOINT = {
        0: f"{side}_little_1_joint",
        1: f"{side}_ring_1_joint",
        2: f"{side}_middle_1_joint",
        3: f"{side}_index_1_joint",
        4: f"{side}_thumb_2_joint",
        5: f"{side}_thumb_1_joint",
    }
    SENSOR_TO_LINK = {
        "small_tip_3x3": f"{side}_little_force_sensor_3",
        "small_tip_12x8": f"{side}_little_force_sensor_2",
        "small_pad_10x8": f"{side}_little_force_sensor_1",
        "ring_tip_3x3": f"{side}_ring_force_sensor_3",
        "ring_tip_12x8": f"{side}_ring_force_sensor_2",
        "ring_pad_10x8": f"{side}_ring_force_sensor_1",
        "middle_tip_3x3": f"{side}_middle_force_sensor_3",
        "middle_tip_12x8": f"{side}_middle_force_sensor_2",
        "middle_pad_10x8": f"{side}_middle_force_sensor_1",
        "index_tip_3x3": f"{side}_index_force_sensor_3",
        "index_tip_12x8": f"{side}_index_force_sensor_2",
        "index_pad_10x8": f"{side}_index_force_sensor_1",
        "thumb_tip_3x3": f"{side}_thumb_force_sensor_4",
        "thumb_tip_12x8": f"{side}_thumb_force_sensor_3",
        "thumb_middle_3x3": f"{side}_thumb_force_sensor_2",
        "thumb_pad_12x8": f"{side}_thumb_force_sensor_1",
        "palm_8x14": f"{side}_palm_force_sensor",
    }
    THUMB_ASSUMED_LINKS = (
        (f"{side}_thumb_force_sensor_4", "thumb_final_tip_contact_assumption", 0.018, 0.026, 5, 5),
    )


def load_jsonl(path):
    rows = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def matmul(a, b):
    out = [[0.0] * 4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            out[i][j] = sum(a[i][k] * b[k][j] for k in range(4))
    return out


def eye():
    return [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]


def rot_axis(axis, q):
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z) or 1.0
    x, y, z = x / n, y / n, z / n
    c, s = math.cos(q), math.sin(q)
    t = 1.0 - c
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0.0],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0.0],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = [[1, 0, 0, 0], [0, cr, -sr, 0], [0, sr, cr, 0], [0, 0, 0, 1]]
    ry = [[cp, 0, sp, 0], [0, 1, 0, 0], [-sp, 0, cp, 0], [0, 0, 0, 1]]
    rz = [[cy, -sy, 0, 0], [sy, cy, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    return matmul(matmul(rz, ry), rx)


def transform(xyz, rpy):
    m = rpy_matrix(rpy)
    m[0][3], m[1][3], m[2][3] = xyz
    return m


def apply(m, p):
    x, y, z = p
    return (
        m[0][0] * x + m[0][1] * y + m[0][2] * z + m[0][3],
        m[1][0] * x + m[1][1] * y + m[1][2] * z + m[1][3],
        m[2][0] * x + m[2][1] * y + m[2][2] * z + m[2][3],
    )


def parse_vec(s, default):
    if not s:
        return default
    return tuple(float(x) for x in s.split())


def parse_urdf(path):
    tree = ET.parse(path)
    joints = {}
    children = {}
    for j in tree.findall("joint"):
        name = j.attrib["name"]
        parent = j.find("parent").attrib["link"]
        child = j.find("child").attrib["link"]
        origin = j.find("origin")
        axis = j.find("axis")
        limit = j.find("limit")
        mimic = j.find("mimic")
        info = {
            "name": name,
            "type": j.attrib.get("type", "fixed"),
            "parent": parent,
            "child": child,
            "xyz": parse_vec(origin.attrib.get("xyz") if origin is not None else "", (0.0, 0.0, 0.0)),
            "rpy": parse_vec(origin.attrib.get("rpy") if origin is not None else "", (0.0, 0.0, 0.0)),
            "axis": parse_vec(axis.attrib.get("xyz") if axis is not None else "", (0.0, 0.0, 1.0)),
            "lower": float(limit.attrib.get("lower", "0")) if limit is not None else 0.0,
            "upper": float(limit.attrib.get("upper", "0")) if limit is not None else 0.0,
            "mimic": None,
        }
        if mimic is not None:
            info["mimic"] = (
                mimic.attrib["joint"],
                float(mimic.attrib.get("multiplier", "1")),
                float(mimic.attrib.get("offset", "0")),
            )
        joints[name] = info
        children[child] = info
    return joints, children


def parse_visual_meshes(path):
    tree = ET.parse(path)
    out = {}
    for link in tree.findall("link"):
        link_name = link.attrib["name"]
        visual = link.find("visual")
        if visual is None:
            continue
        geom = visual.find("geometry")
        mesh = geom.find("mesh") if geom is not None else None
        if mesh is None or "filename" not in mesh.attrib:
            continue
        out[link_name] = mesh.attrib["filename"]
    return out


def transform_direction(m, v):
    x, y, z = v
    return (
        m[0][0] * x + m[0][1] * y + m[0][2] * z,
        m[1][0] * x + m[1][1] * y + m[1][2] * z,
        m[2][0] * x + m[2][1] * y + m[2][2] * z,
    )


def read_stl_triangles(path, max_triangles=800):
    data = path.read_bytes()
    triangles = []
    # Binary STL: 80-byte header + uint32 count + 50 bytes per triangle.
    if len(data) >= 84:
        n = int.from_bytes(data[80:84], "little", signed=False)
        if 84 + n * 50 == len(data):
            step = max(1, math.ceil(n / max_triangles))
            for idx in range(0, n, step):
                off = 84 + idx * 50 + 12
                verts = []
                for _ in range(3):
                    vals = []
                    for j in range(3):
                        vals.append(struct.unpack("<f", data[off:off + 4])[0])
                        off += 4
                    verts.append(tuple(vals))
                triangles.append(tuple(verts))
            return triangles
    # Small fallback for ASCII STL.
    verts = []
    for line in data.decode("utf-8", errors="ignore").splitlines():
        line = line.strip()
        if line.startswith("vertex "):
            parts = line.split()
            verts.append(tuple(float(x) for x in parts[1:4]))
            if len(verts) == 3:
                triangles.append(tuple(verts))
                verts = []
    if len(triangles) > max_triangles:
        step = max(1, math.ceil(len(triangles) / max_triangles))
        triangles = triangles[::step]
    return triangles


def mesh_cap_for_link(link, quality):
    if quality == "full":
        return 1_000_000
    if quality == "high":
        if "base_link" in link:
            return 60_000
        if "force_sensor" in link:
            return 8_000
        return 12_000
    if "base_link" in link:
        return 9000
    if "force_sensor" in link:
        return 1400
    return 3600


def hand_mesh_data(urdf, frames, quality="normal"):
    joints, child_joint_by_link = parse_urdf(urdf)
    visual_meshes = parse_visual_meshes(urdf)
    mesh_root = urdf.parent
    max_frame = max(frames, key=lambda rec: sum(abs(x) for x in (rec.get("force_delta") or [])))
    joint_values = joint_values_from_positions(max_frame.get("positions", []), joints)
    meshes = []
    for link, filename in visual_meshes.items():
        mesh_path = (mesh_root / filename).resolve()
        if not mesh_path.exists():
            continue
        cap = mesh_cap_for_link(link, quality)
        tf = link_transform(link, child_joint_by_link, joint_values)
        triangles = read_stl_triangles(mesh_path, cap)
        x, y, z, i, j, k = [], [], [], [], [], []
        for tri in triangles:
            base = len(x)
            for v in tri:
                p = apply(tf, v)
                x.append(p[0] * 1000.0)
                y.append(p[1] * 1000.0)
                z.append(p[2] * 1000.0)
            i.append(base)
            j.append(base + 1)
            k.append(base + 2)
        if x:
            meshes.append({"link": link, "x": x, "y": y, "z": z, "i": i, "j": j, "k": k, "is_sensor": "force_sensor" in link})
    return meshes


def joint_values_from_positions(positions, joints):
    values = {}
    for idx, joint_name in CONTROL_TO_JOINT.items():
        if joint_name not in joints or idx >= len(positions):
            continue
        j = joints[joint_name]
        frac = max(0.0, min(1.0, float(positions[idx]) / 1800.0))
        values[joint_name] = j["lower"] + frac * (j["upper"] - j["lower"])
    changed = True
    while changed:
        changed = False
        for name, j in joints.items():
            if j["mimic"] and name not in values:
                parent, mul, off = j["mimic"]
                if parent in values:
                    values[name] = values[parent] * mul + off
                    changed = True
    return values


def link_transform(link, child_joint_by_link, joint_values):
    chain = []
    cur = link
    while cur in child_joint_by_link:
        j = child_joint_by_link[cur]
        chain.append(j)
        cur = j["parent"]
    m = eye()
    for j in reversed(chain):
        jm = transform(j["xyz"], j["rpy"])
        if j["type"] == "revolute":
            jm = matmul(jm, rot_axis(j["axis"], joint_values.get(j["name"], 0.0)))
        m = matmul(m, jm)
    return m


def sensor_group(name):
    return "small" if name.startswith("small") else name.split("_", 1)[0]


def infer_shape(name, n, metadata):
    spec = metadata.get("tactile_specs", {}).get(name, {})
    cols = int(spec.get("cols_label") or 1)
    rows = int(spec.get("rows_label") or 1)
    if cols and n % cols == 0:
        return n // cols, cols
    if rows and n % rows == 0:
        return rows, n // rows
    return 1, n


def plane_size(name):
    for key, size in SENSOR_PLANE_SIZE_M.items():
        if key in name:
            return size
    return (0.018, 0.018)


@lru_cache(maxsize=None)
def stl_bounds_for_link(link):
    path = ROOT / "urdf" / "inspire_hand" / "meshes" / f"{link}.STL"
    data = path.read_bytes()
    points = []
    if len(data) >= 84:
        tri_count = struct.unpack("<I", data[80:84])[0]
        if 84 + tri_count * 50 <= len(data):
            offset = 84
            for _ in range(tri_count):
                offset += 12
                for _ in range(3):
                    points.append(struct.unpack("<fff", data[offset:offset + 12]))
                    offset += 12
                offset += 2
    if not points:
        text = data.decode("utf-8", "ignore")
        for line in text.splitlines():
            parts = line.strip().split()
            if len(parts) == 4 and parts[0] == "vertex":
                points.append(tuple(float(x) for x in parts[1:]))
    if not points:
        return (-0.005, -0.005, 0.0), (0.005, 0.005, 0.0)
    mins = tuple(min(p[i] for p in points) for i in range(3))
    maxs = tuple(max(p[i] for p in points) for i in range(3))
    return mins, maxs


def sensor_taxel_local_point(name, row, col, rows, cols):
    link = SENSOR_TO_LINK[name]
    mins, maxs = stl_bounds_for_link(link)
    mx = 0.06 * (maxs[0] - mins[0])
    my = 0.06 * (maxs[1] - mins[1])
    x0, x1 = mins[0] + mx, maxs[0] - mx
    y0, y1 = mins[1] + my, maxs[1] - my
    x = x0 + ((col + 0.5) / max(1, cols)) * (x1 - x0)
    y = y0 + ((row + 0.5) / max(1, rows)) * (y1 - y0)
    z = maxs[2] + 0.00018
    return x, y, z


def expected_taxel_count(name, metadata=None):
    if metadata:
        spec = metadata.get("tactile_specs", {}).get(name, {})
        rows = int(spec.get("rows_label") or 0)
        cols = int(spec.get("cols_label") or 0)
        if rows and cols:
            return rows * cols
    if name in SENSOR_TO_LINK:
        if "3x3" in name:
            return 9
        if "12x8" in name:
            return 96
        if "10x8" in name:
            return 80
        if "8x14" in name:
            return 112
    return None


def decoded_taxels(name, values, metadata=None):
    vals = list(values or [])
    expected = expected_taxel_count(name, metadata)
    if expected and len(vals) >= expected:
        return vals[:expected]
    return vals


def baseline_means(frames, metadata=None):
    sums = {}
    counts = {}
    for rec in frames:
        for name, values in rec.get("sensors_raw", {}).items():
            values = decoded_taxels(name, values, metadata)
            sums.setdefault(name, [0.0] * len(values))
            counts[name] = counts.get(name, 0) + 1
            for i, v in enumerate(values):
                if i < len(sums[name]):
                    sums[name][i] += float(v)
    return {name: [v / max(1, counts[name]) for v in vals] for name, vals in sums.items()}


def select_shape_frames(frames, frame_totals, top_frame_fraction, min_selected_frames):
    hold_idxs = [i for i, rec in enumerate(frames) if rec.get("phase") == "hold"]
    candidates = hold_idxs if hold_idxs else list(range(len(frames)))
    if not candidates:
        return set()
    k = max(min_selected_frames, int(math.ceil(len(candidates) * top_frame_fraction)))
    k = min(len(candidates), k)
    ranked = sorted(candidates, key=lambda i: frame_totals[i], reverse=True)
    selected = sorted(ranked[:k])
    return set(selected)


def make_assumed_thumb_contacts(frames, joints, child_joint_by_link, activation):
    if not frames:
        return []
    final_frame_idx = len(frames) - 1
    joint_values = joint_values_from_positions(frames[-1].get("positions", []), joints)
    points = []
    for link, sensor_name, sx, sy, rows, cols in THUMB_ASSUMED_LINKS:
        tf = link_transform(link, child_joint_by_link, joint_values)
        for r in range(rows):
            for c in range(cols):
                x = ((c + 0.5) / cols - 0.5) * sx
                y = ((r + 0.5) / rows - 0.5) * sy
                p = apply(tf, (x, y, 0.0015))
                points.append(
                    {
                        "p": p,
                        "activation": activation,
                        "sensor": sensor_name,
                        "frame": final_frame_idx,
                        "group": "thumb",
                        "assumed": True,
                    }
                )
    return points


def frame_contact_points(rec, frame_idx, metadata, baseline, joints, child_joint_by_link, activation_threshold):
    joint_values = joint_values_from_positions(rec.get("positions", []), joints)
    points = []
    for name, raw_values in rec.get("sensors_raw", {}).items():
        if name not in SENSOR_TO_LINK:
            continue
        raw_values = decoded_taxels(name, raw_values, metadata)
        rows, cols = infer_shape(name, len(raw_values), metadata)
        link = SENSOR_TO_LINK[name]
        tf = link_transform(link, child_joint_by_link, joint_values)
        base_vals = baseline.get(name, [0.0] * len(raw_values))
        for i, raw in enumerate(raw_values):
            act = max(0.0, float(raw) - (base_vals[i] if i < len(base_vals) else 0.0) - activation_threshold)
            if act <= 0.0:
                continue
            r, c = divmod(i, cols)
            p = apply(tf, sensor_taxel_local_point(name, r, c, rows, cols))
            points.append({"p": p, "activation": act, "sensor": name, "frame": frame_idx, "group": sensor_group(name), "assumed": False})
    return points


def collect_contact_points(
    session,
    urdf,
    activation_threshold,
    top_frame_fraction=0.35,
    min_selected_frames=12,
    assume_thumb_final_contact=False,
):
    metadata = json.loads((session / "metadata.json").read_text(encoding="utf-8"))
    frames = load_jsonl(session / "frames_raw.jsonl")
    baseline = baseline_means(load_jsonl(session / "baseline_raw.jsonl"), metadata)
    joints, child_joint_by_link = parse_urdf(urdf)

    frame_points = []
    frame_totals = []
    max_frame = max(frames, key=lambda rec: sum(abs(x) for x in (rec.get("force_delta") or [])))
    layout_positions = {}

    for frame_idx, rec in enumerate(frames):
        points_this_frame = frame_contact_points(rec, frame_idx, metadata, baseline, joints, child_joint_by_link, activation_threshold)
        frame_total = sum(p["activation"] for p in points_this_frame)
        frame_points.append(points_this_frame)
        frame_totals.append(frame_total)

    selected_frames = select_shape_frames(frames, frame_totals, top_frame_fraction, min_selected_frames)
    points = [p for i, pts in enumerate(frame_points) if i in selected_frames for p in pts]
    sensor_totals = {}
    for p in points:
        sensor_totals[p["sensor"]] = sensor_totals.get(p["sensor"], 0.0) + p["activation"]

    assumed_thumb_points = []
    if assume_thumb_final_contact:
        assumed_activation = max([p["activation"] for p in points] + [120.0]) * 0.45
        assumed_thumb_points = make_assumed_thumb_contacts(frames, joints, child_joint_by_link, assumed_activation)
        points.extend(assumed_thumb_points)
        sensor_totals["thumb_final_pose_contact_assumption"] = sum(p["activation"] for p in assumed_thumb_points)

    joint_values = joint_values_from_positions(max_frame.get("positions", []), joints)
    for name, link in SENSOR_TO_LINK.items():
        tf = link_transform(link, child_joint_by_link, joint_values)
        layout_positions[name] = apply(tf, (0.0, 0.0, 0.0))

    return metadata, frames, points, frame_totals, sensor_totals, layout_positions, selected_frames, assumed_thumb_points


def sampled_frame_indices(frames, max_frames):
    if len(frames) <= max_frames:
        return list(range(len(frames)))
    keep = {0, len(frames) - 1}
    hold = [i for i, rec in enumerate(frames) if rec.get("phase") == "hold"]
    if hold:
        keep.add(hold[0])
    for k in range(max_frames):
        keep.add(round(k * (len(frames) - 1) / max(1, max_frames - 1)))
    return sorted(i for i in keep if 0 <= i < len(frames))


def hand_skeleton_frame(urdf, rec):
    joints, child_joint_by_link = parse_urdf(urdf)
    joint_values = joint_values_from_positions(rec.get("positions", []), joints)
    links = {WRIST_LINK}
    for j in joints.values():
        links.add(j["parent"])
        links.add(j["child"])
    nodes = {link: apply(link_transform(link, child_joint_by_link, joint_values), (0.0, 0.0, 0.0)) for link in links}
    hx, hy, hz = [], [], []
    for j in joints.values():
        if j["parent"] not in nodes or j["child"] not in nodes:
            continue
        a, b = nodes[j["parent"]], nodes[j["child"]]
        hx += [a[0] * 1000.0, b[0] * 1000.0, None]
        hy += [a[1] * 1000.0, b[1] * 1000.0, None]
        hz += [a[2] * 1000.0, b[2] * 1000.0, None]
    return hx, hy, hz


def compute_hand_model(urdf, frames):
    joints, child_joint_by_link = parse_urdf(urdf)
    max_frame = max(frames, key=lambda rec: sum(abs(x) for x in (rec.get("force_delta") or [])))
    joint_values = joint_values_from_positions(max_frame.get("positions", []), joints)
    links = {WRIST_LINK}
    for j in joints.values():
        links.add(j["parent"])
        links.add(j["child"])
    nodes = {link: apply(link_transform(link, child_joint_by_link, joint_values), (0.0, 0.0, 0.0)) for link in links}
    edges = [(j["parent"], j["child"]) for j in joints.values() if j["parent"] in nodes and j["child"] in nodes]
    return nodes, edges


def bounds(points):
    if not points:
        return ((0, 0), (1, 1), (0, 1))
    xs = [p["p"][0] for p in points]
    ys = [p["p"][1] for p in points]
    zs = [p["p"][2] for p in points]
    return (min(xs), max(xs)), (min(ys), max(ys)), (min(zs), max(zs))


def robust_extent(vals):
    if not vals:
        return 0.0
    vals = sorted(vals)
    lo = vals[int(0.02 * (len(vals) - 1))]
    hi = vals[int(0.98 * (len(vals) - 1))]
    return max(0.0, hi - lo)


def font(size=18):
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size)
    except Exception:
        return ImageFont.load_default()


def draw_title(draw, text):
    draw.text((28, 22), text, fill=(25, 32, 42), font=font(25))


def project_points(points, axes, width, height, margin=70):
    a, b = axes
    vals_a = [p["p"][a] for p in points] or [0.0, 1.0]
    vals_b = [p["p"][b] for p in points] or [0.0, 1.0]
    min_a, max_a = min(vals_a), max(vals_a)
    min_b, max_b = min(vals_b), max(vals_b)
    if abs(max_a - min_a) < 1e-9:
        max_a += 0.001
    if abs(max_b - min_b) < 1e-9:
        max_b += 0.001
    scale = min((width - 2 * margin) / (max_a - min_a), (height - 2 * margin) / (max_b - min_b))
    off_x = margin + ((width - 2 * margin) - (max_a - min_a) * scale) / 2
    off_y = margin + ((height - 2 * margin) - (max_b - min_b) * scale) / 2
    def f(p):
        return (off_x + (p[a] - min_a) * scale, height - off_y - (p[b] - min_b) * scale)
    return f, scale, (min_a, max_a, min_b, max_b)


def point_color(group, activation, max_activation):
    base = COLORS.get(group, (60, 60, 60))
    k = min(1.0, activation / max(1.0, max_activation))
    return tuple(int(245 * (1 - k) + base[i] * k) for i in range(3))


def save_projection(points, path, axes=(0, 2), title="Contact projection"):
    w, h = 1400, 950
    img = Image.new("RGB", (w, h), (248, 249, 251))
    d = ImageDraw.Draw(img)
    draw_title(d, title)
    f, scale, bb = project_points(points, axes, w, h)
    max_act = max([p["activation"] for p in points] + [1.0])
    for p in sorted(points, key=lambda x: x["activation"]):
        x, y = f(p["p"])
        r = 2 + int(7 * min(1.0, p["activation"] / max_act))
        d.ellipse((x - r, y - r, x + r, y + r), fill=point_color(p["group"], p["activation"], max_act))
    labels = ["X", "Y", "Z"]
    d.text((70, h - 48), f"{labels[axes[0]]}, mm", fill=(85, 92, 100), font=font(18))
    d.text((20, 80), f"{labels[axes[1]]}, mm", fill=(85, 92, 100), font=font(18))
    d.rectangle((60, 60, w - 60, h - 60), outline=(215, 221, 229), width=2)
    img.save(path)


def save_pseudo3d(points, path):
    w, h = 1400, 950
    img = Image.new("RGB", (w, h), (248, 249, 251))
    d = ImageDraw.Draw(img)
    draw_title(d, "3D контактное облако по левой руке (URDF FK)")
    if not points:
        img.save(path)
        return
    xs = [p["p"][0] for p in points]
    ys = [p["p"][1] for p in points]
    zs = [p["p"][2] for p in points]
    cx, cy, cz = statistics.mean(xs), statistics.mean(ys), statistics.mean(zs)
    max_span = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs), 0.001)
    scale = 620 / max_span
    max_act = max([p["activation"] for p in points] + [1.0])
    for p in sorted(points, key=lambda q: q["p"][1]):
        x, y, z = p["p"]
        u = w / 2 + ((x - cx) * 0.95 + (y - cy) * 0.45) * scale
        v = h / 2 - ((z - cz) * 0.95 - (y - cy) * 0.25) * scale
        r = 2 + int(8 * min(1.0, p["activation"] / max_act))
        d.ellipse((u - r, v - r, u + r, v + r), fill=point_color(p["group"], p["activation"], max_act))
    d.text((70, h - 55), "Цвет: палец/ладонь; размер/насыщенность: активация taxel относительно baseline", fill=(70, 78, 88), font=font(18))
    img.save(path)


def decimate_points(points, max_points=4500):
    if len(points) <= max_points:
        return points
    ranked = sorted(points, key=lambda p: p["activation"], reverse=True)
    keep_strong = ranked[: max_points // 2]
    rest = ranked[max_points // 2 :]
    step = max(1, len(rest) // max(1, max_points - len(keep_strong)))
    return keep_strong + rest[::step][: max_points - len(keep_strong)]


def save_interactive_3d(points, summary, path):
    try:
        import plotly.graph_objects as go
    except Exception as exc:
        path.with_suffix(".txt").write_text(f"Plotly import failed: {exc}\n", encoding="utf-8")
        return

    pts = decimate_points(points)
    xs = [p["p"][0] * 1000.0 for p in pts]
    ys = [p["p"][1] * 1000.0 for p in pts]
    zs = [p["p"][2] * 1000.0 for p in pts]
    acts = [p["activation"] for p in pts]
    labels = [p["sensor"] for p in pts]
    groups = [p["group"] for p in pts]
    max_act = max(acts + [1.0])
    colors = [
        "rgb(%d,%d,%d)" % COLORS.get(g, (70, 70, 70))
        for g in groups
    ]
    sizes = [3.0 + 7.0 * min(1.0, a / max_act) for a in acts]

    traces = [
        go.Scatter3d(
            x=xs,
            y=ys,
            z=zs,
            mode="markers",
            marker=dict(size=sizes, color=colors, opacity=0.78, line=dict(width=0)),
            text=labels,
            hovertemplate="sensor=%{text}<br>x=%{x:.1f} mm<br>y=%{y:.1f} mm<br>z=%{z:.1f} mm<extra></extra>",
            name="active taxels",
        )
    ]

    if xs:
        minx, maxx = min(xs), max(xs)
        miny, maxy = min(ys), max(ys)
        minz, maxz = min(zs), max(zs)
        corners = [
            (minx, miny, minz), (maxx, miny, minz), (maxx, maxy, minz), (minx, maxy, minz),
            (minx, miny, maxz), (maxx, miny, maxz), (maxx, maxy, maxz), (minx, maxy, maxz),
        ]
        edges = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]
        bx, by, bz = [], [], []
        for a, b in edges:
            bx += [corners[a][0], corners[b][0], None]
            by += [corners[a][1], corners[b][1], None]
            bz += [corners[a][2], corners[b][2], None]
        traces.append(
            go.Scatter3d(
                x=bx,
                y=by,
                z=bz,
                mode="lines",
                line=dict(color="rgba(20,20,20,0.55)", width=4),
                hoverinfo="skip",
                name="contact envelope bbox",
            )
        )

    title = (
        f"{summary.get('session')} tactile contact envelope, {HAND_SIDE} hand URDF"
        f"<br><sup>L={summary['sorted_dimensions_mm']['L']:.1f} mm, "
        f"W={summary['sorted_dimensions_mm']['W']:.1f} mm, "
        f"H={summary['sorted_dimensions_mm']['H']:.1f} mm</sup>"
    )
    fig = go.Figure(data=traces)
    fig.update_layout(
        title=title,
        width=1280,
        height=880,
        paper_bgcolor="white",
        plot_bgcolor="white",
        scene=dict(
            xaxis_title="X, mm",
            yaxis_title="Y, mm",
            zaxis_title="Z, mm",
            aspectmode="data",
            bgcolor="rgb(247,248,250)",
            xaxis=dict(showbackground=True, backgroundcolor="rgb(240,243,247)", gridcolor="rgb(210,216,224)"),
            yaxis=dict(showbackground=True, backgroundcolor="rgb(240,243,247)", gridcolor="rgb(210,216,224)"),
            zaxis=dict(showbackground=True, backgroundcolor="rgb(240,243,247)", gridcolor="rgb(210,216,224)"),
            camera=dict(eye=dict(x=1.45, y=-1.75, z=1.15)),
        ),
        legend=dict(orientation="h", y=0.02),
        margin=dict(l=0, r=0, t=80, b=0),
    )
    fig.write_html(str(path), include_plotlyjs=True, full_html=True)


def save_rotation_gif(points, summary, path, frames=72):
    pts = decimate_points(points, 3500)
    if not pts:
        return
    w, h = 1100, 850
    coords = [(p["p"][0] * 1000.0, p["p"][1] * 1000.0, p["p"][2] * 1000.0) for p in pts]
    acts = [p["activation"] for p in pts]
    groups = [p["group"] for p in pts]
    cx = statistics.mean([p[0] for p in coords])
    cy = statistics.mean([p[1] for p in coords])
    cz = statistics.mean([p[2] for p in coords])
    span = max(
        max(p[0] for p in coords) - min(p[0] for p in coords),
        max(p[1] for p in coords) - min(p[1] for p in coords),
        max(p[2] for p in coords) - min(p[2] for p in coords),
        1.0,
    )
    scale = 560.0 / span
    max_act = max(acts + [1.0])
    gif_frames = []
    title_font = font(28)
    small_font = font(18)
    for k in range(frames):
        angle = 2.0 * math.pi * k / frames
        ca, sa = math.cos(angle), math.sin(angle)
        rendered = []
        for (x, y, z), act, group in zip(coords, acts, groups):
            x -= cx
            y -= cy
            z -= cz
            xr = x * ca - y * sa
            yr = x * sa + y * ca
            zr = z
            screen_x = w / 2 + (xr * 0.95 + yr * 0.16) * scale
            screen_y = h / 2 - (zr * 0.92 - yr * 0.12) * scale
            rendered.append((yr, screen_x, screen_y, act, group))

        img = Image.new("RGB", (w, h), (248, 249, 251))
        d = ImageDraw.Draw(img)
        d.text((34, 24), "360 degree tactile contact cloud", fill=(20, 28, 38), font=title_font)
        d.text(
            (34, 62),
            f"{summary.get('session')} | {HAND_SIDE} hand URDF | L/W/H={summary['sorted_dimensions_mm']['L']:.1f}/"
            f"{summary['sorted_dimensions_mm']['W']:.1f}/{summary['sorted_dimensions_mm']['H']:.1f} mm",
            fill=(75, 84, 96),
            font=small_font,
        )
        d.rectangle((45, 105, w - 45, h - 55), outline=(216, 222, 230), width=2)
        d.line((w / 2, h - 95, w / 2 + 100, h - 95), fill=(80, 90, 104), width=3)
        d.line((w / 2, h - 95, w / 2, h - 195), fill=(80, 90, 104), width=3)
        d.text((w / 2 + 106, h - 106), "X/Y", fill=(80, 90, 104), font=small_font)
        d.text((w / 2 + 8, h - 205), "Z", fill=(80, 90, 104), font=small_font)
        for _, sx, sy, act, group in sorted(rendered, key=lambda q: q[0]):
            base = COLORS.get(group, (65, 65, 65))
            kk = min(1.0, act / max_act)
            c = tuple(int(235 * (1 - kk) + base[i] * kk) for i in range(3))
            r = 2 + int(7 * kk)
            d.ellipse((sx - r, sy - r, sx + r, sy + r), fill=c)
        d.text((34, h - 42), "Color: finger/palm. Dot size: taxel activation over baseline.", fill=(75, 84, 96), font=small_font)
        gif_frames.append(img)
    gif_frames[0].save(path, save_all=True, append_images=gif_frames[1:], duration=70, loop=0, optimize=True)


def primitive_params(points, summary):
    if not points:
        return (0.0, 0.0, 0.0), (10.0, 10.0, 10.0), "ellipsoid"
    xs = sorted(p["p"][0] * 1000.0 for p in points)
    ys = sorted(p["p"][1] * 1000.0 for p in points)
    zs = sorted(p["p"][2] * 1000.0 for p in points)
    def pct(vals, q):
        return vals[int(q * (len(vals) - 1))]
    mins = [pct(xs, 0.02), pct(ys, 0.02), pct(zs, 0.02)]
    maxs = [pct(xs, 0.98), pct(ys, 0.98), pct(zs, 0.98)]
    center = tuple((a + b) / 2.0 for a, b in zip(mins, maxs))
    radii = tuple(max(8.0, (b - a) / 2.0) for a, b in zip(mins, maxs))
    shape = "box" if "flat" in summary["shape_class_from_tactile_envelope"] else "ellipsoid"
    return center, radii, shape


def ellipsoid_mesh(center, radii, n_u=28, n_v=14):
    cx, cy, cz = center
    rx, ry, rz = radii
    x, y, z = [], [], []
    for iv in range(n_v + 1):
        v = -math.pi / 2.0 + math.pi * iv / n_v
        for iu in range(n_u):
            u = 2.0 * math.pi * iu / n_u
            x.append(cx + rx * math.cos(v) * math.cos(u))
            y.append(cy + ry * math.cos(v) * math.sin(u))
            z.append(cz + rz * math.sin(v))
    i, j, k = [], [], []
    for iv in range(n_v):
        for iu in range(n_u):
            a = iv * n_u + iu
            b = iv * n_u + (iu + 1) % n_u
            c = (iv + 1) * n_u + iu
            d = (iv + 1) * n_u + (iu + 1) % n_u
            i += [a, b]
            j += [c, d]
            k += [b, c]
    return x, y, z, i, j, k


def box_edges(center, radii):
    cx, cy, cz = center
    rx, ry, rz = radii
    corners = [
        (cx - rx, cy - ry, cz - rz), (cx + rx, cy - ry, cz - rz),
        (cx + rx, cy + ry, cz - rz), (cx - rx, cy + ry, cz - rz),
        (cx - rx, cy - ry, cz + rz), (cx + rx, cy - ry, cz + rz),
        (cx + rx, cy + ry, cz + rz), (cx - rx, cy + ry, cz + rz),
    ]
    edges = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]
    xs, ys, zs = [], [], []
    for a, b in edges:
        xs += [corners[a][0], corners[b][0], None]
        ys += [corners[a][1], corners[b][1], None]
        zs += [corners[a][2], corners[b][2], None]
    return xs, ys, zs


def hand_traces(hand_nodes, hand_edges):
    try:
        import plotly.graph_objects as go
    except Exception:
        return []
    hx, hy, hz = [], [], []
    for parent, child in hand_edges:
        if parent not in hand_nodes or child not in hand_nodes:
            continue
        a, b = hand_nodes[parent], hand_nodes[child]
        hx += [a[0] * 1000.0, b[0] * 1000.0, None]
        hy += [a[1] * 1000.0, b[1] * 1000.0, None]
        hz += [a[2] * 1000.0, b[2] * 1000.0, None]
    nx = [p[0] * 1000.0 for p in hand_nodes.values()]
    ny = [p[1] * 1000.0 for p in hand_nodes.values()]
    nz = [p[2] * 1000.0 for p in hand_nodes.values()]
    return [
        go.Scatter3d(
            x=hx, y=hy, z=hz, mode="lines",
            line=dict(color="rgba(45,54,67,0.78)", width=7),
            hoverinfo="skip", name=f"{HAND_SIDE} hand URDF skeleton",
        ),
        go.Scatter3d(
            x=nx, y=ny, z=nz, mode="markers",
            marker=dict(size=4, color="rgba(45,54,67,0.65)"),
            hoverinfo="skip", name="URDF joints/links",
        ),
    ]


def mesh_traces(hand_meshes):
    try:
        import plotly.graph_objects as go
    except Exception:
        return []
    traces = []
    for mesh in hand_meshes:
        if mesh["is_sensor"]:
            color = "rgb(42,48,58)"
            opacity = 0.92
        else:
            color = "rgb(184,193,208)"
            opacity = 0.78
        traces.append(
            go.Mesh3d(
                x=mesh["x"], y=mesh["y"], z=mesh["z"],
                i=mesh["i"], j=mesh["j"], k=mesh["k"],
                color=color,
                opacity=opacity,
                flatshading=False,
                lighting=dict(ambient=0.68, diffuse=0.86, specular=0.08, roughness=0.72),
                hoverinfo="skip",
                name=mesh["link"],
                showscale=False,
            )
        )
    return traces


def save_interactive_hand_object(points, summary, hand_meshes, path):
    try:
        import plotly.graph_objects as go
    except Exception as exc:
        path.with_suffix(".txt").write_text(f"Plotly import failed: {exc}\n", encoding="utf-8")
        return
    pts = decimate_points(points, 3000)
    max_act = max([p["activation"] for p in pts] + [1.0])
    traces = mesh_traces(hand_meshes)
    real_pts = [p for p in pts if not p.get("assumed")]
    assumed_pts = [p for p in pts if p.get("assumed")]
    if real_pts:
        traces.append(
            go.Scatter3d(
                x=[p["p"][0] * 1000.0 for p in real_pts],
                y=[p["p"][1] * 1000.0 for p in real_pts],
                z=[p["p"][2] * 1000.0 for p in real_pts],
                mode="markers",
                marker=dict(
                    size=[3.0 + 6.0 * min(1.0, p["activation"] / max_act) for p in real_pts],
                    color=["rgb(%d,%d,%d)" % COLORS.get(p["group"], (70, 70, 70)) for p in real_pts],
                    opacity=0.78,
                ),
                text=[p["sensor"] for p in real_pts],
                hovertemplate="real contact %{text}<br>x=%{x:.1f} y=%{y:.1f} z=%{z:.1f} mm<extra></extra>",
                name="real tactile taxels",
            )
        )
    if assumed_pts:
        traces.append(
            go.Scatter3d(
                x=[p["p"][0] * 1000.0 for p in assumed_pts],
                y=[p["p"][1] * 1000.0 for p in assumed_pts],
                z=[p["p"][2] * 1000.0 for p in assumed_pts],
                mode="markers",
                marker=dict(
                    size=7,
                    color="rgb(207,62,54)",
                    opacity=0.96,
                    symbol="diamond",
                    line=dict(width=1, color="rgb(90,20,18)"),
                ),
                text=[p["sensor"] for p in assumed_pts],
                hovertemplate="assumed thumb support %{text}<br>x=%{x:.1f} y=%{y:.1f} z=%{z:.1f} mm<extra></extra>",
                name="assumed final thumb contact",
            )
        )
    if pts:
        traces.append(
            go.Scatter3d(
                x=[p["p"][0] * 1000.0 for p in pts],
                y=[p["p"][1] * 1000.0 for p in pts],
                z=[p["p"][2] * 1000.0 for p in pts],
                mode="markers",
                marker=dict(size=1.5, color="rgba(10,80,95,0.22)"),
                hoverinfo="skip",
                name="shape support samples",
                showlegend=False,
            )
        )
    # Let Plotly build a local alpha-shape surface directly from contact taxels.
    # This is intentionally a contact envelope, not a made-up full ellipsoid.
    if len(pts) >= 8:
        traces.append(
            go.Mesh3d(
                x=[p["p"][0] * 1000.0 for p in pts],
                y=[p["p"][1] * 1000.0 for p in pts],
                z=[p["p"][2] * 1000.0 for p in pts],
                alphahull=14,
                color="rgb(28,132,162)",
                opacity=0.42,
                flatshading=False,
                lighting=dict(ambient=0.72, diffuse=0.7, specular=0.12, roughness=0.82),
                name="contact-area alpha hull",
                hoverinfo="skip",
            )
        )
    fig = go.Figure(data=traces)
    fig.update_layout(
        title=(
            f"{HAND_SIDE.title()} hand STL grasp + tactile contact hull: {summary.get('object')}"
            f"<br><sup>L={summary['sorted_dimensions_mm']['L']:.1f} mm, "
            f"W={summary['sorted_dimensions_mm']['W']:.1f} mm, H={summary['sorted_dimensions_mm']['H']:.1f} mm, "
            f"class={summary['shape_class_from_tactile_envelope']}</sup>"
        ),
        width=1400,
        height=920,
        paper_bgcolor="white",
        annotations=[
            dict(
                x=0.02, y=0.98, xref="paper", yref="paper", showarrow=False,
                align="left",
                bgcolor="rgba(255,255,255,0.86)",
                bordercolor="rgba(60,70,80,0.25)",
                borderwidth=1,
                text=(
                    f"<b>Tactile envelope</b><br>"
                    f"L = {summary['sorted_dimensions_mm']['L']:.1f} mm<br>"
                    f"W = {summary['sorted_dimensions_mm']['W']:.1f} mm<br>"
                    f"H = {summary['sorted_dimensions_mm']['H']:.1f} mm<br>"
                    f"{summary['shape_class_from_tactile_envelope']}<br>"
                    f"points = {summary.get('contact_points', 0)}"
                ),
            )
        ],
        scene=dict(
            xaxis_title="X, mm", yaxis_title="Y, mm", zaxis_title="Z, mm",
            aspectmode="data",
            bgcolor="rgb(247,248,250)",
            camera=dict(eye=dict(x=1.55, y=-1.65, z=1.15)),
        ),
        margin=dict(l=0, r=0, t=88, b=0),
    )
    fig.write_html(str(path), include_plotlyjs=True, full_html=True)


def save_interactive_frame_animation(session, urdf, frames, summary, activation_threshold, assume_thumb_final_contact, path, max_frames=120):
    try:
        import plotly.graph_objects as go
    except Exception as exc:
        path.with_suffix(".txt").write_text(f"Plotly import failed: {exc}\n", encoding="utf-8")
        return

    metadata = json.loads((session / "metadata.json").read_text(encoding="utf-8"))
    baseline = baseline_means(load_jsonl(session / "baseline_raw.jsonl"))
    joints, child_joint_by_link = parse_urdf(urdf)
    indices = sampled_frame_indices(frames, max_frames)
    frame_points = [
        decimate_points(frame_contact_points(frames[i], i, metadata, baseline, joints, child_joint_by_link, activation_threshold), 900)
        for i in indices
    ]
    all_points = [p for pts in frame_points for p in pts]
    max_act = max([p["activation"] for p in all_points] + [1.0])
    assumed_thumb = []
    if assume_thumb_final_contact:
        assumed_thumb = make_assumed_thumb_contacts(frames, joints, child_joint_by_link, max_act * 0.45)

    def contact_trace(pts):
        return go.Scatter3d(
            x=[p["p"][0] * 1000.0 for p in pts],
            y=[p["p"][1] * 1000.0 for p in pts],
            z=[p["p"][2] * 1000.0 for p in pts],
            mode="markers",
            marker=dict(
                size=[3.0 + 6.0 * min(1.0, p["activation"] / max_act) for p in pts],
                color=["rgb(%d,%d,%d)" % COLORS.get(p["group"], (70, 70, 70)) for p in pts],
                opacity=0.82,
            ),
            text=[p["sensor"] for p in pts],
            hovertemplate="frame contact %{text}<br>x=%{x:.1f} y=%{y:.1f} z=%{z:.1f} mm<extra></extra>",
            name="real tactile taxels in current frame",
        )

    def assumed_trace():
        return go.Scatter3d(
            x=[p["p"][0] * 1000.0 for p in assumed_thumb],
            y=[p["p"][1] * 1000.0 for p in assumed_thumb],
            z=[p["p"][2] * 1000.0 for p in assumed_thumb],
            mode="markers",
            marker=dict(size=7, color="rgb(207,62,54)", symbol="diamond", opacity=0.96, line=dict(width=1, color="rgb(90,20,18)")),
            text=[p["sensor"] for p in assumed_thumb],
            hovertemplate="assumed final thumb support<br>x=%{x:.1f} y=%{y:.1f} z=%{z:.1f} mm<extra></extra>",
            name="assumed final thumb contact",
        )

    def hull_trace(pts):
        if len(pts) < 8:
            return go.Mesh3d(x=[], y=[], z=[], i=[], j=[], k=[], name="current-frame contact hull", hoverinfo="skip")
        return go.Mesh3d(
            x=[p["p"][0] * 1000.0 for p in pts],
            y=[p["p"][1] * 1000.0 for p in pts],
            z=[p["p"][2] * 1000.0 for p in pts],
            alphahull=14,
            color="rgb(28,132,162)",
            opacity=0.34,
            flatshading=False,
            hoverinfo="skip",
            name="current-frame contact hull",
        )

    first_idx = indices[0] if indices else 0
    first_points = frame_points[0] if frame_points else []
    hx, hy, hz = hand_skeleton_frame(urdf, frames[first_idx])
    traces = [
        go.Scatter3d(
            x=hx,
            y=hy,
            z=hz,
            mode="lines",
            line=dict(color="rgba(15,22,32,0.94)", width=8),
            hoverinfo="skip",
            name="moving URDF skeleton",
        ),
        contact_trace(first_points),
        assumed_trace(),
        hull_trace(first_points + assumed_thumb),
    ]

    plotly_frames = []
    slider_steps = []
    for visual_i, frame_idx in enumerate(indices):
        pts = frame_points[visual_i]
        hx, hy, hz = hand_skeleton_frame(urdf, frames[frame_idx])
        phase = frames[frame_idx].get("phase", "")
        frame_name = str(frame_idx)
        plotly_frames.append(
            go.Frame(
                name=frame_name,
                data=[
                    go.Scatter3d(x=hx, y=hy, z=hz),
                    contact_trace(pts),
                    assumed_trace(),
                    hull_trace(pts + assumed_thumb),
                ],
                traces=[0, 1, 2, 3],
                layout=go.Layout(
                    title=(
                        f"Frame-by-frame {HAND_SIDE} hand grasp: {summary.get('object')}"
                        f"<br><sup>frame={frame_idx}, phase={phase}, sampled {len(indices)}/{len(frames)} frames</sup>"
                    )
                ),
            )
        )
        slider_steps.append(
            dict(
                label=str(frame_idx),
                method="animate",
                args=[[frame_name], dict(mode="immediate", frame=dict(duration=0, redraw=True), transition=dict(duration=0))],
            )
        )

    fig = go.Figure(data=traces, frames=plotly_frames)
    fig.update_layout(
        title=(
            f"Frame-by-frame {HAND_SIDE} hand grasp: {summary.get('object')}"
            f"<br><sup>moving URDF skeleton + per-frame contact taxels, sampled {len(indices)}/{len(frames)} frames</sup>"
        ),
        width=1400,
        height=920,
        paper_bgcolor="white",
        scene=dict(
            xaxis_title="X, mm",
            yaxis_title="Y, mm",
            zaxis_title="Z, mm",
            aspectmode="data",
            bgcolor="rgb(247,248,250)",
            camera=dict(eye=dict(x=1.55, y=-1.65, z=1.15)),
        ),
        updatemenus=[
            dict(
                type="buttons",
                showactive=False,
                x=0.02,
                y=0.04,
                xanchor="left",
                yanchor="bottom",
                buttons=[
                    dict(label="Play", method="animate", args=[None, dict(frame=dict(duration=90, redraw=True), transition=dict(duration=0), fromcurrent=True)]),
                    dict(label="Pause", method="animate", args=[[None], dict(mode="immediate", frame=dict(duration=0, redraw=False), transition=dict(duration=0))]),
                ],
            )
        ],
        sliders=[
            dict(
                active=0,
                currentvalue=dict(prefix="frame: "),
                pad=dict(t=30),
                steps=slider_steps,
            )
        ],
        annotations=[
            dict(
                x=0.02,
                y=0.98,
                xref="paper",
                yref="paper",
                showarrow=False,
                align="left",
                bgcolor="rgba(255,255,255,0.86)",
                bordercolor="rgba(60,70,80,0.25)",
                borderwidth=1,
                text=(
                    "Black: moving URDF skeleton<br>"
                    "Colored dots: real taxels per frame<br>"
                    "Blue surface: current contact area"
                ),
            )
        ],
        margin=dict(l=0, r=0, t=88, b=0),
    )
    fig.write_html(str(path), include_plotlyjs=True, full_html=True)


def project_scene_point(p, center, scale, angle, w, h):
    x, y, z = p
    x -= center[0]
    y -= center[1]
    z -= center[2]
    ca, sa = math.cos(angle), math.sin(angle)
    xr = x * ca - y * sa
    yr = x * sa + y * ca
    return (
        w / 2 + (xr * 0.95 + yr * 0.16) * scale,
        h / 2 - (z * 0.92 - yr * 0.12) * scale,
        yr,
    )


def convex_hull_2d(points):
    pts = sorted(set((round(x, 3), round(y, 3)) for x, y in points))
    if len(pts) <= 2:
        return pts
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def save_hand_object_gif(points, summary, hand_meshes, path, frames=72, mesh_edge_target=1200):
    pts = decimate_points(points, 2200)
    mesh_points = []
    mesh_edges = []
    for mesh in hand_meshes:
        tri_count = len(mesh["i"])
        step = max(1, math.ceil(tri_count / mesh_edge_target))
        for idx in range(0, tri_count, step):
            tri = [mesh["i"][idx], mesh["j"][idx], mesh["k"][idx]]
            coords = [(mesh["x"][a], mesh["y"][a], mesh["z"][a]) for a in tri]
            mesh_points.extend(coords)
            mesh_edges.append((coords[0], coords[1], mesh["is_sensor"]))
            mesh_edges.append((coords[1], coords[2], mesh["is_sensor"]))
            mesh_edges.append((coords[2], coords[0], mesh["is_sensor"]))
    scene_points = [tuple(v * 1000.0 for v in p["p"]) for p in pts] + mesh_points
    if not scene_points:
        return
    cx = statistics.mean(p[0] for p in scene_points)
    cy = statistics.mean(p[1] for p in scene_points)
    cz = statistics.mean(p[2] for p in scene_points)
    span = max(
        max(p[0] for p in scene_points) - min(p[0] for p in scene_points),
        max(p[1] for p in scene_points) - min(p[1] for p in scene_points),
        max(p[2] for p in scene_points) - min(p[2] for p in scene_points),
        1.0,
    )
    w, h = 1200, 900
    scale = 650.0 / span
    max_act = max([p["activation"] for p in pts] + [1.0])
    gif_frames = []
    for frame_i in range(frames):
        angle = 2.0 * math.pi * frame_i / frames
        img = Image.new("RGB", (w, h), (248, 249, 251))
        d = ImageDraw.Draw(img)
        d.text((34, 24), f"{HAND_SIDE.title()} hand STL grasp + tactile contact hull", fill=(20, 28, 38), font=font(28))
        d.rounded_rectangle((34, 66, 420, 178), radius=8, fill=(255, 255, 255), outline=(214, 221, 229), width=2)
        d.text((54, 82), f"L {summary['sorted_dimensions_mm']['L']:.1f} mm", fill=(25, 32, 42), font=font(22))
        d.text((54, 112), f"W {summary['sorted_dimensions_mm']['W']:.1f} mm", fill=(25, 32, 42), font=font(22))
        d.text((54, 142), f"H {summary['sorted_dimensions_mm']['H']:.1f} mm", fill=(25, 32, 42), font=font(22))
        d.rectangle((45, 200, w - 45, h - 55), outline=(216, 222, 230), width=2)
        contact_screen = [
            project_scene_point(tuple(v * 1000.0 for v in p["p"]), (cx, cy, cz), scale, angle, w, h)[:2]
            for p in pts
        ]
        hull = convex_hull_2d(contact_screen)
        if len(hull) >= 3:
            overlay = Image.new("RGBA", (w, h), (0, 0, 0, 0))
            od = ImageDraw.Draw(overlay)
            od.polygon(hull, fill=(40, 150, 180, 58), outline=(20, 110, 145, 210))
            img = Image.alpha_composite(img.convert("RGBA"), overlay).convert("RGB")
            d = ImageDraw.Draw(img)
        projected_edges = []
        for a, b, is_sensor in mesh_edges:
            ax, ay, da = project_scene_point(a, (cx, cy, cz), scale, angle, w, h)
            bx, by, db = project_scene_point(b, (cx, cy, cz), scale, angle, w, h)
            projected_edges.append(((da + db) / 2.0, ax, ay, bx, by, is_sensor))
        for _, ax, ay, bx, by, is_sensor in sorted(projected_edges, key=lambda item: item[0]):
            color = (48, 55, 66) if is_sensor else (165, 174, 190)
            width_line = 2 if is_sensor else 1
            d.line((ax, ay, bx, by), fill=color, width=width_line)
        rendered = []
        for p in pts:
            sx, sy, depth = project_scene_point(tuple(v * 1000.0 for v in p["p"]), (cx, cy, cz), scale, angle, w, h)
            rendered.append((depth, sx, sy, p))
        for _, sx, sy, p in sorted(rendered, key=lambda item: item[0]):
            kk = min(1.0, p["activation"] / max_act)
            base = COLORS.get(p["group"], (65, 65, 65))
            c = tuple(int(235 * (1 - kk) + base[i] * kk) for i in range(3))
            r = 2 + int(6 * kk)
            d.ellipse((sx - r, sy - r, sx + r, sy + r), fill=c)
        d.text((34, h - 42), f"Gray wireframe: real {HAND_SIDE}-hand STL meshes. Blue filled contour: contact-point hull. Dots: active taxels.", fill=(75, 84, 96), font=font(18))
        gif_frames.append(img)
    gif_frames[0].save(path, save_all=True, append_images=gif_frames[1:], duration=70, loop=0, optimize=True)


def save_hand_layout(layout_positions, sensor_totals, path):
    points = [{"p": p, "activation": sensor_totals.get(name, 0.0), "group": sensor_group(name), "sensor": name} for name, p in layout_positions.items()]
    w, h = 1700, 1050
    img = Image.new("RGB", (w, h), (248, 249, 251))
    d = ImageDraw.Draw(img)
    draw_title(d, "Левая рука: расположение сенсорных зон из URDF")
    plot_w = 1120
    f, _, _ = project_points(points, (0, 2), plot_w, h, margin=82)
    max_act = max([p["activation"] for p in points] + [1.0])
    ordered = sorted(points, key=lambda p: p["sensor"])
    for idx, p in enumerate(ordered, start=1):
        x, y = f(p["p"])
        r = 9 + int(24 * min(1.0, p["activation"] / max_act))
        c = point_color(p["group"], p["activation"], max_act)
        d.ellipse((x - r, y - r, x + r, y + r), fill=c, outline=(30, 36, 44), width=2)
        d.text((x - 6, y - 8), str(idx), fill=(255, 255, 255), font=font(13))
    d.line((plot_w + 24, 72, plot_w + 24, h - 68), fill=(215, 221, 229), width=2)
    d.text((plot_w + 52, 84), "Легенда сенсорных зон", fill=(25, 32, 42), font=font(22))
    for idx, p in enumerate(ordered, start=1):
        col = 0 if idx <= 9 else 1
        row = (idx - 1) % 9
        x0 = plot_w + 52 + col * 270
        y0 = 132 + row * 82
        c = point_color(p["group"], p["activation"], max_act)
        d.ellipse((x0, y0 + 3, x0 + 22, y0 + 25), fill=c, outline=(30, 36, 44), width=1)
        d.text((x0 + 32, y0), f"{idx}. {p['sensor']}", fill=(35, 42, 52), font=font(15))
        d.text((x0 + 32, y0 + 24), f"sum={p['activation']:.0f}", fill=(95, 104, 116), font=font(13))
    d.text((70, h - 55), "Размер точки показывает суммарную активацию датчика; подписи вынесены в легенду, чтобы не перекрывать руку.", fill=(70, 78, 88), font=font(18))
    img.save(path)


def save_timeline(frame_totals, frames, path):
    w, h = 1400, 620
    img = Image.new("RGB", (w, h), (248, 249, 251))
    d = ImageDraw.Draw(img)
    draw_title(d, "Динамика контакта по времени")
    left, top, right, bottom = 80, 90, w - 60, h - 80
    d.rectangle((left, top, right, bottom), outline=(215, 221, 229), width=2)
    max_v = max(frame_totals + [1.0])
    n = max(1, len(frame_totals) - 1)
    pts = []
    for i, v in enumerate(frame_totals):
        x = left + (right - left) * i / n
        y = bottom - (bottom - top) * v / max_v
        pts.append((x, y))
    if len(pts) > 1:
        d.line(pts, fill=(30, 111, 159), width=4)
    force_max = max([max(abs(x) for x in (f.get("force_delta") or [0])) for f in frames] + [1.0])
    force_pts = []
    for i, f in enumerate(frames):
        v = max(abs(x) for x in (f.get("force_delta") or [0]))
        x = left + (right - left) * i / n
        y = bottom - (bottom - top) * v / force_max
        force_pts.append((x, y))
    if len(force_pts) > 1:
        d.line(force_pts, fill=(207, 62, 54), width=3)
    d.text((left, bottom + 20), "кадр", fill=(70, 78, 88), font=font(18))
    d.text((left, top - 28), "синий: tactile activation, красный: max |force_delta|", fill=(70, 78, 88), font=font(18))
    img.save(path)


def save_shape_summary(points, metadata, path):
    xs = [p["p"][0] for p in points]
    ys = [p["p"][1] for p in points]
    zs = [p["p"][2] for p in points]
    ext = {
        "X_mm": robust_extent(xs) * 1000.0,
        "Y_mm": robust_extent(ys) * 1000.0,
        "Z_mm": robust_extent(zs) * 1000.0,
    }
    dims = sorted(ext.values(), reverse=True)
    ratio = dims[0] / max(1e-6, dims[1]) if len(dims) >= 2 else 0.0
    flat_ratio = dims[2] / max(1e-6, dims[0]) if len(dims) >= 3 else 0.0
    if flat_ratio < 0.25:
        shape = "flat / sheet-like contact envelope"
    elif ratio > 1.8:
        shape = "elongated contact envelope"
    else:
        shape = "compact / rounded contact envelope"
    out = {
        "session": metadata.get("session_name"),
        "object": metadata.get("object_name"),
        "mode": metadata.get("mode"),
        "contact_points": len(points),
        "real_contact_points": sum(1 for p in points if not p.get("assumed")),
        "assumed_contact_points": sum(1 for p in points if p.get("assumed")),
        "hand_side": HAND_SIDE,
        "robust_extent_mm": ext,
        "sorted_dimensions_mm": {"L": dims[0] if dims else 0.0, "W": dims[1] if len(dims) > 1 else 0.0, "H": dims[2] if len(dims) > 2 else 0.0},
        "shape_class_from_tactile_envelope": shape,
        "note": f"Dimensions are a tactile contact envelope in the {HAND_SIDE} hand URDF frame, not a complete object mesh.",
    }
    if out["assumed_contact_points"]:
        out["assumptions"] = [
            "Thumb tactile arrays were zero in the recording; the final thumb pose is used as an explicit object support/contact boundary.",
            f"Assumed thumb samples are marked with assumed=true in {CONTACT_POINTS_FILENAME} and are not raw tactile activations.",
        ]
    path.write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")
    return out


def save_points_csv(points, path):
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["frame", "sensor", "group", "x_m", "y_m", "z_m", "activation", "assumed"])
        for p in points:
            writer.writerow([p["frame"], p["sensor"], p["group"], *p["p"], p["activation"], bool(p.get("assumed"))])


def save_readme(summary, out_dir):
    side = summary.get("hand_side", HAND_SIDE)
    text = f"""# Tactile Shape Report Visualizations

Session: `{summary.get('session')}`
Object: `{summary.get('object')}`
Hand model: {side} Inspire RH56DFTP URDF (`FTP_{side}_hand.urdf`)

Generated figures:

- `{SENSOR_LAYOUT_FILENAME}` - URDF-based {side} hand sensor layout, weighted by total tactile activation.
- `02_contact_cloud_3d.png` - pseudo-3D contact cloud from active taxels.
- `03_contact_projection_xz.png` - side projection of the tactile contact envelope.
- `04_contact_projection_xy.png` - palm-plane projection of the tactile contact envelope.
- `05_contact_projection_yz.png` - cross projection of the tactile contact envelope.
- `06_contact_timeline.png` - tactile activation and force-delta dynamics.
- `07_interactive_contact_cloud.html` - interactive rotatable 3D scene.
- `08_contact_cloud_rotation_360.gif` - 360 degree rotation animation.
- `09_interactive_hand_object_primitive.html` - interactive real STL {side} hand with tactile contact hull and dimensions.
- `10_hand_object_primitive_rotation_360.gif` - 360 degree animation of the STL hand wireframe and tactile dimension box.
- `11_interactive_frame_by_frame_grasp.html` - frame-by-frame 3D animation with moving URDF skeleton lines, per-frame taxel contacts, and current contact-area hull.
- `{CONTACT_POINTS_FILENAME}` - reconstructed contact points for downstream analysis.
- `shape_summary.json` - tactile envelope dimensions and coarse shape class.

Estimated tactile envelope:

- L: {summary['sorted_dimensions_mm']['L']:.1f} mm
- W: {summary['sorted_dimensions_mm']['W']:.1f} mm
- H: {summary['sorted_dimensions_mm']['H']:.1f} mm
- coarse class: `{summary['shape_class_from_tactile_envelope']}`

Important: this is not full 3D reconstruction. It is a report-ready tactile contact envelope computed from raw taxel activation, baseline subtraction, and {side}-hand URDF forward kinematics.
"""
    if summary.get("assumptions"):
        text += "\nAssumptions used in this run:\n\n"
        for item in summary["assumptions"]:
            text += f"- {item}\n"
    (out_dir / "README.md").write_text(text, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", default=str(DEFAULT_SESSION))
    parser.add_argument("--urdf", default=str(DEFAULT_URDF))
    parser.add_argument("--out-root", default=str(DEFAULT_OUT))
    parser.add_argument("--activation-threshold", type=float, default=40.0)
    parser.add_argument("--top-frame-fraction", type=float, default=0.35, help="Fraction of hold frames used for shape reconstruction.")
    parser.add_argument("--min-selected-frames", type=int, default=12, help="Minimum number of high-contact frames used for shape reconstruction.")
    parser.add_argument("--assume-thumb-final-contact", action="store_true", help="Use final thumb pose as an explicit contact/support boundary when thumb tactile data is all zero.")
    parser.add_argument("--max-animation-frames", type=int, default=120, help="Maximum sampled frames in frame-by-frame HTML animation.")
    parser.add_argument("--mesh-quality", choices=("normal", "high", "full"), default="normal", help="STL mesh density for hand rendering.")
    parser.add_argument("--gif-mesh-edge-target", type=int, default=1200, help="Approximate triangles per link used as wireframe edges in the hand GIF.")
    args = parser.parse_args()

    session = Path(args.session).resolve()
    urdf = Path(args.urdf).resolve()
    configure_hand_side("right" if "right" in urdf.name.lower() else "left")
    out_dir = Path(args.out_root).resolve() / session.name
    out_dir.mkdir(parents=True, exist_ok=True)

    metadata, frames, points, frame_totals, sensor_totals, layout_positions, selected_frames, assumed_thumb_points = collect_contact_points(
        session,
        urdf,
        args.activation_threshold,
        top_frame_fraction=args.top_frame_fraction,
        min_selected_frames=args.min_selected_frames,
        assume_thumb_final_contact=args.assume_thumb_final_contact,
    )
    hand_meshes = hand_mesh_data(urdf, frames, quality=args.mesh_quality)
    summary = save_shape_summary(points, metadata, out_dir / "shape_summary.json")
    summary["selected_shape_frames"] = {
        "count": len(selected_frames),
        "first": min(selected_frames) if selected_frames else None,
        "last": max(selected_frames) if selected_frames else None,
        "method": "top tactile-activation hold frames",
    }
    if assumed_thumb_points:
        summary["thumb_final_contact_assumption"] = {
            "enabled": True,
            "points": len(assumed_thumb_points),
            "frame": assumed_thumb_points[0]["frame"],
            "links": [item[0] for item in THUMB_ASSUMED_LINKS],
        }
    (out_dir / "shape_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    save_points_csv(points, out_dir / CONTACT_POINTS_FILENAME)
    save_hand_layout(layout_positions, sensor_totals, out_dir / SENSOR_LAYOUT_FILENAME)
    save_pseudo3d(points, out_dir / "02_contact_cloud_3d.png")
    save_projection(points, out_dir / "03_contact_projection_xz.png", axes=(0, 2), title="Проекция контактов X-Z: высота/длина контактной формы")
    save_projection(points, out_dir / "04_contact_projection_xy.png", axes=(0, 1), title="Проекция контактов X-Y: распределение по ладони/пальцам")
    save_projection(points, out_dir / "05_contact_projection_yz.png", axes=(1, 2), title="Проекция контактов Y-Z: поперечный профиль контактной формы")
    save_timeline(frame_totals, frames, out_dir / "06_contact_timeline.png")
    save_interactive_3d(points, summary, out_dir / "07_interactive_contact_cloud.html")
    save_rotation_gif(points, summary, out_dir / "08_contact_cloud_rotation_360.gif")
    save_interactive_hand_object(points, summary, hand_meshes, out_dir / "09_interactive_hand_object_primitive.html")
    save_hand_object_gif(points, summary, hand_meshes, out_dir / "10_hand_object_primitive_rotation_360.gif", mesh_edge_target=args.gif_mesh_edge_target)
    save_interactive_frame_animation(
        session,
        urdf,
        frames,
        summary,
        args.activation_threshold,
        args.assume_thumb_final_contact,
        out_dir / "11_interactive_frame_by_frame_grasp.html",
        max_frames=args.max_animation_frames,
    )
    save_readme(summary, out_dir)
    print(out_dir)


if __name__ == "__main__":
    main()
