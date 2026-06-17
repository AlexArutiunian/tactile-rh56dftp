#!/usr/bin/env python3
import json
import math
import struct
import time
import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RIGHT_URDF = ROOT / "urdf" / "inspire_hand" / "FTP_right_hand.urdf"
LEFT_URDF = ROOT / "urdf" / "inspire_hand" / "FTP_left_hand.urdf"

TACTILE_SPECS = {
    "small_tip_3x3": ("TACTILE_SMALL_FINGER_TIP_3x3", 3, 3),
    "small_tip_12x8": ("TACTILE_SMALL_FINGER_TIP_12x8", 12, 8),
    "small_pad_10x8": ("TACTILE_SMALL_FINGER_PALM_10x8", 10, 8),
    "ring_tip_3x3": ("TACTILE_RING_FINGER_TIP_3x3", 3, 3),
    "ring_tip_12x8": ("TACTILE_RING_FINGER_TIP_12x8", 12, 8),
    "ring_pad_10x8": ("TACTILE_RING_FINGER_PALM_10x8", 10, 8),
    "middle_tip_3x3": ("TACTILE_MIDDLE_FINGER_TIP_3x3", 3, 3),
    "middle_tip_12x8": ("TACTILE_MIDDLE_FINGER_TIP_12x8", 12, 8),
    "middle_pad_10x8": ("TACTILE_MIDDLE_FINGER_PALM_10x8", 10, 8),
    "index_tip_3x3": ("TACTILE_INDEX_FINGER_TIP_3x3", 3, 3),
    "index_tip_12x8": ("TACTILE_INDEX_FINGER_TIP_12x8", 12, 8),
    "index_pad_10x8": ("TACTILE_INDEX_FINGER_PALM_10x8", 10, 8),
    "thumb_tip_3x3": ("TACTILE_THUMB_TIP_3x3", 3, 3),
    "thumb_tip_12x8": ("TACTILE_THUMB_TIP_12x8", 12, 8),
    "thumb_middle_3x3": ("TACTILE_THUMB_MIDDLE_3x3", 3, 3),
    "thumb_pad_12x8": ("TACTILE_THUMB_PALM_12x8", 12, 8),
    "palm_8x14": ("TACTILE_PALM_8x14", 8, 14),
}

SENSOR_TO_LINK = {
    "small_tip_3x3": "right_little_force_sensor_3",
    "small_tip_12x8": "right_little_force_sensor_2",
    "small_pad_10x8": "right_little_force_sensor_1",
    "ring_tip_3x3": "right_ring_force_sensor_3",
    "ring_tip_12x8": "right_ring_force_sensor_2",
    "ring_pad_10x8": "right_ring_force_sensor_1",
    "middle_tip_3x3": "right_middle_force_sensor_3",
    "middle_tip_12x8": "right_middle_force_sensor_2",
    "middle_pad_10x8": "right_middle_force_sensor_1",
    "index_tip_3x3": "right_index_force_sensor_3",
    "index_tip_12x8": "right_index_force_sensor_2",
    "index_pad_10x8": "right_index_force_sensor_1",
    "thumb_tip_3x3": "right_thumb_force_sensor_4",
    "thumb_tip_12x8": "right_thumb_force_sensor_3",
    "thumb_middle_3x3": "right_thumb_force_sensor_2",
    "thumb_pad_12x8": "right_thumb_force_sensor_1",
    "palm_8x14": "right_palm_force_sensor",
}

CONTROL_TO_JOINT = {
    0: "right_little_1_joint",
    1: "right_ring_1_joint",
    2: "right_middle_1_joint",
    3: "right_index_1_joint",
    4: "right_thumb_2_joint",
    5: "right_thumb_1_joint",
}


def configure_hand_side(side):
    global SENSOR_TO_LINK, CONTROL_TO_JOINT
    if side not in {"left", "right"}:
        raise ValueError(f"Unsupported hand side: {side}")
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
    CONTROL_TO_JOINT = {
        0: f"{side}_little_1_joint",
        1: f"{side}_ring_1_joint",
        2: f"{side}_middle_1_joint",
        3: f"{side}_index_1_joint",
        4: f"{side}_thumb_2_joint",
        5: f"{side}_thumb_1_joint",
    }

SENSOR_PLANE_SIZE = {
    "3x3": (0.014, 0.014),
    "12x8": (0.018, 0.026),
    "10x8": (0.020, 0.030),
    "8x14": (0.052, 0.040),
}

COLORS = {
    "thumb": (0.85, 0.12, 0.10),
    "index": (0.05, 0.42, 0.78),
    "middle": (0.05, 0.62, 0.30),
    "ring": (0.62, 0.28, 0.82),
    "small": (0.95, 0.58, 0.05),
    "palm": (0.10, 0.10, 0.10),
}


def raw_to_list(raw):
    if raw is None:
        return []
    if isinstance(raw, bytes):
        return list(raw)
    if isinstance(raw, (list, tuple)):
        out = []
        for x in raw:
            try:
                out.append(int(x))
            except Exception:
                pass
        return out
    try:
        return [int(raw)]
    except Exception:
        return []


def decode_taxels(raw, rows, cols):
    return raw_to_list(raw)[: rows * cols]


def read_tactile(client):
    out = {}
    for alias, (register, rows, cols) in TACTILE_SPECS.items():
        try:
            out[alias] = decode_taxels(client.get(register), rows, cols)
        except Exception:
            out[alias] = []
    return out


def read_forces(client):
    out = []
    for i in range(6):
        try:
            out.append(float(client.get(f"FORCE_ACT({i})")))
        except Exception:
            out.append(0.0)
    return out


def read_positions(client, fallback=None):
    vals = []
    for i in range(6):
        try:
            vals.append(float(client.get(f"POS_SET({i})")))
        except Exception:
            vals.append(float((fallback or [0, 0, 0, 0, 0, 0])[i]))
    return vals


def baseline_means(samples):
    sums = {}
    counts = {}
    for sample in samples:
        for name, values in sample.items():
            sums.setdefault(name, [0.0] * len(values))
            counts[name] = counts.get(name, 0) + 1
            for i, value in enumerate(values):
                sums[name][i] += float(value)
    return {name: [v / max(1, counts[name]) for v in values] for name, values in sums.items()}


def parse_vec(text, default):
    if not text:
        return default
    return tuple(float(x) for x in text.split())


def parse_urdf(path=RIGHT_URDF):
    tree = ET.parse(path)
    joints = {}
    child_joint_by_link = {}
    visuals = {}
    for joint in tree.findall("joint"):
        name = joint.attrib["name"]
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        origin = joint.find("origin")
        axis = joint.find("axis")
        limit = joint.find("limit")
        mimic = joint.find("mimic")
        info = {
            "name": name,
            "type": joint.attrib.get("type", "fixed"),
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
        child_joint_by_link[child] = info
    for link in tree.findall("link"):
        visual = link.find("visual")
        if visual is None:
            continue
        geometry = visual.find("geometry")
        mesh = geometry.find("mesh") if geometry is not None else None
        if mesh is not None and "filename" in mesh.attrib:
            visuals[link.attrib["name"]] = (path.parent / mesh.attrib["filename"]).resolve()
    return joints, child_joint_by_link, visuals


def matmul(a, b):
    out = [[0.0] * 4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            out[i][j] = sum(a[i][k] * b[k][j] for k in range(4))
    return out


def eye():
    return [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]


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


def apply(m, p):
    x, y, z = p
    return (
        m[0][0] * x + m[0][1] * y + m[0][2] * z + m[0][3],
        m[1][0] * x + m[1][1] * y + m[1][2] * z + m[1][3],
        m[2][0] * x + m[2][1] * y + m[2][2] * z + m[2][3],
    )


def joint_values_from_positions(positions, joints):
    values = {}
    for idx, joint_name in CONTROL_TO_JOINT.items():
        if idx >= len(positions) or joint_name not in joints:
            continue
        joint = joints[joint_name]
        frac = max(0.0, min(1.0, float(positions[idx]) / 1800.0))
        values[joint_name] = joint["lower"] + frac * (joint["upper"] - joint["lower"])
    changed = True
    while changed:
        changed = False
        for name, joint in joints.items():
            if joint["mimic"] and name not in values:
                parent, mul, off = joint["mimic"]
                if parent in values:
                    values[name] = values[parent] * mul + off
                    changed = True
    return values


def link_transform(link, child_joint_by_link, joint_values):
    chain = []
    cur = link
    while cur in child_joint_by_link:
        joint = child_joint_by_link[cur]
        chain.append(joint)
        cur = joint["parent"]
    m = eye()
    for joint in reversed(chain):
        jm = transform(joint["xyz"], joint["rpy"])
        if joint["type"] == "revolute":
            jm = matmul(jm, rot_axis(joint["axis"], joint_values.get(joint["name"], 0.0)))
        m = matmul(m, jm)
    return m


def quaternion_from_matrix(m):
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        return (0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        return ((m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        return ((m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s)
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
    return ((m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s)


def sensor_group(name):
    return "small" if name.startswith("small") else name.split("_", 1)[0]


def plane_size(name):
    for key, size in SENSOR_PLANE_SIZE.items():
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
    # Taxels live on the visible sensor face. Use actual STL extents instead of
    # a guessed plane; otherwise 12x8 arrays can float outside the finger.
    mx = 0.06 * (maxs[0] - mins[0])
    my = 0.06 * (maxs[1] - mins[1])
    x0, x1 = mins[0] + mx, maxs[0] - mx
    y0, y1 = mins[1] + my, maxs[1] - my
    x = x0 + ((col + 0.5) / max(1, cols)) * (x1 - x0)
    y = y0 + ((row + 0.5) / max(1, rows)) * (y1 - y0)
    z = maxs[2] + 0.00018
    return x, y, z


def tactile_points(sample, baseline, joints, child_joint_by_link, positions, threshold):
    joint_values = joint_values_from_positions(positions, joints)
    points = []
    for name, values in sample.items():
        if name not in SENSOR_TO_LINK:
            continue
        register, rows, cols = TACTILE_SPECS[name]
        base = baseline.get(name, [0.0] * len(values))
        tf = link_transform(SENSOR_TO_LINK[name], child_joint_by_link, joint_values)
        for i, raw in enumerate(values[: rows * cols]):
            delta = max(0.0, float(raw) - (base[i] if i < len(base) else 0.0))
            if delta < threshold:
                continue
            r, c = divmod(i, cols)
            points.append({
                "sensor": name,
                "group": sensor_group(name),
                "delta": delta,
                "p": apply(tf, sensor_taxel_local_point(name, r, c, rows, cols)),
            })
    return points
