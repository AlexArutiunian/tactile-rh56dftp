#!/usr/bin/env python3
import json
import statistics
from collections import deque
from pathlib import Path


TACTILE_SPECS = {
    "small_tip_3x3": (3, 3),
    "small_tip_12x8": (12, 8),
    "small_pad_10x8": (10, 8),
    "ring_tip_3x3": (3, 3),
    "ring_tip_12x8": (12, 8),
    "ring_pad_10x8": (10, 8),
    "middle_tip_3x3": (3, 3),
    "middle_tip_12x8": (12, 8),
    "middle_pad_10x8": (10, 8),
    "index_tip_3x3": (3, 3),
    "index_tip_12x8": (12, 8),
    "index_pad_10x8": (10, 8),
    "thumb_tip_3x3": (3, 3),
    "thumb_tip_12x8": (12, 8),
    "thumb_middle_3x3": (3, 3),
    "thumb_pad_12x8": (12, 8),
    "palm_8x14": (8, 14),
}


def load_jsonl(path):
    path = Path(path)
    with path.open(encoding="utf-8") as f:
        return [json.loads(line) for line in f if line.strip()]


def save_jsonl(path, rows):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")


def _sensor_values(frame):
    return frame.get("sensors_raw", frame)


def baseline_stats(frames):
    values = {}
    for frame in frames:
        for name, arr in _sensor_values(frame).items():
            values.setdefault(name, [[] for _ in arr])
            for i, raw in enumerate(arr):
                if i < len(values[name]):
                    values[name][i].append(float(raw))

    stats = {}
    for name, per_taxel in values.items():
        means = []
        stds = []
        max_abs = []
        for series in per_taxel:
            if not series:
                means.append(0.0)
                stds.append(0.0)
                max_abs.append(0.0)
                continue
            mean = statistics.mean(series)
            means.append(mean)
            stds.append(statistics.pstdev(series) if len(series) > 1 else 0.0)
            max_abs.append(max(abs(v - mean) for v in series))
        stats[name] = {"mean": means, "std": stds, "max_abs_idle_delta": max_abs}
    return stats


def positive_deltas(sample, stats):
    out = {}
    for name, arr in sample.items():
        base = stats.get(name, {}).get("mean", [])
        out[name] = [
            max(0.0, float(raw) - (base[i] if i < len(base) else 0.0))
            for i, raw in enumerate(arr)
        ]
    return out


def adaptive_thresholds(stats, min_delta=80.0, sigma=6.0, idle_margin=3.0):
    thresholds = {}
    for name, st in stats.items():
        std = st.get("std", [])
        idle = st.get("max_abs_idle_delta", [])
        n = max(len(std), len(idle))
        arr = []
        for i in range(n):
            arr.append(max(
                float(min_delta),
                float(sigma) * (std[i] if i < len(std) else 0.0),
                float(idle_margin) * (idle[i] if i < len(idle) else 0.0),
            ))
        thresholds[name] = arr
    return thresholds


def component_labels(mask, rows, cols, diagonal=True):
    visited = [False] * len(mask)
    comps = []
    if diagonal:
        neigh = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    else:
        neigh = [(-1, 0), (0, -1), (0, 1), (1, 0)]

    for start, active in enumerate(mask):
        if not active or visited[start]:
            continue
        q = deque([start])
        visited[start] = True
        comp = []
        while q:
            idx = q.popleft()
            comp.append(idx)
            r, c = divmod(idx, cols)
            for dr, dc in neigh:
                rr, cc = r + dr, c + dc
                ni = rr * cols + cc
                if 0 <= rr < rows and 0 <= cc < cols and ni < len(mask) and mask[ni] and not visited[ni]:
                    visited[ni] = True
                    q.append(ni)
        comps.append(comp)
    return comps


class TactileNoiseFilter:
    def __init__(
        self,
        baseline_frames,
        min_delta=80.0,
        sigma=6.0,
        idle_margin=3.0,
        min_component_size=2,
        min_component_size_3x3=1,
        temporal_frames=2,
        strong_delta=1500.0,
        diagonal=True,
    ):
        self.stats = baseline_stats(baseline_frames)
        self.thresholds = adaptive_thresholds(self.stats, min_delta, sigma, idle_margin)
        self.min_component_size = int(min_component_size)
        self.min_component_size_3x3 = int(min_component_size_3x3)
        self.temporal_frames = max(1, int(temporal_frames))
        self.strong_delta = float(strong_delta)
        self.diagonal = bool(diagonal)
        self.streaks = {}

    def _min_size(self, name):
        rows, cols = TACTILE_SPECS.get(name, (1, 1))
        return self.min_component_size_3x3 if rows * cols <= 18 else self.min_component_size

    def filter_sample(self, sample):
        deltas = positive_deltas(sample, self.stats)
        filtered_raw = {}
        filtered_delta = {}
        debug = {}

        for name, arr in sample.items():
            rows, cols = TACTILE_SPECS.get(name, (1, max(1, len(arr))))
            n = rows * cols
            base = self.stats.get(name, {}).get("mean", [0.0] * len(arr))
            th = self.thresholds.get(name, [80.0] * len(arr))
            dlt = deltas.get(name, [])
            raw_mask = [
                i < len(dlt) and dlt[i] >= (th[i] if i < len(th) else 80.0)
                for i in range(min(n, len(arr)))
            ]

            kept_component = [False] * len(raw_mask)
            comps = component_labels(raw_mask, rows, cols, self.diagonal)
            min_size = self._min_size(name)
            for comp in comps:
                peak = max(dlt[i] for i in comp)
                if len(comp) >= min_size or peak >= self.strong_delta:
                    for i in comp:
                        kept_component[i] = True

            streak = self.streaks.setdefault(name, [0] * len(raw_mask))
            if len(streak) != len(raw_mask):
                streak = [0] * len(raw_mask)
                self.streaks[name] = streak
            kept_temporal = [False] * len(raw_mask)
            for i, keep in enumerate(kept_component):
                streak[i] = streak[i] + 1 if keep else 0
                kept_temporal[i] = streak[i] >= self.temporal_frames

            out_raw = []
            out_delta = []
            for i, raw in enumerate(arr):
                keep = i < len(kept_temporal) and kept_temporal[i]
                out_raw.append(raw if keep else int(round(base[i] if i < len(base) else 0.0)))
                out_delta.append(dlt[i] if keep and i < len(dlt) else 0.0)

            filtered_raw[name] = out_raw
            filtered_delta[name] = out_delta
            debug[name] = {
                "raw_active": int(sum(raw_mask)),
                "component_active": int(sum(kept_component)),
                "filtered_active": int(sum(kept_temporal)),
                "components": [{"size": len(c), "peak_delta": max(dlt[i] for i in c)} for c in comps],
            }

        return filtered_raw, filtered_delta, debug


def active_counts(deltas, threshold=1.0):
    return {name: sum(1 for v in arr if abs(v) >= threshold) for name, arr in deltas.items()}
