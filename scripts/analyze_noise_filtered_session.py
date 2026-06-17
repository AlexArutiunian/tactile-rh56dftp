#!/usr/bin/env python3
import argparse
import csv
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from tactile_noise_filter import (
    TACTILE_SPECS,
    TactileNoiseFilter,
    active_counts,
    load_jsonl,
    positive_deltas,
    save_jsonl,
)


def font(size):
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size)
    except Exception:
        return ImageFont.load_default()


def draw_heatmap(name, raw_max, filtered_max, path):
    rows, cols = TACTILE_SPECS[name]
    cell = 24 if rows * cols > 18 else 42
    pad = 52
    gap = 40
    w = cols * cell * 2 + pad * 2 + gap
    h = rows * cell + pad * 2 + 38
    img = Image.new("RGB", (w, h), "white")
    d = ImageDraw.Draw(img)
    d.text((18, 12), f"{name}: raw max vs filtered max", fill=(24, 32, 42), font=font(18))
    maxv = max(raw_max + filtered_max + [1.0])

    def draw_panel(xoff, title, values):
        d.text((xoff, 40), title, fill=(70, 78, 88), font=font(13))
        for r in range(rows):
            for c in range(cols):
                i = r * cols + c
                v = values[i] if i < len(values) else 0.0
                k = min(1.0, v / maxv)
                color = (int(248 - 225 * k), int(250 - 125 * k), int(252 - 55 * k))
                x0 = xoff + c * cell
                y0 = pad + r * cell
                d.rectangle((x0, y0, x0 + cell - 2, y0 + cell - 2), fill=color, outline=(218, 224, 232))
                if k > 0.15:
                    d.text((x0 + 3, y0 + 3), str(int(v)), fill=(15, 30, 45), font=font(9))

    left = pad
    right = pad + cols * cell + gap
    draw_panel(left, "raw", raw_max)
    draw_panel(right, "filtered", filtered_max)
    img.save(path)


def max_arrays(sensor_names, frames, key):
    out = {}
    for name in sensor_names:
        rows, cols = TACTILE_SPECS[name]
        out[name] = [0.0] * (rows * cols)
    for frame in frames:
        sensors = frame.get(key, {})
        for name, arr in sensors.items():
            if name not in out:
                continue
            for i, v in enumerate(arr[: len(out[name])]):
                out[name][i] = max(out[name][i], float(v))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True, help="Folder with baseline_raw.jsonl and frames_raw.jsonl")
    ap.add_argument("--out", default=None)
    ap.add_argument("--segment", default=None, help="Optional segment filter, for example bottom")
    ap.add_argument("--min-delta", type=float, default=80.0)
    ap.add_argument("--sigma", type=float, default=6.0)
    ap.add_argument("--idle-margin", type=float, default=3.0)
    ap.add_argument("--min-component-size", type=int, default=2)
    ap.add_argument("--min-component-size-3x3", type=int, default=1)
    ap.add_argument("--temporal-frames", type=int, default=2)
    ap.add_argument("--strong-delta", type=float, default=1500.0)
    args = ap.parse_args()

    session = Path(args.session)
    out = Path(args.out) if args.out else Path("outputs") / f"{session.name}_filtered"
    out.mkdir(parents=True, exist_ok=True)

    baseline = load_jsonl(session / "baseline_raw.jsonl")
    frames = load_jsonl(session / "frames_raw.jsonl")
    if args.segment:
        frames = [
            f for f in frames
            if f.get("segment") == args.segment or f.get("phase") == f"manual_{args.segment}"
        ]

    filt = TactileNoiseFilter(
        baseline,
        min_delta=args.min_delta,
        sigma=args.sigma,
        idle_margin=args.idle_margin,
        min_component_size=args.min_component_size,
        min_component_size_3x3=args.min_component_size_3x3,
        temporal_frames=args.temporal_frames,
        strong_delta=args.strong_delta,
    )

    filtered_rows = []
    summary_rows = []
    raw_delta_frames = []
    filtered_delta_frames = []
    for frame in frames:
        sample = frame.get("sensors_raw", {})
        raw_delta = positive_deltas(sample, filt.stats)
        filtered_raw, filtered_delta, debug = filt.filter_sample(sample)
        rec = dict(frame)
        rec["sensors_filtered_raw"] = filtered_raw
        rec["sensors_filtered_delta"] = filtered_delta
        rec["noise_filter"] = debug
        filtered_rows.append(rec)
        raw_delta_frames.append({"sensors_delta": raw_delta})
        filtered_delta_frames.append({"sensors_delta": filtered_delta})
        raw_counts = active_counts(raw_delta, args.min_delta)
        filtered_counts = active_counts(filtered_delta, 1.0)
        summary_rows.append({
            "frame_id": frame.get("frame_id"),
            "raw_active_total": sum(raw_counts.values()),
            "filtered_active_total": sum(filtered_counts.values()),
            "removed_total": sum(raw_counts.values()) - sum(filtered_counts.values()),
            "raw_active_by_sensor": json.dumps(raw_counts, ensure_ascii=False),
            "filtered_active_by_sensor": json.dumps(filtered_counts, ensure_ascii=False),
        })

    save_jsonl(out / "filtered_frames.jsonl", filtered_rows)
    with (out / "filter_frame_summary.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(summary_rows[0].keys()) if summary_rows else ["frame_id"])
        writer.writeheader()
        writer.writerows(summary_rows)

    sensor_names = list(TACTILE_SPECS)
    raw_max = max_arrays(sensor_names, raw_delta_frames, "sensors_delta")
    filtered_max = max_arrays(sensor_names, filtered_delta_frames, "sensors_delta")
    heatmap_dir = out / "heatmaps"
    heatmap_dir.mkdir(exist_ok=True)
    for name in sensor_names:
        draw_heatmap(name, raw_max[name], filtered_max[name], heatmap_dir / f"{name}_raw_vs_filtered.png")

    totals = {
        "session": str(session),
        "frames": len(frames),
        "filter": {
            "min_delta": args.min_delta,
            "sigma": args.sigma,
            "idle_margin": args.idle_margin,
            "min_component_size": args.min_component_size,
            "min_component_size_3x3": args.min_component_size_3x3,
            "temporal_frames": args.temporal_frames,
            "strong_delta": args.strong_delta,
        },
        "raw_active_total": sum(r["raw_active_total"] for r in summary_rows),
        "filtered_active_total": sum(r["filtered_active_total"] for r in summary_rows),
    }
    totals["removed_total"] = totals["raw_active_total"] - totals["filtered_active_total"]
    (out / "filter_summary.json").write_text(json.dumps(totals, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(totals, indent=2, ensure_ascii=False))
    print(f"Saved: {out}")


if __name__ == "__main__":
    main()
