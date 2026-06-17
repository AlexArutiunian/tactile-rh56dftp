#!/usr/bin/env python3
import argparse
import csv
import json
import math
import statistics
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


TACTILE_SPECS = {
    "small_tip_3x3": (3, 3, 14.0, 14.0),
    "small_tip_12x8": (12, 8, 18.0, 26.0),
    "small_pad_10x8": (10, 8, 20.0, 30.0),
    "ring_tip_3x3": (3, 3, 14.0, 14.0),
    "ring_tip_12x8": (12, 8, 18.0, 26.0),
    "ring_pad_10x8": (10, 8, 20.0, 30.0),
    "middle_tip_3x3": (3, 3, 14.0, 14.0),
    "middle_tip_12x8": (12, 8, 18.0, 26.0),
    "middle_pad_10x8": (10, 8, 20.0, 30.0),
    "index_tip_3x3": (3, 3, 14.0, 14.0),
    "index_tip_12x8": (12, 8, 18.0, 26.0),
    "index_pad_10x8": (10, 8, 20.0, 30.0),
    "thumb_tip_3x3": (3, 3, 14.0, 14.0),
    "thumb_tip_12x8": (12, 8, 18.0, 26.0),
    "thumb_middle_3x3": (3, 3, 14.0, 14.0),
    "thumb_pad_12x8": (12, 8, 18.0, 26.0),
    "palm_8x14": (8, 14, 52.0, 40.0),
}


def load_jsonl(path):
    with path.open(encoding="utf-8") as f:
        return [json.loads(line) for line in f if line.strip()]


def baseline_stats(frames):
    vals = {}
    for rec in frames:
        for name, arr in rec.get("sensors_raw", {}).items():
            vals.setdefault(name, [[] for _ in arr])
            for i, v in enumerate(arr):
                vals[name][i].append(float(v))
    means = {}
    stds = {}
    for name, series in vals.items():
        means[name] = [statistics.mean(v) if v else 0.0 for v in series]
        stds[name] = [statistics.pstdev(v) if len(v) > 1 else 0.0 for v in series]
    return means, stds


def font(size):
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size)
    except Exception:
        return ImageFont.load_default()


def circle_fit(points):
    # Algebraic least-squares: x^2 + y^2 + A*x + B*y + C = 0
    if len(points) < 3:
        return None
    sx = sy = sxx = syy = sxy = sz = sxz = syz = 0.0
    n = len(points)
    for x, y, _w in points:
        z = -(x * x + y * y)
        sx += x
        sy += y
        sxx += x * x
        syy += y * y
        sxy += x * y
        sz += z
        sxz += x * z
        syz += y * z
    m = [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, n]]
    b = [sxz, syz, sz]
    sol = solve3(m, b)
    if sol is None:
        return None
    a, bb, c = sol
    cx = -a / 2.0
    cy = -bb / 2.0
    r2 = cx * cx + cy * cy - c
    if r2 <= 0:
        return None
    radius = math.sqrt(r2)
    residuals = [abs(math.hypot(x - cx, y - cy) - radius) for x, y, _w in points]
    angles = [math.atan2(y - cy, x - cx) for x, y, _w in points]
    coverage = angular_coverage(angles)
    return {
        "cx_mm": cx,
        "cy_mm": cy,
        "radius_mm": radius,
        "mean_abs_residual_mm": statistics.mean(residuals),
        "max_abs_residual_mm": max(residuals),
        "angular_coverage_deg": coverage,
    }


def solve3(m, b):
    a = [row[:] + [rhs] for row, rhs in zip(m, b)]
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(a[r][col]))
        if abs(a[piv][col]) < 1e-9:
            return None
        a[col], a[piv] = a[piv], a[col]
        div = a[col][col]
        for j in range(col, 4):
            a[col][j] /= div
        for r in range(3):
            if r == col:
                continue
            f = a[r][col]
            for j in range(col, 4):
                a[r][j] -= f * a[col][j]
    return [a[i][3] for i in range(3)]


def angular_coverage(angles):
    if len(angles) < 2:
        return 0.0
    vals = sorted((a + 2 * math.pi) % (2 * math.pi) for a in angles)
    gaps = [vals[i + 1] - vals[i] for i in range(len(vals) - 1)]
    gaps.append(vals[0] + 2 * math.pi - vals[-1])
    return math.degrees(2 * math.pi - max(gaps))


def draw_heatmap(name, mat, points, fit, path):
    rows, cols, sx, sy = TACTILE_SPECS[name]
    cell = 28
    pad = 72
    w = cols * cell + pad * 2
    h = rows * cell + pad * 2 + 70
    img = Image.new("RGB", (w, h), "white")
    d = ImageDraw.Draw(img)
    d.text((18, 14), name, fill=(20, 28, 38), font=font(20))
    maxv = max(mat + [1.0])
    for r in range(rows):
        for c in range(cols):
            v = mat[r * cols + c] if r * cols + c < len(mat) else 0.0
            k = min(1.0, v / maxv)
            color = (int(245 - 210 * k), int(246 - 100 * k), int(248 - 40 * k))
            x0 = pad + c * cell
            y0 = pad + r * cell
            d.rectangle((x0, y0, x0 + cell - 2, y0 + cell - 2), fill=color, outline=(215, 220, 228))
    for x, y, _ww in points:
        c = (x / sx + 0.5) * cols
        r = (y / sy + 0.5) * rows
        px = pad + c * cell
        py = pad + r * cell
        d.ellipse((px - 4, py - 4, px + 4, py + 4), fill=(15, 90, 130))
    if fit:
        # Draw fitted circle in sensor-local coordinates.
        cx = pad + (fit["cx_mm"] / sx + 0.5) * cols * cell
        cy = pad + (fit["cy_mm"] / sy + 0.5) * rows * cell
        rx = fit["radius_mm"] / sx * cols * cell
        ry = fit["radius_mm"] / sy * rows * cell
        d.ellipse((cx - rx, cy - ry, cx + rx, cy + ry), outline=(220, 70, 45), width=3)
        txt = f"fit R={fit['radius_mm']:.1f}mm resid={fit['mean_abs_residual_mm']:.1f}mm coverage={fit['angular_coverage_deg']:.0f}deg"
    else:
        txt = "fit: not enough stable points"
    d.text((18, h - 44), txt, fill=(70, 78, 88), font=font(16))
    img.save(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True)
    ap.add_argument("--segment", default="bottom")
    ap.add_argument("--out", required=True)
    ap.add_argument("--threshold", type=float, default=500.0)
    ap.add_argument("--min-frames", type=int, default=5)
    args = ap.parse_args()

    session = Path(args.session)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    baseline = load_jsonl(session / "baseline_raw.jsonl")
    frames = [
        rec for rec in load_jsonl(session / "frames_raw.jsonl")
        if rec.get("segment") == args.segment or rec.get("phase") == f"manual_{args.segment}"
    ]
    means, _stds = baseline_stats(baseline)
    rows = []
    for name, spec in TACTILE_SPECS.items():
        shape_rows, cols, sx, sy = spec
        n = shape_rows * cols
        max_delta = [0.0] * n
        active_frames = [0] * n
        for rec in frames:
            arr = rec.get("sensors_raw", {}).get(name, [])
            base = means.get(name, [0.0] * len(arr))
            for i, raw in enumerate(arr[:n]):
                d = max(0.0, float(raw) - (base[i] if i < len(base) else 0.0))
                max_delta[i] = max(max_delta[i], d)
                if d >= args.threshold:
                    active_frames[i] += 1
        pts = []
        for i, dlt in enumerate(max_delta):
            if dlt < args.threshold or active_frames[i] < args.min_frames:
                continue
            r, c = divmod(i, cols)
            x = ((c + 0.5) / cols - 0.5) * sx
            y = ((r + 0.5) / shape_rows - 0.5) * sy
            pts.append((x, y, dlt))
        fit = circle_fit(pts)
        draw_heatmap(name, max_delta, pts, fit, out / f"{name}_heatmap.png")
        rows.append({
            "sensor": name,
            "segment": args.segment,
            "frames": len(frames),
            "candidate_points": len(pts),
            "max_delta": max(max_delta or [0.0]),
            "active_taxels_threshold": sum(1 for v in max_delta if v >= args.threshold),
            **(fit or {}),
        })
    with (out / "circle_fit_summary.csv").open("w", newline="", encoding="utf-8") as f:
        fieldnames = sorted({k for row in rows for k in row})
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    best = sorted(
        [r for r in rows if r.get("radius_mm")],
        key=lambda r: (-(r.get("angular_coverage_deg") or 0), r.get("mean_abs_residual_mm") or 1e9),
    )
    report = {
        "session": session.name,
        "segment": args.segment,
        "frames": len(frames),
        "threshold": args.threshold,
        "min_frames": args.min_frames,
        "best_fits": best[:8],
        "interpretation": (
            "A full circle is reliable only when angular_coverage_deg is high and residual is small. "
            "Low coverage means the tactile data contains a local patch/arc, not a complete circular footprint."
        ),
    }
    (out / "circle_fit_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(out)


if __name__ == "__main__":
    main()
