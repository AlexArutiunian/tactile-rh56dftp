#!/usr/bin/env python3
import argparse
import io
import json
import logging
import sys
import time
from pathlib import Path

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8")
sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8")

logging.getLogger("RH56DFTP").setLevel(logging.ERROR)
logging.getLogger("pymodbus").setLevel(logging.ERROR)
logging.getLogger().setLevel(logging.ERROR)

from RH56DFTP.RH56DFTP_TCP import RH56DFTP_TCP


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


def raw_to_list(raw):
    if raw is None:
        return []
    if isinstance(raw, bytes):
        out = []
        for i in range(0, len(raw), 2):
            if i + 1 < len(raw):
                out.append(int.from_bytes(raw[i:i + 2], "little", signed=False))
        return out
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
    return raw_to_list(raw)[:rows * cols]


def read_all(client):
    out = {}
    for alias, (register, rows, cols) in TACTILE_SPECS.items():
        try:
            out[alias] = decode_taxels(client.get(register), rows, cols)
        except Exception as exc:
            out[alias] = {"error": f"{type(exc).__name__}: {exc}"}
    return out


def add_sample(stats, baseline, sample, threshold):
    for alias, values in sample.items():
        s = stats.setdefault(
            alias,
            {
                "samples": 0,
                "read_errors": 0,
                "len": 0,
                "raw_max": 0.0,
                "delta_abs_max": 0.0,
                "delta_signed_at_max": 0.0,
                "max_idx": None,
                "active_taxels_peak": 0,
            },
        )
        if isinstance(values, dict) and "error" in values:
            s["read_errors"] += 1
            s["last_error"] = values["error"]
            continue
        s["samples"] += 1
        s["len"] = max(s["len"], len(values))
        if values:
            raw_max = max(float(v) for v in values)
            s["raw_max"] = max(s["raw_max"], raw_max)
        base = baseline.get(alias, [])
        n = min(len(values), len(base))
        if n == 0:
            continue
        active = 0
        for idx in range(n):
            delta = float(values[idx]) - float(base[idx])
            if abs(delta) >= threshold:
                active += 1
            if abs(delta) > s["delta_abs_max"]:
                s["delta_abs_max"] = abs(delta)
                s["delta_signed_at_max"] = delta
                s["max_idx"] = idx
        s["active_taxels_peak"] = max(s["active_taxels_peak"], active)


def compute_baseline(client, frames, hz):
    sums = {}
    counts = {}
    dt = 1.0 / hz
    for _ in range(frames):
        sample = read_all(client)
        for alias, values in sample.items():
            if isinstance(values, dict) or not values:
                continue
            if alias not in sums:
                sums[alias] = [0.0] * len(values)
                counts[alias] = 0
            n = min(len(values), len(sums[alias]))
            for i in range(n):
                sums[alias][i] += float(values[i])
            counts[alias] += 1
        time.sleep(dt)
    return {alias: [v / max(1, counts[alias]) for v in vals] for alias, vals in sums.items()}


def status_for(row, threshold):
    if row.get("read_errors", 0) and row.get("samples", 0) == 0:
        return "NO_READ"
    if row.get("len", 0) == 0:
        return "EMPTY"
    if row.get("delta_abs_max", 0.0) >= threshold:
        return "OK_TOUCH"
    if row.get("raw_max", 0.0) > 0:
        return "READS_RAW_NO_DELTA"
    return "ZERO"


def print_recommendations(stats):
    noisy = []
    zero = []
    for alias in TACTILE_SPECS:
        row = stats.get(alias, {})
        if row.get("len", 0) == 0 or row.get("raw_max", 0.0) == 0:
            zero.append(alias)
            continue
        idle_delta = row.get("delta_abs_max", 0.0)
        if idle_delta > 0:
            noisy.append((idle_delta, alias))
    noisy.sort(reverse=True)
    if noisy:
        worst = noisy[0][0]
        recommended = max(80.0, worst * 1.5)
        print("\nObserved signal range:")
        print(f"- worst observed abs(delta): {worst:.0f}")
        print("- If this run was no-touch, set touch threshold above this noise/drift.")
        print("- If you were pressing sensors, this is real signal + drift, not an idle-noise estimate.")
        print(f"- Conservative threshold only for a no-touch calibration run would be > {recommended:.0f}.")
        print("- strongest nonzero channels:")
        for delta, alias in noisy[:6]:
            print(f"  {alias:24s} abs(delta)={delta:.0f}")
    if zero:
        print("\nAlways-zero channels in this run:")
        print(", ".join(zero))


def print_table(stats, threshold):
    print("\n" + "=" * 112)
    print("TACTILE SENSOR CHECK SUMMARY")
    print("=" * 112)
    print(f"{'sensor':24s} {'len':>5s} {'raw_max':>9s} {'max_delta':>10s} {'idx':>5s} {'active':>7s} {'status':>18s}")
    print("-" * 112)
    for alias in TACTILE_SPECS:
        row = stats.get(alias, {})
        status = status_for(row, threshold)
        print(
            f"{alias:24s} "
            f"{int(row.get('len', 0)):5d} "
            f"{row.get('raw_max', 0.0):9.0f} "
            f"{row.get('delta_signed_at_max', 0.0):+10.0f} "
            f"{str(row.get('max_idx')):>5s} "
            f"{int(row.get('active_taxels_peak', 0)):7d} "
            f"{status:>18s}"
        )
    print("-" * 112)
    print(f"threshold = {threshold:.0f}. Touch is counted when abs(value - baseline) >= threshold.")
    print_recommendations(stats)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="192.168.123.210")
    parser.add_argument("--port", type=int, default=6000)
    parser.add_argument("--hz", type=float, default=12.0)
    parser.add_argument("--baseline-frames", type=int, default=30)
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--threshold", type=float, default=80.0)
    parser.add_argument("--out", default="scripts/tactile_sensor_check_summary.json")
    parser.add_argument("--quiet", action="store_true", help="Do not print progress while sampling.")
    parser.add_argument("--progress-interval", type=float, default=5.0, help="Seconds between progress lines.")
    args = parser.parse_args()

    print(f"Connect {args.ip}:{args.port}", flush=True)
    client = RH56DFTP_TCP(host=args.ip, port=args.port)
    try:
        print(f"Baseline {args.baseline_frames} frames. Не трогай руку.", flush=True)
        baseline = compute_baseline(client, args.baseline_frames, args.hz)
        print("Baseline ready.", flush=True)
        print(f"Теперь {args.duration:.0f} секунд нажимай все зоны: каждый палец, tip/pad, отдельно большой палец.", flush=True)

        stats = {}
        start = time.time()
        next_print = start
        dt = 1.0 / args.hz
        while time.time() - start < args.duration:
            sample = read_all(client)
            add_sample(stats, baseline, sample, args.threshold)
            now = time.time()
            if not args.quiet and now >= next_print:
                left = max(0.0, args.duration - (now - start))
                active = [
                    alias for alias, row in stats.items()
                    if row.get("delta_abs_max", 0.0) >= args.threshold
                ]
                print(f"left={left:5.1f}s active_sensors={len(active):2d}: {', '.join(active[-6:])}", flush=True)
                next_print = now + max(0.5, args.progress_interval)
            time.sleep(dt)

        print_table(stats, args.threshold)
        out = {
            "threshold": args.threshold,
            "duration": args.duration,
            "baseline_frames": args.baseline_frames,
            "stats": stats,
        }
        out_path = Path(args.out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"\nSaved: {out_path}", flush=True)
    finally:
        try:
            client.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
