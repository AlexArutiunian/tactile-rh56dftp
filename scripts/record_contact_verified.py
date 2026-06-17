#!/usr/bin/env python3
import argparse
import json
import logging
import time
from datetime import datetime
from pathlib import Path

logging.getLogger("RH56DFTP").setLevel(logging.ERROR)
logging.getLogger("pymodbus").setLevel(logging.ERROR)

from RH56DFTP.RH56DFTP_TCP import RH56DFTP_TCP

from rh56_tactile_common import TACTILE_SPECS, baseline_means, read_forces, read_positions, read_tactile


def write_jsonl(path, rows):
    with path.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")


def main():
    parser = argparse.ArgumentParser(description="Record tactile session after live RViz verification.")
    parser.add_argument("--ip", default="192.168.123.211")
    parser.add_argument("--port", type=int, default=6000)
    parser.add_argument("--object-name", required=True)
    parser.add_argument("--session-name", required=True)
    parser.add_argument("--mode", default="verified_contact")
    parser.add_argument("--out-root", default="data")
    parser.add_argument("--record-sec", type=float, default=8.0)
    parser.add_argument("--record-hz", type=float, default=15.0)
    parser.add_argument("--baseline-sec", type=float, default=2.0)
    parser.add_argument("--segment", default="contact")
    parser.add_argument("--positions", default="0,0,0,0,0,0")
    parser.add_argument("--read-pos-set", action="store_true")
    args = parser.parse_args()

    positions_fallback = [float(x) for x in args.positions.split(",")]
    if len(positions_fallback) != 6:
        raise ValueError("--positions must contain 6 comma-separated values")

    out_root = Path(args.out_root)
    session_dir = out_root / args.session_name
    if session_dir.exists():
        raise SystemExit(f"Refusing to overwrite existing session: {session_dir}")
    session_dir.mkdir(parents=True)

    client = RH56DFTP_TCP(args.ip, args.port)
    input("Baseline: ничего не трогай, затем Enter...")
    baseline_frames = max(1, int(args.baseline_sec * args.record_hz))
    baseline_rows = []
    force_samples = []
    for i in range(baseline_frames):
        tactile = read_tactile(client)
        forces = read_forces(client)
        positions = read_positions(client, positions_fallback) if args.read_pos_set else positions_fallback
        force_samples.append(forces)
        baseline_rows.append({
            "t": time.time(),
            "frame_id": i,
            "phase": "baseline",
            "positions": positions,
            "forces": forces,
            "sensors_raw": tactile,
        })
        time.sleep(1.0 / args.record_hz)
    force_baseline = [
        sum(sample[i] for sample in force_samples) / max(1, len(force_samples))
        for i in range(6)
    ]
    tactile_baseline = baseline_means([row["sensors_raw"] for row in baseline_rows])

    input("Поставь объект как проверил в RViz. Enter = старт записи...")
    frames = []
    total = max(1, int(args.record_sec * args.record_hz))
    for i in range(total):
        tactile = read_tactile(client)
        forces = read_forces(client)
        positions = read_positions(client, positions_fallback) if args.read_pos_set else positions_fallback
        frames.append({
            "t": time.time(),
            "frame_id": i,
            "phase": f"manual_{args.segment}",
            "segment": args.segment,
            "positions": positions,
            "forces": forces,
            "force_baseline": force_baseline,
            "force_delta": [float(forces[j]) - float(force_baseline[j]) for j in range(6)],
            "sensors_raw": tactile,
        })
        if i % max(1, int(args.record_hz)) == 0:
            print(f"record {i}/{total}")
        time.sleep(1.0 / args.record_hz)

    metadata = {
        "session_name": args.session_name,
        "object_name": args.object_name,
        "mode": args.mode,
        "ip": args.ip,
        "port": args.port,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "record_sec": args.record_sec,
        "record_hz": args.record_hz,
        "baseline_sec": args.baseline_sec,
        "segment": args.segment,
        "decoded_taxels": True,
        "important": "Recorded after live RViz verification. sensors_raw arrays are decoded physical taxels, not doubled byte spans.",
        "tactile_specs": {
            alias: {"register": reg, "rows_label": rows, "cols_label": cols}
            for alias, (reg, rows, cols) in TACTILE_SPECS.items()
        },
        "files": {
            "metadata": "metadata.json",
            "baseline_raw": "baseline_raw.jsonl",
            "frames_raw": "frames_raw.jsonl",
        },
    }
    (session_dir / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    write_jsonl(session_dir / "baseline_raw.jsonl", baseline_rows)
    write_jsonl(session_dir / "frames_raw.jsonl", frames)
    print(f"Saved: {session_dir}")


if __name__ == "__main__":
    main()
