#!/usr/bin/env python3
import argparse
import csv
import io
import json
import logging
import re
import sys
import time
from datetime import datetime
from pathlib import Path

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8")
sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8")

logging.getLogger("RH56DFTP").setLevel(logging.ERROR)
logging.getLogger("pymodbus").setLevel(logging.ERROR)

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
    return raw_to_list(raw)[:rows * cols]


def read_tactile_raw(client):
    sensors = {}
    for alias, (reg, rows, cols) in TACTILE_SPECS.items():
        try:
            sensors[alias] = decode_taxels(client.get(reg), rows, cols)
        except Exception:
            sensors[alias] = []
    return sensors


def read_forces(client):
    vals = []
    for i in range(6):
        try:
            vals.append(float(client.get(f"FORCE_ACT({i})")))
        except Exception:
            vals.append(0.0)
    return vals


def set_positions(client, positions):
    for i, v in enumerate(positions):
        client.set(f"POS_SET({i})", int(v))


def write_frame(f, phase, frame_id, positions, forces, force_baseline, raw_sensors, extra=None):
    rec = {
        "t": time.time(),
        "frame_id": frame_id,
        "phase": phase,
        "positions": list(positions),
        "forces": list(forces),
        "force_baseline": list(force_baseline) if force_baseline is not None else None,
        "force_delta": [
            float(forces[i]) - float(force_baseline[i])
            for i in range(6)
        ] if force_baseline is not None else None,
        "sensors_raw": raw_sensors,
    }
    if extra:
        rec.update(extra)
    f.write(json.dumps(rec, ensure_ascii=False) + "\n")
    f.flush()


def sanitize_session_name(name):
    return re.sub(r"[^A-Za-z0-9А-Яа-яЁё_.-]+", "_", name.strip())


def parse_segments(text, default_duration):
    if not text.strip():
        return [("manual_slide", float(default_duration))]
    out = []
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        if ":" in item:
            name, sec = item.split(":", 1)
            name = sanitize_session_name(name) or "segment"
            out.append((name, float(sec)))
        else:
            out.append((sanitize_session_name(item) or "segment", float(default_duration)))
    return out or [("manual_slide", float(default_duration))]


def main():
    parser = argparse.ArgumentParser(description="Record tactile-only slide/roll/contact session without automatic grasp closing.")
    parser.add_argument("--ip", default="192.168.123.211")
    parser.add_argument("--port", type=int, default=6000)
    parser.add_argument("--object-name", required=True)
    parser.add_argument("--trial", type=int, default=1)
    parser.add_argument("--mode", default="slide_manual")
    parser.add_argument("--out-root", default="scripts/tactile_grasp_dataset_new_hand")
    parser.add_argument("--session-name", default="")
    parser.add_argument("--notes", default="")
    parser.add_argument("--baseline-sec", type=float, default=3.0)
    parser.add_argument("--baseline-hz", type=float, default=15.0)
    parser.add_argument("--record-sec", type=float, default=20.0)
    parser.add_argument("--record-hz", type=float, default=15.0)
    parser.add_argument("--segments", default="", help='Comma list like "bottom:10,rim:10,side:10". Each segment gets its own phase label.')
    parser.add_argument("--open-pos", type=int, default=0)
    parser.add_argument("--thumb-rot-pos", type=int, default=0)
    parser.add_argument("--set-initial-pose", action="store_true", help="Send POS_SET once before baseline. Otherwise do not move motors at all.")
    parser.add_argument("--no-enter", action="store_true")
    args = parser.parse_args()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if args.session_name.strip():
        session_name = sanitize_session_name(args.session_name)
    else:
        session_name = f"{timestamp}_{args.object_name}_trial{args.trial:03d}_{args.mode}"

    session_dir = Path(args.out_root) / session_name
    session_dir.mkdir(parents=True, exist_ok=True)

    initial_positions = [args.open_pos] * 6
    initial_positions[5] = args.thumb_rot_pos

    metadata = {
        "session_name": session_name,
        "object_name": args.object_name,
        "trial": args.trial,
        "mode": args.mode,
        "notes": args.notes,
        "ip": args.ip,
        "port": args.port,
        "baseline_sec": args.baseline_sec,
        "record_sec": args.record_sec,
        "record_hz": args.record_hz,
        "segments": parse_segments(args.segments, args.record_sec),
        "open_pos": args.open_pos,
        "thumb_rot_pos": args.thumb_rot_pos,
        "set_initial_pose": args.set_initial_pose,
        "important": "TACTILE-only manual contact/slide recording. No automatic grasp closing is performed.",
        "tactile_specs": {
            alias: {"register": reg, "rows_label": rows, "cols_label": cols}
            for alias, (reg, rows, cols) in TACTILE_SPECS.items()
        },
        "files": {
            "baseline_raw": "baseline_raw.jsonl",
            "frames_raw": "frames_raw.jsonl",
            "grasp_trace_csv": "grasp_trace.csv",
            "metadata": "metadata.json",
        },
    }
    (session_dir / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")

    print("=" * 90)
    print("TACTILE-ONLY MANUAL SLIDE/CONTACT RECORD")
    print("=" * 90)
    print("Session:", session_dir)
    print("Object:", args.object_name)
    print("Mode:", args.mode)
    print()
    print("ВАЖНО:")
    print("  - автоматического закрытия пальцев нет")
    print("  - во время record вручную катай/сдвигай объект по сенсорам")
    print("  - сохраняются positions/forces/force_delta/tactile raw в том же формате")
    print()

    client = RH56DFTP_TCP(host=args.ip, port=args.port)
    frame_id = 0
    try:
        if args.set_initial_pose:
            print("Setting initial pose once:", initial_positions)
            set_positions(client, initial_positions)
            time.sleep(1.0)

        if not args.no_enter:
            input("Пустая рука, без касаний. Нажми Enter для baseline... ")

        baseline_frames = max(1, int(args.baseline_sec * args.baseline_hz))
        force_samples = []
        print(f"Baseline {baseline_frames} frames...")
        with (session_dir / "baseline_raw.jsonl").open("w", encoding="utf-8") as fbase:
            for i in range(baseline_frames):
                forces = read_forces(client)
                raw = read_tactile_raw(client)
                force_samples.append(forces)
                write_frame(
                    fbase,
                    phase="baseline",
                    frame_id=i,
                    positions=initial_positions,
                    forces=forces,
                    force_baseline=None,
                    raw_sensors=raw,
                )
                time.sleep(1.0 / args.baseline_hz)

        force_baseline = [
            sum(sample[i] for sample in force_samples) / max(1, len(force_samples))
            for i in range(6)
        ]
        print("Force baseline:", [round(x, 1) for x in force_baseline])

        if not args.no_enter:
            input("Теперь вручную катай/сдвигай объект по сенсорам. Нажми Enter — начну запись... ")

        frames_path = session_dir / "frames_raw.jsonl"
        trace_path = session_dir / "grasp_trace.csv"
        segments = parse_segments(args.segments, args.record_sec)
        total_frames = sum(max(1, int(sec * args.record_hz)) for _name, sec in segments)
        print(f"Recording {sum(sec for _name, sec in segments):.1f}s, {total_frames} frames...")

        with trace_path.open("w", encoding="utf-8", newline="") as ftrace, \
             frames_path.open("w", encoding="utf-8") as fframes:
            writer = csv.DictWriter(
                ftrace,
                fieldnames=[
                    "t", "step", "phase",
                    "pos0", "pos1", "pos2", "pos3", "pos4", "pos5",
                    "force0", "force1", "force2", "force3", "force4", "force5",
                    "fdelta0", "fdelta1", "fdelta2", "fdelta3", "fdelta4", "fdelta5",
                ],
            )
            writer.writeheader()

            global_step = 0
            for segment_name, segment_sec in segments:
                phase = f"manual_{segment_name}"
                if not args.no_enter and len(segments) > 1:
                    input(f"Segment `{segment_name}` {segment_sec:.1f}s. Подготовь контакт и нажми Enter... ")
                segment_frames = max(1, int(segment_sec * args.record_hz))
                print(f"Segment {segment_name}: {segment_frames} frames")
                for step in range(segment_frames):
                    now = time.time()
                    forces = read_forces(client)
                    fdelta = [float(forces[i]) - float(force_baseline[i]) for i in range(6)]
                    raw = read_tactile_raw(client)
                    write_frame(
                        fframes,
                        phase=phase,
                        frame_id=frame_id,
                        positions=initial_positions,
                        forces=forces,
                        force_baseline=force_baseline,
                        raw_sensors=raw,
                        extra={"manual_motion": True, "segment": segment_name},
                    )
                    frame_id += 1

                    writer.writerow({
                        "t": now,
                        "step": global_step,
                        "phase": phase,
                        "pos0": initial_positions[0], "pos1": initial_positions[1], "pos2": initial_positions[2],
                        "pos3": initial_positions[3], "pos4": initial_positions[4], "pos5": initial_positions[5],
                        "force0": forces[0], "force1": forces[1], "force2": forces[2],
                        "force3": forces[3], "force4": forces[4], "force5": forces[5],
                        "fdelta0": fdelta[0], "fdelta1": fdelta[1], "fdelta2": fdelta[2],
                        "fdelta3": fdelta[3], "fdelta4": fdelta[4], "fdelta5": fdelta[5],
                    })
                    if global_step % max(1, int(args.record_hz * 2)) == 0:
                        print(f"frame {global_step}/{total_frames} phase={phase} fΔ={[round(x, 1) for x in fdelta[:5]]}")
                    global_step += 1
                    time.sleep(1.0 / args.record_hz)

        print("\nDONE")
        print("Session:", session_dir)
        print("Raw frames:", frames_path)
        print("Trace:", trace_path)
    finally:
        try:
            client.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
