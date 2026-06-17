#!/usr/bin/env python3
import argparse
import csv
import json
import logging
import sys
import io
import time
import re
from pathlib import Path
from datetime import datetime

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

FINGER_NAMES = {
    0: "small",
    1: "ring",
    2: "middle",
    3: "index",
    4: "thumb_flex",
    5: "thumb_rot",
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

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--ip", default="192.168.123.210")
    p.add_argument("--port", type=int, default=6000)

    p.add_argument("--object-name", required=True)
    p.add_argument("--trial", type=int, default=1)
    p.add_argument("--mode", default="force_only_power_grasp")
    p.add_argument("--out-root", default="tactile_grasp_dataset")
    p.add_argument("--session-name", default="", help="точное имя папки сессии; если пусто, имя создаётся автоматически")
    p.add_argument("--notes", default="")

    p.add_argument("--baseline-sec", type=float, default=3.0)
    p.add_argument("--baseline-hz", type=float, default=15.0)

    p.add_argument("--open-pos", type=int, default=0)
    p.add_argument("--max-pos", type=int, default=1800)
    p.add_argument("--thumb-rot-pos", type=int, default=0)

    p.add_argument("--grasp-step", type=int, default=80)
    p.add_argument("--grasp-sleep", type=float, default=0.08)
    p.add_argument("--max-close-sec", type=float, default=18.0)

    # Только FORCE_ACT останавливает палец. Tactile только сохраняется.
    p.add_argument("--force-threshold", type=float, default=120.0)
    p.add_argument("--disable-force-stop", action="store_true")

    p.add_argument("--hold-sec", type=float, default=8.0)
    p.add_argument("--hold-hz", type=float, default=15.0)

    p.add_argument("--no-release-at-end", action="store_true")
    p.add_argument("--no-enter", action="store_true")

    args = p.parse_args()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    if args.session_name.strip():
        # безопасное имя папки: буквы, цифры, подчёркивания, дефисы
        session_name = re.sub(r"[^A-Za-z0-9А-Яа-яЁё_.-]+", "_", args.session_name.strip())
    else:
        session_name = f"{timestamp}_{args.object_name}_trial{args.trial:03d}_{args.mode}"

    session_dir = Path(args.out_root) / session_name
    session_dir.mkdir(parents=True, exist_ok=True)

    metadata = {
        "session_name": session_name,
        "object_name": args.object_name,
        "trial": args.trial,
        "mode": args.mode,
        "notes": args.notes,
        "ip": args.ip,
        "port": args.port,
        "baseline_sec": args.baseline_sec,
        "force_threshold": args.force_threshold,
        "disable_force_stop": args.disable_force_stop,
        "open_pos": args.open_pos,
        "max_pos": args.max_pos,
        "thumb_rot_pos": args.thumb_rot_pos,
        "important": "TACTILE is recorded only; it does NOT stop fingers in this script.",
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

    with (session_dir / "metadata.json").open("w", encoding="utf-8") as f:
        json.dump(metadata, f, ensure_ascii=False, indent=2)

    print("=" * 90)
    print("FORCE-ONLY POWER GRASP + TACTILE RAW RECORD")
    print("=" * 90)
    print("Session:", session_dir)
    print("Object:", args.object_name)
    print()
    print("ВАЖНО:")
    print("  - пальцы двигаются через POS_SET")
    print("  - SPEED_SET/FORCE_SET не используются")
    print("  - остановка пальцев только по FORCE_ACT")
    print("  - TACTILE_* матрицы сохраняются, но не стопают хват")
    print()

    client = RH56DFTP_TCP(host=args.ip, port=args.port)

    frame_id = 0
    try:
        open_positions = [args.open_pos] * 6
        open_positions[5] = args.thumb_rot_pos

        print("🖐️ Открываю руку через POS_SET...")
        set_positions(client, open_positions)
        time.sleep(1.0)

        if not args.no_enter:
            input("Пустая рука, без касаний. Нажми Enter для baseline... ")

        baseline_frames = max(1, int(args.baseline_sec * args.baseline_hz))
        force_samples = []

        print(f"📏 Baseline {baseline_frames} frames...")
        with (session_dir / "baseline_raw.jsonl").open("w", encoding="utf-8") as fbase:
            for i in range(baseline_frames):
                forces = read_forces(client)
                raw = read_tactile_raw(client)
                force_samples.append(forces)

                write_frame(
                    fbase,
                    phase="baseline",
                    frame_id=i,
                    positions=open_positions,
                    forces=forces,
                    force_baseline=None,
                    raw_sensors=raw,
                )
                time.sleep(1.0 / args.baseline_hz)

        force_baseline = [
            sum(sample[i] for sample in force_samples) / max(1, len(force_samples))
            for i in range(6)
        ]

        print("✅ Force baseline:", [round(x, 1) for x in force_baseline])

        if not args.no_enter:
            input("Теперь вставь предмет. Нажми Enter — начну закрывать пальцы... ")

        current = [args.open_pos] * 6
        current[5] = args.thumb_rot_pos

        close_targets = [
            args.max_pos,
            args.max_pos,
            args.max_pos,
            args.max_pos,
            args.max_pos,
            args.thumb_rot_pos,
        ]

        frozen = [False, False, False, False, False, True]
        moved_once = [False] * 6
        freeze_reason = [""] * 6

        trace_path = session_dir / "grasp_trace.csv"
        frames_path = session_dir / "frames_raw.jsonl"

        with trace_path.open("w", encoding="utf-8", newline="") as ftrace, \
             frames_path.open("w", encoding="utf-8") as fframes:

            writer = csv.DictWriter(ftrace, fieldnames=[
                "t", "step", "phase",
                "pos0", "pos1", "pos2", "pos3", "pos4", "pos5",
                "force0", "force1", "force2", "force3", "force4", "force5",
                "fdelta0", "fdelta1", "fdelta2", "fdelta3", "fdelta4", "fdelta5",
                "frozen0", "frozen1", "frozen2", "frozen3", "frozen4",
                "reason0", "reason1", "reason2", "reason3", "reason4",
            ])
            writer.writeheader()

            print("\n✊ Закрываю пальцы...")
            close_start = time.time()
            step = 0

            while True:
                now = time.time()

                if now - close_start > args.max_close_sec:
                    print("⏱ max-close-sec reached")
                    break

                forces = read_forces(client)
                fdelta = [float(forces[i]) - float(force_baseline[i]) for i in range(6)]
                raw = read_tactile_raw(client)

                for i in range(5):
                    if frozen[i]:
                        continue

                    if (not args.disable_force_stop) and moved_once[i] and fdelta[i] >= args.force_threshold:
                        frozen[i] = True
                        freeze_reason[i] = f"force Δ={fdelta[i]:.1f}"
                        print(f"🧊 freeze finger {i} {FINGER_NAMES[i]} at pos={current[i]} reason={freeze_reason[i]}")
                        continue

                    if current[i] < close_targets[i]:
                        current[i] = min(close_targets[i], current[i] + args.grasp_step)
                        moved_once[i] = True
                    else:
                        frozen[i] = True
                        freeze_reason[i] = "target_reached"

                set_positions(client, current)

                write_frame(
                    fframes,
                    phase="close",
                    frame_id=frame_id,
                    positions=current,
                    forces=forces,
                    force_baseline=force_baseline,
                    raw_sensors=raw,
                    extra={
                        "frozen": frozen,
                        "freeze_reason": freeze_reason,
                    }
                )
                frame_id += 1

                writer.writerow({
                    "t": now,
                    "step": step,
                    "phase": "close",
                    "pos0": current[0], "pos1": current[1], "pos2": current[2],
                    "pos3": current[3], "pos4": current[4], "pos5": current[5],
                    "force0": forces[0], "force1": forces[1], "force2": forces[2],
                    "force3": forces[3], "force4": forces[4], "force5": forces[5],
                    "fdelta0": fdelta[0], "fdelta1": fdelta[1], "fdelta2": fdelta[2],
                    "fdelta3": fdelta[3], "fdelta4": fdelta[4], "fdelta5": fdelta[5],
                    "frozen0": frozen[0], "frozen1": frozen[1], "frozen2": frozen[2],
                    "frozen3": frozen[3], "frozen4": frozen[4],
                    "reason0": freeze_reason[0], "reason1": freeze_reason[1],
                    "reason2": freeze_reason[2], "reason3": freeze_reason[3],
                    "reason4": freeze_reason[4],
                })
                ftrace.flush()

                print(
                    f"step={step:03d} pos={current[:5]} "
                    f"fΔ={[round(x,1) for x in fdelta[:5]]} "
                    f"frozen={frozen[:5]}"
                )

                step += 1

                if all(frozen[:5]):
                    print("✅ all fingers frozen/done")
                    break

                time.sleep(args.grasp_sleep)

            print(f"\n🤲 Hold recording {args.hold_sec:.1f}s...")
            hold_frames = max(1, int(args.hold_sec * args.hold_hz))

            for h in range(hold_frames):
                forces = read_forces(client)
                raw = read_tactile_raw(client)

                write_frame(
                    fframes,
                    phase="hold",
                    frame_id=frame_id,
                    positions=current,
                    forces=forces,
                    force_baseline=force_baseline,
                    raw_sensors=raw,
                    extra={
                        "frozen": frozen,
                        "freeze_reason": freeze_reason,
                    }
                )
                frame_id += 1

                if h % max(1, int(args.hold_hz)) == 0:
                    fdelta = [float(forces[i]) - float(force_baseline[i]) for i in range(6)]
                    print(f"hold frame {h}/{hold_frames} fΔ={[round(x,1) for x in fdelta[:5]]}")

                time.sleep(1.0 / args.hold_hz)

        if not args.no_release_at_end:
            print("🖐️ Разжимаю руку в конце...")
            set_positions(client, open_positions)
            time.sleep(1.0)

        print("\n✅ DONE")
        print("Session:", session_dir)
        print("Raw frames:", session_dir / "frames_raw.jsonl")
        print("Trace:", session_dir / "grasp_trace.csv")

    finally:
        try:
            client.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
