# RH56DFTP tactile toolkit

Tools for RH56DFTP tactile sensor checks, realtime RViz visualization, recording, offline filtering, and basic contact-shape analysis.

This repository contains code and robot visualization assets only. Recorded datasets, generated PNGs/GIFs, and large report outputs are intentionally not included.

## Contents

- `rh56_tactile/` - shared tactile decoding, URDF geometry, FK helpers, and noise filtering
- `scripts/rviz_urdf_tactile_live.py` - realtime tactile RViz visualization on the URDF hand
- `scripts/rviz_urdf_tactile_live_filtered.py` - realtime RViz visualization with cluster/temporal noise filtering
- `scripts/analyze_noise_filtered_session.py` - offline filtering for recorded sessions
- `scripts/check_all_tactile_sensors.py` - tactile sensor health check
- `scripts/record_tactile_slide_session.py` - manual slide/contact recording
- `scripts/adaptive_grasp_force_record_session.py` - adaptive grasp recording with force/tactile data
- `scripts/generate_tactile_shape_report.py` - report-style 3D/HTML/GIF visualization generator
- `rviz/` - RViz configs
- `urdf/inspire_hand/` - left/right URDF files and STL meshes
- `launch/` - short launch commands for realtime RViz

## Environment

Expected runtime:

- Ubuntu with ROS 2 Humble
- Python 3
- `RH56DFTP` Python API available in `PYTHONPATH` or installed in the active environment
- network access to the hand, usually `192.168.123.211:6000`

Install non-ROS Python helpers:

```bash
pip install -r requirements.txt
```

## Realtime RViz

Filtered left hand, palm-facing view:

```bash
./launch/start_live_filtered_left_palm.sh 192.168.123.211 80
```

Filtered right hand:

```bash
./launch/start_live_filtered_right_palm.sh 192.168.123.211 80
```

Raw, unfiltered left hand:

```bash
./launch/start_live_left_palm.sh 192.168.123.211 80
```

Raw, unfiltered right hand:

```bash
./launch/start_live_right_palm.sh 192.168.123.211 80
```

At startup, keep the hand untouched during baseline collection. The filtered launch prints `raw_taxels` and `filtered_taxels`, which helps tune the threshold.

## Sensor Check

```bash
python3 scripts/check_all_tactile_sensors.py \
  --ip 192.168.123.211 \
  --baseline-frames 10 \
  --duration 30 \
  --threshold 80
```

## Recording

Manual slide/contact recording:

```bash
python3 scripts/record_tactile_slide_session.py \
  --ip 192.168.123.211 \
  --object-name cup_small \
  --session-name cup_small_slide_manual_01 \
  --out-root data/tactile_sessions \
  --duration 25 \
  --baseline-sec 3
```

Adaptive grasp recording:

```bash
python3 scripts/adaptive_grasp_force_record_session.py \
  --ip 192.168.123.211 \
  --object-name cup_small \
  --session-name cup_small_grasp_01 \
  --mode grasp_static \
  --out-root data/tactile_sessions \
  --force-threshold 60 \
  --grasp-step 40 \
  --hold-sec 10 \
  --baseline-sec 3
```

## Offline Noise Filtering

```bash
python3 scripts/analyze_noise_filtered_session.py \
  --session data/tactile_sessions/cup_small_circle_contacts_01 \
  --segment bottom \
  --out outputs/cup_small_circle_contacts_01_filtered
```

The output includes:

- `filtered_frames.jsonl`
- `filter_frame_summary.csv`
- `filter_summary.json`
- `heatmaps/*_raw_vs_filtered.png`

## Notes

The first practical filter is intentionally simple and tunable:

- baseline mean/std per taxel
- adaptive per-taxel threshold
- connected components on each sensor matrix
- temporal persistence across frames

For clean circle/contact-shape experiments, record on one known matrix, preferably `palm_8x14`, with a no-touch baseline and short labeled segments.
