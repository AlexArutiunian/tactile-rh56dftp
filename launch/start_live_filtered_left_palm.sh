#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash

IP="${1:-192.168.123.211}"
THRESHOLD="${2:-80}"

mkdir -p generated
python3 scripts/prepare_urdf_for_rviz.py \
  --urdf urdf/inspire_hand/FTP_left_hand.urdf \
  --out-dir generated >/tmp/rh56_left_filtered_rviz_urdf_paths.txt
PARAM_FILE="$(tail -n 1 /tmp/rh56_left_filtered_rviz_urdf_paths.txt)"
URDF_FILE="$(head -n 1 /tmp/rh56_left_filtered_rviz_urdf_paths.txt)"

python3 scripts/rviz_urdf_tactile_live_filtered.py \
  --side left \
  --urdf "$URDF_FILE" \
  --frame left_wrist_yaw_link \
  --ip "$IP" \
  --threshold "$THRESHOLD" \
  --baseline-sec 3 \
  --hz 8 \
  --min-component-size 2 \
  --min-component-size-3x3 1 \
  --temporal-frames 2 \
  --marker-size 0.008 \
  --flat-colors \
  --print-status &
NODE_PID=$!

cleanup() {
  kill "$NODE_PID" 2>/dev/null || true
  wait "$NODE_PID" 2>/dev/null || true
}
trap cleanup EXIT

sleep 2
rviz2 -d rviz/left_hand_urdf_tactile_palm.rviz --ros-args --params-file "$PARAM_FILE"

