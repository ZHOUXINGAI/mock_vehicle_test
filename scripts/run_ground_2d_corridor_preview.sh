#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_ROOT="${GROUND_2D_PREVIEW_LOG_ROOT:-$REPO_DIR/results/ground_2d_corridor_preview}"
STDOUT_TMP="$(mktemp)"

mkdir -p "$LOG_ROOT"

echo "Running local mock_vehicle_test Ground 2D CorridorPlan preview."
echo "This is offline simulation only; it does not connect to LR24, PX4, MAVROS, or motors."
echo "log_root=$LOG_ROOT"
echo

python3 "$REPO_DIR/scripts/run_ground_2d_corridor_sim.py" \
  --output-dir "$LOG_ROOT" \
  "$@" | tee "$STDOUT_TMP"

OUTPUT_DIR="$(awk -F= '/^output_dir=/{print $2; exit}' "$STDOUT_TMP")"
if [ -z "$OUTPUT_DIR" ] || [ ! -d "$OUTPUT_DIR" ]; then
  echo "Could not find output_dir from simulation output." >&2
  rm -f "$STDOUT_TMP"
  exit 1
fi

cp "$STDOUT_TMP" "$OUTPUT_DIR/run_stdout.log"
rm -f "$STDOUT_TMP"
ln -sfn "$OUTPUT_DIR" "$LOG_ROOT/latest"

echo
echo "Ground 2D CorridorPlan preview saved:"
echo "  output: $OUTPUT_DIR"
echo "  latest: $LOG_ROOT/latest"
