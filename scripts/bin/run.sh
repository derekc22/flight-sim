#!/bin/bash
set -e

source .env

usage() {
	exit_code="$1"
	cat >&2 <<EOF
USAGE: $0
  -a <AIRCRAFT>
  [-d DATA_BOOL]
  [-p PLOT_BOOL]
  [-m ANALYSIS_BOOL]
  -o <OUT_DIR>
  [-x FAST_BOOL]
  [-y HEADLESS_BOOL]
  [-b BUILD_BOOL]
EOF
	exit "$exit_code"
}

while getopts "a:dpo:mxybh" opt; do
	case "$opt" in
		a) AIRCRAFT="$OPTARG" ;;
		d) DATA_BOOL=1 ;;
		p) PLOT_BOOL=1 ;;
		o) OUT_DIR="$OPTARG" ;;
		m) ANALYSIS_BOOL=1 ;;
		x) FAST_BOOL=1 ;;
		y) HEADLESS_BOOL=1 ;;
		b) BUILD_BOOL=1 ;;
			h) usage 0 ;;
			?) usage 1 ;;
	esac
done

# set defaults for flags
: "${DATA_BOOL:=0}"
: "${PLOT_BOOL:=0}"
: "${OUT_DIR:=$(date +"%Y%b%d_%H-%M-%S")}"
: "${ANALYSIS_BOOL:=0}"
: "${FAST_BOOL:=0}"
: "${HEADLESS_BOOL:=0}"
: "${BUILD_BOOL:=0}"

DATA_DIR_PATH="$PROJ_PATH/results/data/$OUT_DIR"
LOG_DIR_PATH="$PROJ_PATH/results/logs/$OUT_DIR"
PLOT_DIR_PATH="$PROJ_PATH/results/figures/$OUT_DIR"
REPORT_DIR_PATH="$PROJ_PATH/results/reports/$OUT_DIR"

# required args check
[[ -z "$AIRCRAFT" ]] && usage 1

# validate arg combinations
if [[ "$DATA_BOOL" -eq 0 && "$PLOT_BOOL" -eq 1 ]]; then
	echo "ERROR: DATA_BOOL must be enabled for PLOT_BOOL to be enabled" >&2
	exit 1
fi

if [[ "$DATA_BOOL" -eq 0 && "$ANALYSIS_BOOL" -eq 1 ]]; then
	echo "ERROR: DATA_BOOL must be enabled for ANALYSIS_BOOL to be enabled" >&2
	exit 1
fi

cd "$PROJ_PATH" || exit 1

"$PROJ_PATH/scripts/lib/create_dirs.sh" \
	"$LOG_DIR_PATH" \
	"$DATA_DIR_PATH" \
	"$PLOT_DIR_PATH" \
	"$REPORT_DIR_PATH"

if [ "$HEADLESS_BOOL" -eq 0 ]; then
	"$PROJ_PATH/scripts/lib/launch.sh" "$AIRCRAFT" &
	FG_PID=$!

	trap 'kill "$FG_PID" 2>/dev/null || true' EXIT
fi

if [ "$BUILD_BOOL" -eq 1 ]; then
	rm -rf "$PROJ_PATH/build"
fi

if [[ "$BUILD_BOOL" -eq 0 && "$HEADLESS_BOOL" -eq 0 ]]; then
	sleep 10
fi

cmake -B "$PROJ_PATH/build" -S "$PROJ_PATH"
cmake --build "$PROJ_PATH/build"

"$PROJ_PATH/build/flight-sim" \
	"$AIRCRAFT" \
	"$DATA_BOOL" \
	"$ANALYSIS_BOOL" \
	"$FAST_BOOL" \
	"$LOG_DIR_PATH" \
	"$DATA_DIR_PATH" \
	"$REPORT_DIR_PATH"

if [ "$PLOT_BOOL" -eq 1 ]; then
	"$PROJ_PATH/scripts/bin/plot.sh" "$OUT_DIR"
fi

if [ "$ANALYSIS_BOOL" -eq 1 ]; then
	"$PROJ_PATH/scripts/bin/analyze.sh" "$OUT_DIR"
fi
