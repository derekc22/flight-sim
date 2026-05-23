#!/bin/bash
set -e

usage() {
	exit_code="$1"
	cat >&2 <<EOF
USAGE: $0
  -a <AIRCRAFT>
  -t <TIME_SEC>
  [-r TRIM_BOOL]
  [-s SENSOR_BOOL]
  [-c CONTROL_BOOL]
  [-e ESTIMATION_BOOL]
  [-w WIND_BOOL]
  [-v VERBOSE_BOOL]
  [-d DATA_BOOL]
  [-p PLOT_BOOL]
  -o <DATA_DIR>
  [-m ANALYSIS_BOOL]
  [-z HEADLESS_BOOL]
  [-q QUICK_BOOL]
EOF
	exit "$exit_code"
}

while getopts "a:t:rscewvdpo:mzqh" opt; do
	case "$opt" in
		a) AIRCRAFT="$OPTARG" ;;
		t) TIME_SEC="$OPTARG" ;;
		r) TRIM_BOOL=1 ;;
		s) SENSOR_BOOL=1 ;;
		c) CONTROL_BOOL=1 ;;
		e) ESTIMATION_BOOL=1 ;;
		w) WIND_BOOL=1 ;;
		v) VERBOSE_BOOL=1 ;;
		d) DATA_BOOL=1 ;;
		p) PLOT_BOOL=1 ;;
		o) DATA_DIR="$OPTARG" ;;
		m) ANALYSIS_BOOL=1 ;;
		z) HEADLESS_BOOL=1 ;;
		q) QUICK_BOOL=1 ;;
			h) usage 0 ;;
			?) usage 1 ;;
	esac
done

# set defaults for flags
: "${TRIM_BOOL:=0}"
: "${SENSOR_BOOL:=0}"
: "${CONTROL_BOOL:=0}"
: "${ESTIMATION_BOOL:=0}"
: "${WIND_BOOL:=0}"
: "${VERBOSE_BOOL:=0}"
: "${DATA_BOOL:=0}"
: "${PLOT_BOOL:=0}"
: "${DATA_DIR:=$(date +"%Y%b%d_%H-%M-%S")}"
: "${ANALYSIS_BOOL:=0}"
: "${HEADLESS_BOOL:=0}"
: "${QUICK_BOOL:=0}"

# required args check
[[ -z "$AIRCRAFT" || -z "$TIME_SEC" ]] && usage 1

# validate arg combinations
if [[ "$DATA_BOOL" -eq 0 && "$PLOT_BOOL" -eq 1 ]]; then
	echo "ERROR: DATA_BOOL must be enabled for PLOT_BOOL to be enabled" >&2
	exit 1
fi

if [[ "$DATA_BOOL" -eq 0 && "$ANALYSIS_BOOL" -eq 1 ]]; then
	echo "ERROR: DATA_BOOL must be enabled for ANALYSIS_BOOL to be enabled" >&2
	exit 1
fi

if [ "$HEADLESS_BOOL" -eq 0 ]; then
	source .env

	cd "$DIR" || exit 1

	./scripts/lib/write_in_xml.sh
	./scripts/lib/write_out_xml.sh

	./scripts/lib/launch.sh "$AIRCRAFT" &
	FG_PID=$!

	trap 'kill "$FG_PID" 2>/dev/null || true' EXIT
fi

if [ "$QUICK_BOOL" -eq 0 ]; then
	rm -rf build
fi

if [[ "$QUICK_BOOL" -eq 1 && "$HEADLESS_BOOL" -eq 0 ]]; then
	sleep 10
fi

cmake -B build -S .
cmake --build build

./build/flight-sim \
	"$AIRCRAFT" \
	"$TIME_SEC" \
	"$TRIM_BOOL" \
	"$SENSOR_BOOL" \
	"$CONTROL_BOOL" \
	"$ESTIMATION_BOOL" \
	"$WIND_BOOL" \
	"$VERBOSE_BOOL" \
	"$DATA_BOOL" \
	"$DATA_DIR" \
	"$ANALYSIS_BOOL" \

if [ "$DATA_BOOL" -eq 1 ]; then
	./scripts/lib/dump_args.sh \
		AIRCRAFT="$AIRCRAFT" \
		TIME_SEC="$TIME_SEC" \
		TRIM_BOOL="$TRIM_BOOL" \
		SENSOR_BOOL="$SENSOR_BOOL" \
		CONTROL_BOOL="$CONTROL_BOOL" \
		ESTIMATION_BOOL="$ESTIMATION_BOOL" \
		WIND_BOOL="$WIND_BOOL" \
		VERBOSE_BOOL="$VERBOSE_BOOL" \
		DATA_BOOL="$DATA_BOOL" \
		PLOT_BOOL="$PLOT_BOOL" \
		DATA_DIR="$DATA_DIR" \
		ANALYSIS_BOOL="$ANALYSIS_BOOL" \
		HEADLESS_BOOL="$HEADLESS_BOOL" \
		QUICK_BOOL="$QUICK_BOOL"
fi

if [ "$PLOT_BOOL" -eq 1 ]; then
	./scripts/bin/plot.sh "$DATA_DIR"
fi

if [ "$ANALYSIS_BOOL" -eq 1 ]; then
	./scripts/lib/analysis/init.sh "$DATA_DIR"
fi
