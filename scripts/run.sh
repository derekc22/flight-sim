#!/bin/bash
set -e

usage() {
	echo "USAGE: $0 -a <AIRCRAFT> -t <TIME_SEC> [-r <TRIM_BOOL>] [-s <SENSOR_BOOL>] [-c <CONTROL_BOOL>] [-e <ESTIMATION_BOOL>] [-v <VERBOSE_BOOL>] [-d <DATA_BOOL>] [-o <OUT_DIR>] [-p <PLOT_BOOL>] [-z <TEST_BOOL>] [-q <QUICK_BOOL>]" >&2
	exit 1
}

while getopts "a:t:o:rscevdpzqh" opt; do
	case "$opt" in
		a) AIRCRAFT="$OPTARG" ;;
		t) TIME_SEC="$OPTARG" ;;
		r) TRIM_BOOL=1 ;;
		s) SENSOR_BOOL=1 ;;
		c) CONTROL_BOOL=1 ;;
		e) ESTIMATION_BOOL=1 ;;
		v) VERBOSE_BOOL=1 ;;
		d) DATA_BOOL=1 ;;
		o) OUT_DIR="$OPTARG" ;;
		p) PLOT_BOOL=1 ;;
		z) TEST_BOOL=1 ;;
		q) QUICK_BOOL=1 ;;
		h) usage ;;
		?) usage ;;
	esac
done

# set defaults for flags
: "${TRIM_BOOL:=0}"
: "${SENSOR_BOOL:=0}"
: "${CONTROL_BOOL:=0}"
: "${ESTIMATION_BOOL:=0}"
: "${VERBOSE_BOOL:=0}"
: "${DATA_BOOL:=0}"
: "${PLOT_BOOL:=0}"
: "${TEST_BOOL:=0}"
: "${QUICK_BOOL:=0}"
: "${OUT_DIR:=$(date +"%Y%b%d_%H-%M-%S")}"

# required args check
[[ -z "$AIRCRAFT" || -z "$TIME_SEC" ]] && usage

# validate arg combinations
if [[ "$DATA_BOOL" -eq 0 && "$PLOT_BOOL" -eq 1 ]]; then
	echo "ERROR: DATA_BOOL must be enabled for PLOT_BOOL to be enabled" >&2
	exit 1
fi

if [ "$TEST_BOOL" -eq 0 ]; then
	source .env

	cd "$DIR" || exit 1

	./scripts/write_in_xml.sh
	./scripts/write_out_xml.sh

	./scripts/launch.sh "$AIRCRAFT" &
	FG_PID=$!

	trap 'kill "$FG_PID" 2>/dev/null || true' EXIT
fi

if [ "$QUICK_BOOL" -eq 0 ]; then
	rm -rf build
fi

if [[ "$QUICK_BOOL" -eq 1 && "$TEST_BOOL" -eq 0 ]]; then
	sleep 10
fi

cmake -B build -S .
cmake --build build

./build/flight-sim \
	"$TIME_SEC" \
	"$TRIM_BOOL" \
	"$SENSOR_BOOL" \
	"$CONTROL_BOOL" \
	"$ESTIMATION_BOOL" \
	"$VERBOSE_BOOL" \
	"$DATA_BOOL" \
	"$OUT_DIR"

if [ "$DATA_BOOL" -eq 1 ]; then
	./scripts/dump_args.sh \
		"$OUT_DIR" \
		"$AIRCRAFT" \
		"$TIME_SEC" \
		"$TRIM_BOOL" \
		"$SENSOR_BOOL" \
		"$CONTROL_BOOL" \
		"$ESTIMATION_BOOL" \
		"$VERBOSE_BOOL" \
		"$DATA_BOOL" \
		"$PLOT_BOOL" \
		"$TEST_BOOL" \
		"$QUICK_BOOL"
fi

if [ "$PLOT_BOOL" -eq 1 ]; then
	./scripts/plot.sh "$OUT_DIR"
fi
