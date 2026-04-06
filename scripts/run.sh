#!/bin/bash
set -e

usage() {
  echo "USAGE: $0 -a <AIRCRAFT> -t <TIME_SEC> [-r <TRIM_BOOL>] [-s <SENSOR_BOOL>] [-c <CONTROL_BOOL>] [-v <VERBOSE_BOOL>] [-d <DATA_BOOL>] [-o <OUT_DIR>] [-p <PLOT>]" >&2
  exit 1
}

while getopts "a:t:o:rscvdph" opt; do
  case "$opt" in
    a) AIRCRAFT="$OPTARG" ;;
    t) TIME_SEC="$OPTARG" ;;
    r) TRIM_BOOL=1 ;;
    s) SENSOR_BOOL=1 ;;
    c) CONTROL_BOOL=1 ;;
    v) VERBOSE_BOOL=1 ;;
    d) DATA_BOOL=1 ;;
    o) OUT_DIR="$OPTARG" ;;
    p) PLOT=1 ;;
    h) usage ;;
    ?) usage ;;
  esac
done

# set defaults for flags
: "${TRIM_BOOL:=0}"
: "${CONTROL_BOOL:=0}"
: "${SENSOR_BOOL:=0}"
: "${VERBOSE_BOOL:=0}"
: "${DATA_BOOL:=0}"
: "${PLOT:=0}"
: "${OUT_DIR:=$(date +"%Y%b%d_%H-%M-%S")}"

# required args check
[[ -z "$AIRCRAFT" || -z "$TIME_SEC" ]] && usage
if [[ "$TRIM_BOOL" -eq 1 && "$CONTROL_BOOL" -eq 1 ]]; then
  echo "ERROR: TRIM_BOOL and CONTROL_BOOL cannot both be enabled" >&2
  exit 1
fi

source .env

cd "$DIR" || exit 1

./scripts/write_in_xml.sh
./scripts/write_out_xml.sh

./scripts/launch.sh "$AIRCRAFT" &
FG_PID=$!

trap 'kill "$FG_PID" 2>/dev/null || true' EXIT

sleep 0

rm -rf build
cmake -B build -S .
cmake --build build

./build/flight-sim "$TIME_SEC" "$TRIM_BOOL" "$SENSOR_BOOL" "$CONTROL_BOOL" "$VERBOSE_BOOL" "$DATA_BOOL" "$OUT_DIR"

if [ "$PLOT" -eq 1 ]; then
  ./scripts/plot.sh "$OUT_DIR"
fi
