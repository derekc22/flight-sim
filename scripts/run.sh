#!/bin/bash
set -e

usage() {
  echo "USAGE: $0 -a <AIRCRAFT> -t <TIME_SEC> [-r <TRIM_BOOL>] [-v <VERBOSE_BOOL>] [-d <DATA_BOOL>] [-f <FOLDER_BOOL>]" >&2
  exit 1
}

while getopts "a:t:f:rvdh" opt; do
  case "$opt" in
    a) AIRCRAFT="$OPTARG" ;;
    t) TIME_SEC="$OPTARG" ;;
    r) TRIM_BOOL=1 ;;
    v) VERBOSE_BOOL=1 ;;
    d) DATA_BOOL=1 ;;
    f) DATA_FOLDER="$OPTARG" ;;
    h) usage ;;
    ?) usage ;;
  esac
done

# set defaults for flags
: "${TRIM_BOOL:=0}"
: "${VERBOSE_BOOL:=0}"
: "${DATA_BOOL:=0}"
: "${DATA_FOLDER:=$(date +"%Y%b%d_%H-%M-%S")}"

# required args check
[[ -z "$AIRCRAFT" || -z "$TIME_SEC" ]] && usage

source .env

cd "$DIR" || exit 1

./scripts/write_in_xml.sh
./scripts/write_out_xml.sh

./scripts/launch.sh "$AIRCRAFT" &
FG_PID=$!

trap 'kill "$FG_PID" 2>/dev/null || true' EXIT

sleep 5

rm -rf build
cmake -B build -S .
cmake --build build

./build/flight-sim "$TIME_SEC" "$TRIM_BOOL" "$VERBOSE_BOOL" "$DATA_BOOL" "$DATA_FOLDER"
