#!/bin/bash
set -e

AIRCRAFT="${1:?usage: ./scripts/run.sh <AIRCRAFT> <TIME>}"
TIME="${2:?usage: ./scripts/run.sh <AIRCRAFT> <TIME>}"

source .env

cd "$DIR" || exit 1

source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate ame532

./scripts/write_in_xml.sh
./scripts/write_out_xml.sh

./scripts/launch.sh "$AIRCRAFT" &
FG_PID=$!

trap 'kill "$FG_PID" 2>/dev/null || true' EXIT

sleep 5

rm -rf build
cmake -B build -S . -DCMAKE_PREFIX_PATH="$CONDA_PREFIX"
cmake --build build

./build/flight-sim "$TIME"
