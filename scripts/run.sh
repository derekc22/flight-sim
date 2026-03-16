#!/bin/bash
set -e

source .env

cd "$DIR" || exit 1

./scripts/write_in_xml.sh
./scripts/write_out_xml.sh

./scripts/launch.sh &
FG_PID=$!

trap 'kill "$FG_PID" 2>/dev/null || true' EXIT

sleep 10

rm -rf build
cmake -B build -S . -DCMAKE_PREFIX_PATH="$CONDA_PREFIX"
cmake --build build

./build/flight-sim