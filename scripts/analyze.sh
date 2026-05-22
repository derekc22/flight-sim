#!/bin/bash
set -e

if [ -z "$1" ]; then
  echo "Usage: $0 <DATA_DIR>"
  exit 1
fi

DATA_DIR="$1"

./src/analysis/scripts/run_m.sh "$DATA_DIR" init
./src/analysis/scripts/run_m.sh "$DATA_DIR" ss2tf
./src/analysis/scripts/run_m.sh "$DATA_DIR" time_response