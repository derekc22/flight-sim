#!/bin/bash
set -e

if [ -z "$1" ]; then
  echo "Usage: $0 <DATA_DIR>"
  exit 1
fi

DATA_DIR="$1"

./scripts/lib/analysis/run_m.sh "$DATA_DIR" init
./scripts/lib/analysis/run_m.sh "$DATA_DIR" ss2tf