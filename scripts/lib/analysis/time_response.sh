#!/bin/bash
set -e

if [ -z "$1" ] || [ -z "$2" ]; then
	echo "Usage: $0 <DATA_DIR> <CONFIG_PATH>"
	exit 1
fi

DATA_DIR="$1"
CONFIG_PATH="$2"

./scripts/lib/analysis/run_m.sh "$DATA_DIR" time_response "$CONFIG_PATH"