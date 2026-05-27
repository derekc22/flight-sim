#!/bin/bash
set -e

source .env

if [ -z "$1" ]; then
	echo "Usage: $0 <OUT_DIR>"
	exit 1
fi

OUT_DIR="$1"

"$PROJ_PATH/scripts/lib/analysis/run_m.sh" "$OUT_DIR" linear load
"$PROJ_PATH/scripts/lib/analysis/run_m.sh" "$OUT_DIR" linear ss2tf
