#!/bin/bash
set -e

source .env

if [ -z "$1" ]; then
	echo "Usage: $0 <DATA_DIR>"
	exit 1
fi

DATA_DIR="$1"

"$PROJ_PATH/scripts/lib/analysis/run_m.sh" "$DATA_DIR" linear load
"$PROJ_PATH/scripts/lib/analysis/run_m.sh" "$DATA_DIR" linear ss2tf
