#!/bin/bash
set -e

source .env

if [ -z "$1" ] || [ -z "$2" ]; then
	echo "Usage: $0 <OUT_DIR> <CONFIG_PATH>"
	exit 1
fi

OUT_DIR="$1"
CONFIG_PATH="$2"

"$PROJ_PATH/scripts/lib/analysis/run_m.sh" "$OUT_DIR" linear frequency_domain "$CONFIG_PATH"
