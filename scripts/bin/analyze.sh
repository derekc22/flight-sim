#!/bin/bash
set -e

source .env

CONDA_BASE_PATH=$(conda info --base)
source "$CONDA_BASE_PATH/etc/profile.d/conda.sh"
conda activate ame532

if [ -z "$1" ]; then
	echo "Usage: $0 <OUT_DIR>"
	exit 1
fi

OUT_DIR="$1"

cd "$PROJ_PATH" || exit 1
python -m analysis.analyze "$PROJ_PATH" "$OUT_DIR"
