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
CONFIG_PATH="$PROJ_PATH/config/analyze.json"
DATA_DIR_PATH="$PROJ_PATH/results/$OUT_DIR/data"
PLOT_DIR_PATH="$PROJ_PATH/results/$OUT_DIR/figures/analysis"
REPORT_DIR_PATH="$PROJ_PATH/results/$OUT_DIR/reports/analysis"

cd "$PROJ_PATH" || exit 1
python -m analysis.analyze \
	"$CONFIG_PATH" \
	"$DATA_DIR_PATH" \
	"$PLOT_DIR_PATH" \
	"$REPORT_DIR_PATH"
