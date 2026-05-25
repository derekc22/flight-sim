#!/bin/bash
set -e

source .env

if [ -z "$1" ] || [ -z "$2" ] || [ -z "$3" ]; then
    echo "Usage: $0 <DATA_DIR> <ANALYSIS_TYPE> <MATLAB_SCRIPT> [CONFIG_PATH]"
    exit 1
fi

DATA_DIR="$1"
ANALYSIS_TYPE="$2"
MATLAB_SCRIPT="$3"
CONFIG_PATH="$4"

DATA_DIR_PATH="$PROJ_PATH/results/data/$DATA_DIR"
DATA_MAT_PATH="$PROJ_PATH/results/data/$DATA_DIR/data.mat"
PLOT_DIR_PATH="$PROJ_PATH/results/figures/$DATA_DIR"
REPORT_DIR_PATH="$PROJ_PATH/results/reports/$DATA_DIR"
MATLAB_SCRIPT_DIR_PATH="$PROJ_PATH/scripts/lib/analysis/matlab/$ANALYSIS_TYPE"

"$MATLAB_PATH" -batch \
"DATA_DIR_PATH='$DATA_DIR_PATH'; \
DATA_MAT_PATH='$DATA_MAT_PATH'; \
PLOT_DIR_PATH='$PLOT_DIR_PATH'; \
REPORT_DIR_PATH='$REPORT_DIR_PATH'; \
CONFIG_PATH='$CONFIG_PATH'; \
run('$MATLAB_SCRIPT_DIR_PATH/${MATLAB_SCRIPT}_.m')"
