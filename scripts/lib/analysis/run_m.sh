#!/bin/bash
set -e

source .env

if [ -z "$1" ] || [ -z "$2" ]; then
    echo "Usage: $0 <DATA_DIR> <MATLAB_SCRIPT> [CONFIG_PATH]"
    exit 1
fi

DATA_DIR="$1"
MATLAB_SCRIPT="$2"
CONFIG_PATH="$3"

DATA_DIR_PATH="$DIR/data/$DATA_DIR"
DATA_MAT_PATH="$DIR/data/$DATA_DIR/data.mat"
PLOT_DIR_PATH="$DIR/figures/$DATA_DIR"

"$MATLAB" -batch \
"DATA_DIR_PATH='$DATA_DIR_PATH'; \
DATA_MAT_PATH='$DATA_MAT_PATH'; \
PLOT_DIR_PATH='$PLOT_DIR_PATH'; \
CONFIG_PATH='$CONFIG_PATH'; \
run('scripts/lib/analysis/matlab/${MATLAB_SCRIPT}_.m')"
