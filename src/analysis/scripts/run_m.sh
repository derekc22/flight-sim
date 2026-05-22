#!/bin/bash
set -e

source .env

if [ -z "$1" ]; then
  echo "Usage: $0 <DATA_DIR>"
  exit 1
fi

DATA_DIR="$1"
MATLAB_SCRIPT="$2"

DATA_DIR_PATH="$DIR/data/$DATA_DIR"
DATA_MAT_PATH="$DIR/data/$DATA_DIR/data.mat"
PLOT_DIR_PATH="$DIR/figures/$DATA_DIR"

"$MATLAB" -batch \
"data_dir_path='$DATA_DIR_PATH'; \
data_mat_path='$DATA_MAT_PATH'; \
plot_dir_path='$PLOT_DIR_PATH'; \
run('src/analysis/matlab/${MATLAB_SCRIPT}_.m')"