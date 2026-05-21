#!/bin/bash
set -e

source .env

if [ -z "$1" ]; then
  echo "Usage: $0 <DATA_DIR>"
  exit 1
fi

DATA_DIR="$1"
DATA_PATH="$DIR/data/$DATA_DIR"
MATLAB_SCRIPT="$DIR/src/analysis/matlab/init.m"

"$MATLAB" -batch "data_path='$DATA_PATH'; run('$MATLAB_SCRIPT')"