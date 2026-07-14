#!/bin/bash
set -e

if [ -z "$1" ] || [ -z "$2" ] || [ -z "$3" ] || [ -z "$4" ]; then
	echo "Usage: $0 <LOG_DIR_PATH> <DATA_DIR_PATH> <PLOT_DIR_PATH> <REPORT_DIR_PATH>"
	exit 1
fi

LOG_DIR_PATH="$1"
DATA_DIR_PATH="$2"
PLOT_DIR_PATH="$3"
REPORT_DIR_PATH="$4"

mkdir -p "$LOG_DIR_PATH"
mkdir -p "$DATA_DIR_PATH"
mkdir -p "$PLOT_DIR_PATH"
mkdir -p "$REPORT_DIR_PATH"
