#!/bin/bash
set -e

if [ -z "$1" ] || [ -z "$2" ] || [ -z "$3" ]; then
	echo "Usage: $0 <DATA_DIR_PATH> <PLOT_DIR_PATH> <REPORT_DIR_PATH>"
	exit 1
fi

DATA_DIR_PATH="$1"
PLOT_DIR_PATH="$2"
REPORT_DIR_PATH="$3"

mkdir -p "$DATA_DIR_PATH"
mkdir -p "$PLOT_DIR_PATH"
mkdir -p "$REPORT_DIR_PATH"
