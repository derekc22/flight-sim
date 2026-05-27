#!/bin/bash
set -e

source .env

OUT_DIR=""

for arg in "$@"; do
	if [[ "$arg" != *=* ]]; then
		echo "ERROR: expected KEY=VALUE argument, got '$arg'" >&2
		exit 1
	fi

	if [[ "${arg%%=*}" == "OUT_DIR" ]]; then
		OUT_DIR="${arg#*=}"
	fi
done

if [[ -z "$OUT_DIR" ]]; then
	echo "ERROR: OUT_DIR is required" >&2
	exit 1
fi

REPORT_DIR_PATH="$PROJ_PATH/results/reports/$OUT_DIR"
ARGS_PATH="$REPORT_DIR_PATH/args.txt"
printf "%s\n" "$@" > "$ARGS_PATH"

echo "File saved successfully to $ARGS_PATH"
