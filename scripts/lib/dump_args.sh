#!/bin/bash
set -e

source .env

DATA_DIR=""

for arg in "$@"; do
	if [[ "$arg" != *=* ]]; then
		echo "ERROR: expected KEY=VALUE argument, got '$arg'" >&2
		exit 1
	fi

	if [[ "${arg%%=*}" == "DATA_DIR" ]]; then
		DATA_DIR="${arg#*=}"
	fi
done

if [[ -z "$DATA_DIR" ]]; then
	echo "ERROR: DATA_DIR is required" >&2
	exit 1
fi

REPORT_DIR_PATH="$PROJ_PATH/results/reports/$DATA_DIR"
ARGS_PATH="$REPORT_DIR_PATH/args.txt"
printf "%s\n" "$@" > "$ARGS_PATH"

echo "File saved successfully to $ARGS_PATH"
