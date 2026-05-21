#!/bin/bash
set -e

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

OUT_PATH="data/$DATA_DIR/args.txt"
mkdir -p "$(dirname "$OUT_PATH")"

printf "%s\n" "$@" > "$OUT_PATH"

echo "File saved successfully to $OUT_PATH"
