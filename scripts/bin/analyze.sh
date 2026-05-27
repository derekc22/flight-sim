#!/bin/bash
set -e

source .env

if [ -z "$1" ]; then
	echo "Usage: $0 <OUT_DIR>"
	exit 1
fi

OUT_DIR="$1"
ANALYZE_CONFIG_PATH="$PROJ_PATH/config/analyze.json"

run_group() {
	group="$1"
	enabled_count="$(jq -r --arg group "$group" '.[$group] // {} | to_entries | map(select(.value != null)) | length' "$ANALYZE_CONFIG_PATH")"

	if [ "$enabled_count" -eq 0 ]; then
		return
	fi

	INIT_SCRIPT_PATH="$PROJ_PATH/scripts/lib/analysis/$group/init.sh"

	# run init.sh
	"$INIT_SCRIPT_PATH" "$OUT_DIR"

	while IFS=$'\t' read -r analysis rel_path; do
		SCRIPT_PATH="$PROJ_PATH/scripts/lib/analysis/$group/$analysis.sh"
		CONFIG_PATH="$PROJ_PATH/config/$rel_path"

		# run analysis script
		"$SCRIPT_PATH" "$OUT_DIR" "$CONFIG_PATH"
	done < <(jq -r --arg group "$group" '.[$group] // {} | to_entries[] | select(.value != null) | [.key, .value] | @tsv' "$ANALYZE_CONFIG_PATH")
}

run_group linear
run_group nonlinear
