#!/bin/bash
set -e

if [ -z "$1" ]; then
	echo "Usage: $0 <DATA_DIR>"
	exit 1
fi

DATA_DIR="$1"

while IFS=$'\t' read -r key rel_path; do
	if [ "$rel_path" != "null" ]; then
		script_name="${key%_config}"
		config_path="config/$rel_path"
		"./scripts/lib/analysis/${script_name}.sh" "$DATA_DIR" "$config_path"
	fi
done < <(jq -r 'to_entries[] | [.key, .value] | @tsv' config/analyze.json)
