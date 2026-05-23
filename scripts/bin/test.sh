#!/bin/bash
set -e

usage() {
	exit_code="$1"
	cat >&2 <<EOF
USAGE: $0
  -t <TEST_SUITE>

Available test suites:
  transforms_s3
  transforms_so3
  transforms_se3
EOF
	exit "$exit_code"
}

while getopts "t:h" opt; do
	case "$opt" in
		t) TEST_SUITE="$OPTARG" ;;
		h) usage 0 ;;
		?) usage 1 ;;
	esac
done

# set default for TEST_SUITE flag
: "${TEST_SUITE:=0}"

cmake -S . -B build -DGTest_DIR=/opt/homebrew/lib/cmake/GTest
cmake --build build --target flight_sim_tests

if [ "$TEST_SUITE" = "0" ]; then
    ctest --test-dir build --output-on-failure
else
    ./build/flight_sim_tests --gtest_filter="${TEST_SUITE}.*"
fi
