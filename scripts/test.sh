#!/usr/bin/env bash
set -e

cmake -S . -B build -DGTest_DIR=/opt/homebrew/lib/cmake/GTest
cmake --build build
ctest --test-dir build --output-on-failure