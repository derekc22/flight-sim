#!/bin/bash
set -e

rm -rf build
cmake -B build -S .
cmake --build build

./build/flight-sim