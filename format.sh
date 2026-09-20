#!/bin/bash
set -e

source .env

cd "$PROJ_PATH"

find include src tests -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.tpp' \) -print0 | xargs -0 clang-format -i
