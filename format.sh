#!/bin/bash
set -e

PROJ_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$PROJ_PATH"

find include src tests -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.tpp' \) -print0 | xargs -0 clang-format -i
