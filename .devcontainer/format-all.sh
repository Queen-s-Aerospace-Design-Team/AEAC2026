#!/bin/bash

set -e

format_dir="ros_ws/src"

echo "Running clang-format on $format_dir..."

# change to repository root (one level up from this script)
cd "$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)/.."

find $format_dir \
  -type f \
  -regex '.*\.\(cpp\|c\|hpp\|h\|cc\|hh\)' \
  -exec clang-format -i {} +

echo "Finished formatting!"
