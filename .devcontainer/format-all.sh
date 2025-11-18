#!/bin/bash

set -e

echo "Running clang-format on entire repository..."

find .. \
  -type f \
  -regex '.*\.\(cpp\|c\|hpp\|h\|cc\|hh\)' \
  -exec clang-format -i {} +

echo "Finished formatting!"
