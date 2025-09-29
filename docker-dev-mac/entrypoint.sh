#!/bin/bash

# Exits cleanly if something fails
set -e

# To execute command you want
echo "==> Container ready"
exec "$@"
