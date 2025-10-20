#!/usr/bin/env bash
set -euo pipefail

# Optional: if ArduPilot's prereq script ever drops PATH tweaks into ~/.profile
# this makes the script resilient. Harmless if the file doesn't exist.
. ~/.profile 2>/dev/null || true

VEHICLE="${VEHICLE:-ArduCopter}"
FRAME="${FRAME:-quad}"
SPEEDUP="${SIM_SPEEDUP:-1}"
OUT="${OUT_TARGET:-udp:host.docker.internal:14550}"  # QGC on host Mac

cd /home/qadt/ardupilot

# sim_vehicle.py handles build+run+logs; --out sends MAVLink to QGC
exec sim_vehicle.py -v "$VEHICLE" -f "$FRAME" --speedup "$SPEEDUP" --out="$OUT" ${SIM_EXTRA_ARGS:-}