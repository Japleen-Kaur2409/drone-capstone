#!/usr/bin/env bash
# Start ArduCopter SITL, then MAVProxy in front of it.
set -euo pipefail

HOME_LOC="${HOME_LOC:--35.363261,149.165230,584,353}"   # lat,lon,alt(m),heading
MODEL="${MODEL:-quad}"
SPEEDUP="${SPEEDUP:-1}"
# Extra param files from the mounted ./config dir, comma-separated (e.g. plnd.parm)
PARAMS="${PARAMS:-plnd.parm,sitl_rangefinder.parm}"

DEFAULTS=/ardupilot/Tools/autotest/default_params/copter.parm
IFS=',' read -ra extra <<< "$PARAMS"
for f in "${extra[@]}"; do
    [ -n "$f" ] && DEFAULTS="$DEFAULTS,/config/$f"
done

cd /ardupilot
mkdir -p /sitl && cd /sitl

/ardupilot/build/sitl/bin/arducopter \
    --model "$MODEL" \
    --home "$HOME_LOC" \
    --speedup "$SPEEDUP" \
    --defaults "$DEFAULTS" \
    -w -I0 &
SITL_PID=$!
trap 'kill $SITL_PID 2>/dev/null || true' EXIT

# SITL listens on tcp:5760 (single client, so don't probe it); give it a moment
sleep 3

# --out tcpin: listeners are reachable from the Mac via published ports
exec mavproxy.py \
    --master tcp:127.0.0.1:5760 \
    --out tcpin:0.0.0.0:14550 \
    --out tcpin:0.0.0.0:14551 \
    --state-basedir /sitl \
    "$@"
