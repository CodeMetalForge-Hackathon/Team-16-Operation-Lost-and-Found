#!/bin/bash
# Keeps fusion_pi.py alive. Restarts it if it exits OR if it stops producing frames.
#
# Why external: when the Hailo infer() call wedges, the stuck process can hold the GIL, so a watchdog
# thread inside it would never run. This script only looks at the age of the frame file the detector
# writes (/dev/shm/team16_fusion/status.json, refreshed ~3x/second).
#
# Started by start_fusion.sh; stopped by stop_fusion.sh. Extra args are passed to fusion_pi.py.
cd ~/team16_thermal/thermal
. ../venv/bin/activate
export PYTHONPATH=$HOME/team16_thermal/hailort/hailort/libhailort/bindings/python/platform
STATUS=/dev/shm/team16_fusion/status.json
GRACE=60   # seconds after launch before frames are required (model load, camera open)
STALE=15   # seconds without a fresh frame before the detector is declared hung
LOG=../fusion.log
P=""

say() { echo "[supervisor $(date '+%F %T')] $*" >> "$LOG"; }
kill_child() {  # polite first (releases Hailo cleanly), then force
  [ -n "$P" ] && kill -0 "$P" 2>/dev/null || return 0
  kill -USR1 "$P" 2>/dev/null; sleep 1       # stack dump into the log for diagnosis
  kill "$P" 2>/dev/null
  for i in $(seq 1 20); do kill -0 "$P" 2>/dev/null || return 0; sleep 0.5; done
  say "child ignored SIGTERM, sending SIGKILL"
  kill -9 "$P" 2>/dev/null; sleep 1
}
trap 'kill_child; exit 0' TERM INT

RESTARTS=0
while true; do
  python3 fusion_pi.py --log ../fusion_detections.jsonl "$@" >> "$LOG" 2>&1 < /dev/null &
  P=$!
  echo $P > ../fusion.pid
  START=$(date +%s)
  say "started fusion_pi.py pid $P (restart #$RESTARTS)"
  while kill -0 "$P" 2>/dev/null; do
    sleep 2
    NOW=$(date +%s)
    [ $((NOW - START)) -lt $GRACE ] && continue
    M=$(stat -c %Y "$STATUS" 2>/dev/null || echo 0)
    if [ $((NOW - M)) -gt $STALE ]; then
      say "WATCHDOG: no new frames for $((NOW - M))s - restarting detector"
      kill_child
      break
    fi
  done
  wait "$P" 2>/dev/null
  RESTARTS=$((RESTARTS + 1))
  say "detector stopped, restarting in 5 s"
  sleep 5
done
