#!/bin/bash
# SDR / phone-detect bridge for the dashboard (port 8085). Read-only: it only connects to the rf hub's localhost ports.
# Usage: start_sdr_feed.sh [--demo]
cd ~/team16_dashboard
if [ -f sdr_feed.pid ] && kill -0 $(cat sdr_feed.pid) 2>/dev/null; then echo 'sdr feed already running'; exit 0; fi
setsid nohup python3 sdr_feed.py "$@" > sdr_feed.log 2>&1 < /dev/null &
echo $! > sdr_feed.pid
echo "sdr feed started (pid $(cat sdr_feed.pid)) on :8085  args: $*"
