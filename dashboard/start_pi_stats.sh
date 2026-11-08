#!/bin/bash
# Pi health feed for the dashboard (port 8087). Read-only.
cd ~/team16_dashboard
if [ -f pi_stats.pid ] && kill -0 $(cat pi_stats.pid) 2>/dev/null; then echo 'pi stats already running'; exit 0; fi
setsid nohup python3 pi_stats.py "$@" > pi_stats.log 2>&1 < /dev/null &
echo $! > pi_stats.pid
echo "pi stats started (pid $(cat pi_stats.pid)) on :8087"
