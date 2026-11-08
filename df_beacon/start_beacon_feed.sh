#!/bin/bash
# Beacon telemetry feed for the dashboard (port 8083). Receive-only.
# Usage: start_beacon_feed.sh [--serial /dev/ttyUSB0] [--sdr-file PATH] [--demo]
# With no arguments it just serves "no signal yet" until a source posts to /ingest.
cd ~/team16_beacon
if [ -f feed.pid ] && kill -0 $(cat feed.pid) 2>/dev/null; then echo 'beacon feed already running'; exit 0; fi
setsid nohup python3 beacon_feed.py "$@" > feed.log 2>&1 < /dev/null &
echo $! > feed.pid
echo "beacon feed started (pid $(cat feed.pid)) on :8083  args: $*"
