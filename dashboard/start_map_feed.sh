#!/bin/bash
# Lidar map feed for the dashboard (port 8086). The lidar laptop POSTs its map and pose here.
# Runs in a small loop so it comes straight back if it ever exits. State (map + trail) is kept in map_feed_state.json.
cd ~/team16_dashboard
if [ -f map_feed.pid ] && kill -0 $(cat map_feed.pid) 2>/dev/null; then echo 'map feed already running'; exit 0; fi
setsid nohup bash -c 'while true; do python3 map_feed.py >> map_feed.log 2>&1; sleep 2; done' > /dev/null 2>&1 < /dev/null &
echo $! > map_feed.pid
echo "map feed started (pid $(cat map_feed.pid)) on :8086"
