#!/bin/bash
# INS coast service for the dashboard (port 8084). Receive-only. Usage: start_ins.sh [--serial /dev/ttyUSB0 | --sim | --replay FILE]
cd ~/team16_ins
if [ -f ins.pid ] && kill -0 $(cat ins.pid) 2>/dev/null; then echo 'INS already running'; exit 0; fi
setsid nohup python3 dr_run.py --log ins.jsonl "$@" > ins.log 2>&1 < /dev/null &
echo $! > ins.pid
echo "INS service started (pid $(cat ins.pid)) on :8084  args: $*"
