#!/bin/bash
# Stop the INS service and wait until it has really exited, so a restart can reuse port 8084.
cd ~/team16_ins
if [ -f ins.pid ]; then
  P=$(cat ins.pid)
  if kill -0 $P 2>/dev/null; then
    kill $P
    for i in $(seq 1 20); do kill -0 $P 2>/dev/null || break; sleep 0.25; done
    kill -0 $P 2>/dev/null && { kill -9 $P; sleep 0.5; }
    echo stopped
  fi
fi
rm -f ins.pid
