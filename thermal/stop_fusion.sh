#!/bin/bash
# Stop only the processes start_fusion.sh started. Waits for fusion_pi.py to release the Hailo and
# cameras (restarting too soon after a kill can leave the next process hanging).
stop() {  # $1 = pidfile, $2 = wait loops (0.5 s each)
  if [ -f "$1" ] && kill -0 $(cat "$1") 2>/dev/null; then
    P=$(cat "$1"); kill $P
    for i in $(seq 1 $2); do kill -0 $P 2>/dev/null || break; sleep 0.5; done
    kill -0 $P 2>/dev/null && { echo "forcing $1"; kill -9 $P; sleep 1; }
    echo "stopped $(basename $1 .pid)"
  fi
  rm -f "$1"
}
stop ~/team16_thermal/fusion.pid 20
stop ~/team16_thermal/web.pid 6
sleep 3
