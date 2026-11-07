#!/bin/bash
# Fused thermal+visible detector as TWO processes (plus a watchdog supervisor for the first):
#   fusion_supervisor.sh -> fusion_pi.py - cameras + Hailo + fusion, writes frames/status to /dev/shm/team16_fusion;
#                           restarted automatically if it exits or stops producing frames
#   fusion_web.py - web server: fused view :8081, thermal view :8082, calibrate endpoints
# Stop other feeds first (stop_feed.sh, stop_color.sh): only one process can own the Hailo and the Pi camera.
# Usage: start_fusion.sh [--record]   (--record also saves paired frames to ~/team16_thermal/pairs)
cd ~/team16_thermal/thermal
. ../venv/bin/activate
export PYTHONPATH=$HOME/team16_thermal/hailort/hailort/libhailort/bindings/python/platform
alive() { [ -f "$1" ] && kill -0 $(cat "$1") 2>/dev/null; }
if alive ../web.pid; then echo 'web already running'; else
  setsid nohup python3 fusion_web.py > ../web.log 2>&1 < /dev/null &
  echo $! > ../web.pid
fi
if alive ../supervisor.pid; then echo 'fusion already running'; else
  EXTRA=""; [ "$1" = "--record" ] && EXTRA="--record $HOME/team16_thermal/pairs"
  : > ../fusion.log
  setsid nohup ./fusion_supervisor.sh $EXTRA > /dev/null 2>&1 < /dev/null &
  echo $! > ../supervisor.pid
fi
echo "started. Fused: http://$(hostname -I | awk '{print $1}'):8081/   Thermal: :8082"
