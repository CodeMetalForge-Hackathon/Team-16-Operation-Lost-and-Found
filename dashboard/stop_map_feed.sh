#!/bin/bash
cd ~/team16_dashboard
if [ -f map_feed.pid ]; then
  LOOP=$(cat map_feed.pid)
  CHILD=$(pgrep -P "$LOOP" 2>/dev/null)
  kill "$LOOP" 2>/dev/null          # the loop first, so it does not restart the feed
  [ -n "$CHILD" ] && kill $CHILD 2>/dev/null
  echo stopped
fi
rm -f map_feed.pid
