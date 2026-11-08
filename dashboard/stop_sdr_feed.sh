#!/bin/bash
cd ~/team16_dashboard
[ -f sdr_feed.pid ] && kill $(cat sdr_feed.pid) 2>/dev/null && echo stopped
rm -f sdr_feed.pid
