#!/bin/bash
cd ~/team16_dashboard
[ -f pi_stats.pid ] && kill $(cat pi_stats.pid) 2>/dev/null && echo stopped
rm -f pi_stats.pid
