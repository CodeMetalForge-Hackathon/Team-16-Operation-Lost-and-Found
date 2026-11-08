#!/bin/bash
cd ~/team16_beacon
[ -f feed.pid ] && kill $(cat feed.pid) 2>/dev/null && echo stopped
rm -f feed.pid
