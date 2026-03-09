#!/bin/bash
set -e

FG="/Applications/FlightGear.app/Contents/MacOS/FlightGear"

"$FG" \
--ignore-autosave \
--generic=socket,out,100,127.0.0.1,5510,udp,udp-out \
--generic=socket,in,100,127.0.0.1,5511,udp,udp-in \
--fdm=external \
--timeofday=noon \
--prop:/sim/time/warp=43200 \
--graphics-preset=minimal-quality \
--aircraft=ASW28 \
--prop:/sim/current-view/view-number-raw=1 \
--disable-real-weather-fetch
# --prop:/devices/status/mice/mouse[0]/mode=2 \