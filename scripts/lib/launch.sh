#!/bin/bash
set -e

source .env

AIRCRAFT="${1:?usage: $0 <AIRCRAFT>}"

exec "$FG_PATH" \
--ignore-autosave \
--generic=socket,out,100,127.0.0.1,5510,udp,udp-out \
--generic=socket,in,100,127.0.0.1,5511,udp,udp-in \
--fdm=external \
--timeofday=noon \
--graphics-preset=minimal-quality \
--aircraft="$AIRCRAFT" \
--prop:/sim/current-view/view-number-raw=1 \
--disable-real-weather-fetch \
--geometry=800x700 \
--prop:/engines/engine[0]/rpm=2300 \
--httpd=8080 \
--data="$PROJ_PATH"/schema