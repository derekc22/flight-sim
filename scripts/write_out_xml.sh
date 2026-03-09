#!/bin/bash
set -e

SRC="/Users/derekchibuzor/Documents/VSCodeProjects/flight-sim/config/udp-out.xml"
DEST="/Users/derekchibuzor/Library/Application Support/FlightGear/fgdata_2024_1/Protocol/udp-out.xml"

cp "$SRC" "$DEST"