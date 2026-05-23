#!/bin/bash
set -e

source .env

SRC_IN="${SRC}/udp-in.xml"
DEST_IN="${DEST}/udp-in.xml"

cp "$SRC_IN" "$DEST_IN"