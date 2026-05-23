#!/bin/bash
set -e

source .env

SRC_OUT="${SRC}/udp-out.xml"
DEST_OUT="${DEST}/udp-out.xml"

cp "$SRC_OUT" "$DEST_OUT"