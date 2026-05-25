#!/bin/bash
set -e

source .env

SRC_PATH="${PROJ_PATH}/schema/udp-in.xml"
DEST_PATH="${PRTCL_PATH}/udp-in.xml"

cp "$SRC_PATH" "$DEST_PATH"
