#!/bin/bash
set -e

source .env

SRC_PATH="${PROJ_PATH}/schema/udp-out.xml"
DEST_PATH="${PRTCL_PATH}/udp-out.xml"

cp "$SRC_PATH" "$DEST_PATH"
