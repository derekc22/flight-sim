#!/bin/bash
set -e

source .env

find \
	"$HOME/Library/Application Support/FlightGear" \
	-type f -name '*-set.xml' -print 2>/dev/null \
	| sed 's#.*/##; s/-set\.xml$//' \
	| sort -u