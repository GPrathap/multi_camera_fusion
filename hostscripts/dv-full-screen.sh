#!/bin/bash
echo "Running chromium browser"
chromium-browser --app=http://localhost:8888/ --start-fullscreen 2>&1 || { echo >&2 "I require Chromium browser, but it's not installed.  Aborting."; exit 1; } &
