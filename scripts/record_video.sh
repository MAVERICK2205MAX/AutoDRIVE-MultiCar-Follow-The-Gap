#!/usr/bin/env bash
################################################################################
# AutoDRIVE Simulator Video Recorder
# Records smooth MP4 video of the 2-Car race in Track Mode
################################################################################

DURATION=${1:-20}
OUT_FILE=${2:-"$HOME/autodrive_ws/autodrive_2cars_track_race.mp4"}

echo "=== Searching for AutoDRIVE Simulator Window ==="
WIN_LINE=$(DISPLAY=:0 xwininfo -root -tree | grep '"AutoDRIVE Simulator"' | grep "1280x720" | head -n 1)

if [ -z "$WIN_LINE" ]; then
    echo "Error: AutoDRIVE Simulator window not found!"
    exit 1
fi

WIN_ID=$(echo "$WIN_LINE" | awk '{print $1}')
echo "Found AutoDRIVE Simulator window: $WIN_ID"
echo "Recording $DURATION seconds of video to: $OUT_FILE"
echo "Recording in progress... (press Ctrl+C anytime to stop early)"

timeout -s SIGINT "${DURATION}" gst-launch-1.0 -e \
    ximagesrc xid="$WIN_ID" ! \
    videoconvert ! \
    openh264enc bitrate=4000000 ! \
    h264parse ! \
    mp4mux ! \
    filesink location="$OUT_FILE" 2>/dev/null

if [ -f "$OUT_FILE" ] && [ -s "$OUT_FILE" ]; then
    SIZE=$(ls -lh "$OUT_FILE" | awk '{print $5}')
    echo "=========================================================="
    echo " RECORDING COMPLETE!"
    echo " File: $OUT_FILE"
    echo " Size: $SIZE"
    echo " Duration: ${DURATION}s"
    echo "=========================================================="
else
    echo "Recording failed or file empty."
fi
