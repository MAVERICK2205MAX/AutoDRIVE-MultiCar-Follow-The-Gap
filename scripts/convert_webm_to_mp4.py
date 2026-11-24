#!/usr/bin/env python3
"""Convert any recorded .webm from ~/Videos/Screencasts/ to .mp4 using OpenCV."""
import os
import glob
import sys
import cv2

screencast_dir = os.path.expanduser('~/Videos/Screencasts')
out_dir = os.path.expanduser('~/Videos')

webms = sorted(glob.glob(os.path.join(screencast_dir, '*.webm')), key=os.path.getmtime)
if not webms:
    print(f"No .webm files found in {screencast_dir}")
    sys.exit(1)

latest_webm = webms[-1]
out_mp4 = os.path.join(out_dir, 'tutorial5_recording.mp4')
print(f"Converting latest screencast: {latest_webm} -> {out_mp4}...")

cap = cv2.VideoCapture(latest_webm)
if not cap.isOpened():
    print(f"Error opening video: {latest_webm}")
    sys.exit(1)

w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(out_mp4, fourcc, fps, (w, h))

count = 0
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    out.write(frame)
    count += 1
    if count % 100 == 0:
        print(f"Progress: {count}/{total_frames} frames ({count/total_frames*100:.1f}%)")

cap.release()
out.release()
print(f"Done! Converted video saved to: {out_mp4} ({os.path.getsize(out_mp4)/(1024*1024):.2f} MB)")
