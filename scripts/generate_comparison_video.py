#!/usr/bin/env python3
import csv
import os
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image

video_path = '/home/admin1/autodrive_ws/path_planning_comparison.mp4'
width, height = 1280, 720
fps = 30
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(video_path, fourcc, fps, (width, height))

# Load map
yaml_path = '/home/admin1/autodrive_ws/maps/F1tenth_Map_walled.yaml'
with open(yaml_path) as f:
    meta = yaml.safe_load(f)
img_map = np.array(Image.open('/home/admin1/autodrive_ws/maps/' + meta['image']).convert('L'))
res = meta['resolution']
origin = meta['origin']
h, w = img_map.shape
grid = np.flipud(img_map)
ext = [origin[0], origin[0] + w * res, origin[1], origin[1] + h * res]

# Load waypoints
raw_pts = []
with open('/home/admin1/autodrive_ws/waypoints/raw_path.csv') as f:
    for r in csv.DictReader(f):
        raw_pts.append([float(r['x']), float(r['y'])])
raw_pts = np.array(raw_pts)

s_pts = []
with open('/home/admin1/autodrive_ws/waypoints/smoothed_path.csv') as f:
    for r in csv.DictReader(f):
        s_pts.append([float(r['x']), float(r['y'])])
s_pts = np.array(s_pts)

def fig_to_frame(fig):
    fig.canvas.draw()
    buf = np.asarray(fig.canvas.buffer_rgba())
    plt.close(fig)
    bgr = cv2.cvtColor(buf, cv2.COLOR_RGBA2BGR)
    return cv2.resize(bgr, (width, height))

print('1. Rendering Title Slide...')
fig, ax = plt.subplots(figsize=(16, 9), facecolor='#11111b')
ax.axis('off')
ax.text(0.5, 0.68, 'F1TENTH Global Path Planning', color='#89b4fa', fontsize=32, fontweight='bold', ha='center')
ax.text(0.5, 0.54, 'Tutorial 5: Adapting the Base Repository (AutoDRIVE)', color='#cdd6f4', fontsize=20, ha='center')
ax.text(0.5, 0.42, 'Raw Grid Planner (A*) vs Shape-Preserving Spline (PCHIP)', color='#fab387', fontsize=18, ha='center')
ax.text(0.5, 0.28, 'Metrics • Obstacle Clearance • Continuous Trajectory Comparison', color='#a6e3a1', fontsize=15, ha='center')
frame_title = fig_to_frame(fig)
for _ in range(150): # 5s
    out.write(frame_title)

print('2. Rendering Map & Virtual Wall Setup...')
fig, ax = plt.subplots(figsize=(16, 9), facecolor='#181825')
ax.set_facecolor('#181825')
ax.imshow(grid, cmap='gray', origin='lower', extent=ext)
ax.plot([0.0, 1.5], [4.05, 4.05], 'r-', lw=6, label='Virtual Wall (Y=4.05m)')
ax.plot(0.792, 3.535, 'go', ms=12, label='Start (0.792, 3.535)')
ax.plot(0.600, 4.701, 'm*', ms=16, label='Goal (0.600, 4.701)')
ax.set_title('Phase 2: Map & Virtual Wall (Prevents 1m Short Hop -> Forces Full 26.2m Lap)', color='white', fontsize=16, pad=12, fontweight='bold')
ax.set_xlabel('X [meters]', color='white')
ax.set_ylabel('Y [meters]', color='white')
ax.tick_params(colors='white')
ax.legend(loc='lower left', fontsize=12, facecolor='#313244', labelcolor='white')
ax.grid(True, alpha=0.25)
frame_map = fig_to_frame(fig)
for _ in range(240): # 8s
    out.write(frame_map)

print('3. Rendering Quantitative Metrics Table...')
fig, ax = plt.subplots(figsize=(16, 9), facecolor='#11111b')
ax.axis('off')
ax.text(0.5, 0.88, 'Phase 3: Quantitative Benchmark & Safety Validation', color='#89b4fa', fontsize=24, fontweight='bold', ha='center')
table_data = [
    ['Metric', 'Raw Path (A*)', 'Smoothed Path (PCHIP)', 'Safety Status'],
    ['Algorithm', 'A* Search (Grid 52x154)', 'PCHIP Spline (Hermite)', 'Optimal & Continuous'],
    ['Trajectory Length', '26.49 meters', '26.24 meters', 'Full Lap Complete'],
    ['Waypoint Count', '269 points', '263 points (dt = 0.10m)', 'Uniformly Resampled'],
    ['Min Obstacle Clearance', '0.283 meters', '0.285 meters', 'Exceeds 0.20m Radius'],
    ['Violations (< 0.20m)', '0 / 269 points', '0 / 263 points', '100% Collision-Free'],
    ['Curvature / Steering', 'Discontinuous (45/90 deg)', 'C1 Continuous (Smooth)', 'Physically Drivable']
]
table = ax.table(cellText=table_data, loc='center', cellLoc='center', colWidths=[0.24, 0.25, 0.27, 0.24])
table.auto_set_font_size(False)
table.set_fontsize(13)
table.scale(1.0, 2.5)
for (r, c), cell in table.get_celld().items():
    if r == 0:
        cell.set_facecolor('#313244')
        cell.get_text().set_color('#fab387')
        cell.get_text().set_fontweight('bold')
    else:
        cell.set_facecolor('#1e1e2e' if r % 2 == 0 else '#181825')
        cell.get_text().set_color('#cdd6f4')
frame_metrics = fig_to_frame(fig)
for _ in range(270): # 9s
    out.write(frame_metrics)

print('4. Rendering Full Track Overlay Comparison...')
fig, ax = plt.subplots(figsize=(16, 9), facecolor='#181825')
ax.set_facecolor('#181825')
ax.imshow(grid, cmap='gray', origin='lower', extent=ext)
ax.plot(raw_pts[:, 0], raw_pts[:, 1], '--', color='#f38ba8', lw=2.2, label='Raw Path (A* - Jagged Grid Steps)')
ax.plot(s_pts[:, 0], s_pts[:, 1], '-', color='#89b4fa', lw=3.2, label='Smoothed Path (PCHIP - Continuous Curve)')
ax.plot([0.0, 1.5], [4.05, 4.05], 'r-', lw=3)
ax.plot(0.792, 3.535, 'go', ms=10, label='Start Point')
ax.plot(0.600, 4.701, 'm*', ms=14, label='Goal Point')
ax.set_title('Phase 3: Full Circuit Overlay: Jagged A* vs Smooth PCHIP Spline', color='white', fontsize=16, pad=12, fontweight='bold')
ax.tick_params(colors='white')
ax.legend(loc='lower left', fontsize=12, facecolor='#313244', labelcolor='white')
ax.grid(True, alpha=0.25)
frame_full = fig_to_frame(fig)
for _ in range(300): # 10s
    out.write(frame_full)

print('5. Rendering Corner Zooms...')
corners = [
    ('North U-Turn (Corner 1)', (-2.0, 5.0), 2.5),
    ('South U-Turn (Corner 2)', (-1.5, -6.5), 2.5),
    ('West Hallway Transition', (-2.0, 0.0), 2.5)
]
for title, center, span in corners:
    fig, ax = plt.subplots(figsize=(16, 9), facecolor='#181825')
    ax.set_facecolor('#181825')
    ax.imshow(grid, cmap='gray', origin='lower', extent=ext)
    ax.plot(raw_pts[:, 0], raw_pts[:, 1], 's--', color='#f38ba8', lw=2.5, ms=5, label='Raw A* (45°/90° Kinks)')
    ax.plot(s_pts[:, 0], s_pts[:, 1], 'o-', color='#a6e3a1', lw=3.5, ms=4, label='PCHIP Spline (Continuous & Safe)')
    ax.set_xlim(center[0] - span, center[0] + span)
    ax.set_ylim(center[1] - span, center[1] + span)
    ax.set_title(f'Corner Detail: {title} — Eliminating Angular Kinks', color='white', fontsize=16, pad=12, fontweight='bold')
    ax.tick_params(colors='white')
    ax.legend(loc='upper right', fontsize=12, facecolor='#313244', labelcolor='white')
    ax.grid(True, alpha=0.3)
    frame_corner = fig_to_frame(fig)
    for _ in range(180): # 6s per corner = 18s total
        out.write(frame_corner)

print('6. Rendering Vehicle Lap Animation...')
n_pts = len(s_pts)
for i in range(0, n_pts, 1):
    fig, ax = plt.subplots(figsize=(16, 9), facecolor='#181825')
    ax.set_facecolor('#181825')
    ax.imshow(grid, cmap='gray', origin='lower', extent=ext)
    ax.plot(s_pts[:, 0], s_pts[:, 1], '-', color='#45475a', lw=2)
    ax.plot(s_pts[:i+1, 0], s_pts[:i+1, 1], '-', color='#89b4fa', lw=3.5, label='Traversed Path')
    car_x, car_y = s_pts[i, 0], s_pts[i, 1]
    ax.plot(car_x, car_y, 'o', color='#f9e2af', ms=14, label='F1TENTH Vehicle')
    ax.plot([0.0, 1.5], [4.05, 4.05], 'r-', lw=3)
    ax.set_title(f'Phase 5: Vehicle Following Trajectory in AutoDRIVE (Lap Progress: {i/n_pts*100:.0f}%)', color='white', fontsize=16, pad=12, fontweight='bold')
    ax.tick_params(colors='white')
    ax.legend(loc='lower left', fontsize=12, facecolor='#313244', labelcolor='white')
    ax.grid(True, alpha=0.25)
    f = fig_to_frame(fig)
    out.write(f)

out.release()
print('Video successfully generated:', video_path)
print(f'File size: {os.path.getsize(video_path) / (1024*1024):.2f} MB')
