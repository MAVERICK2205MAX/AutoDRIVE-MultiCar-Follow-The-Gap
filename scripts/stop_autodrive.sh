#!/bin/bash
echo "=== Stopping all AutoDRIVE, ROS 2, and F1TENTH processes ==="
fuser -k 4567/tcp 2>/dev/null || true
pkill -9 -f "AutoDRIVE Simulator" 2>/dev/null || true
pkill -9 -f "simulator_bringup" 2>/dev/null || true
pkill -9 -f "autodrive_multicar_sim" 2>/dev/null || true
pkill -9 -f "reactive_node" 2>/dev/null || true
pkill -9 -f "autodrive_incoming_bridge" 2>/dev/null || true
pkill -9 -f "autodrive_outgoing_bridge" 2>/dev/null || true
pkill -9 -f "autodrive_bridge" 2>/dev/null || true
pkill -9 -f "autodrive_roboracer" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
sleep 1
fuser -k 4567/tcp 2>/dev/null || true
echo "All processes stopped cleanly."
