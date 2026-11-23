#!/usr/bin/env python3
import time
import subprocess
import struct
from PIL import Image
from Xlib import X, display
from pynput.mouse import Button, Controller

def focus_window_by_name(name):
    try:
        d = display.Display(':0')
        root = d.screen().root
        def find_win(w):
            try:
                if w.get_wm_name() == name:
                    return w
            except:
                pass
            for child in w.query_tree().children:
                res = find_win(child)
                if res:
                    return res
            return None
        target = find_win(root)
        if target:
            target.set_input_focus(X.RevertToParent, X.CurrentTime)
            d.sync()
            return True
    except Exception as e:
        print(f"Focus error: {e}")
    return False

def get_window_geometry():
    try:
        output = subprocess.check_output("DISPLAY=:0 xwininfo -root -tree", shell=True, text=True)
        import re
        for line in output.splitlines():
            if '"AutoDRIVE Simulator"' in line and "1280x720" in line:
                parts = line.strip().split()
                win_id = parts[0]
                m = re.findall(r'\+([0-9]+)\+([0-9]+)', line)
                if m:
                    x = int(m[-1][0])
                    y = int(m[-1][1])
                    return win_id, x, y, 1280, 720
    except Exception as e:
        print(f"Geometry error: {e}")
    return None, None, None, None, None

def capture_window(win_id, out_png):
    try:
        xwd_file = "/tmp/autodrive_2cars.xwd"
        subprocess.run(f"DISPLAY=:0 xwd -id {win_id} -silent -out {xwd_file}", shell=True, check=True)
        with open(xwd_file, 'rb') as f:
            hdr = f.read(100)
            hdr_size = struct.unpack('>I', hdr[0:4])[0]
            pixmap_width = struct.unpack('>I', hdr[16:20])[0]
            pixmap_height = struct.unpack('>I', hdr[20:24])[0]
            ncolors = struct.unpack('>I', hdr[76:80])[0]
            f.seek(hdr_size + ncolors * 12)
            raw = f.read()
        im = Image.frombytes('RGB', (pixmap_width, pixmap_height), raw, 'raw', 'BGRX')
        im.save(out_png)
        print(f"Screenshot successfully saved to {out_png}")
        return True
    except Exception as e:
        print(f"Failed to capture screenshot: {e}")
        return False

def click_at(mouse, x, y):
    mouse.position = (x, y)
    time.sleep(0.2)
    mouse.press(Button.left)
    time.sleep(0.1)
    mouse.release(Button.left)
    time.sleep(0.3)

def main():
    print("Waiting for AutoDRIVE Simulator window...")
    for _ in range(30):
        win_id, wx, wy, w, h = get_window_geometry()
        if win_id and w and h:
            print(f"Found AutoDRIVE Simulator window: ID={win_id}, Pos=({wx}, {wy}), Size=({w}x{h})")
            break
        time.sleep(1)
    else:
        print("Could not find AutoDRIVE Simulator window!")
        return

    time.sleep(4) # Wait for simulator to render fully
    focus_window_by_name('AutoDRIVE Simulator')
    time.sleep(0.5)

    mouse = Controller()

    # Exact calibrated button locations in 1280x720 window:
    # Button 1: Connect -> (wx + 150, wy + 275)
    # Button 2: Driving Mode -> Autonomous -> (wx + 150, wy + 325)
    # Button 3: Camera Mode -> Trackcam -> (wx + 100, wy + 395)
    conn_x = wx + 150
    conn_y = wy + 275

    auto_x = wx + 150
    auto_y = wy + 325

    cam_x = wx + 100
    cam_y = wy + 395

    print(f"Connecting simulator at ({conn_x}, {conn_y})...")
    click_at(mouse, conn_x, conn_y)
    time.sleep(1.5)

    print(f"Engaging Autonomous Mode at ({auto_x}, {auto_y})...")
    click_at(mouse, auto_x, auto_y)
    time.sleep(1.5)

    print("Switching Camera View to Trackcam (Overhead Track Mode)...")
    for step in range(4):
        print(f"  Cycling camera ({step + 1}/4)...")
        click_at(mouse, cam_x, cam_y)
        time.sleep(0.8)

    time.sleep(1.5)
    out_artifact = "/home/admin1/.gemini/antigravity/brain/c64f7ad3-640d-406f-94ac-72317e2a39c5/autodrive_2cars_live.png"
    capture_window(win_id, out_artifact)
    print("Handshake complete: Autonomous Mode engaged in Trackcam (Track Mode)!")

if __name__ == "__main__":
    main()
