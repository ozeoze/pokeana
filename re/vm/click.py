#!/usr/bin/env python3
"""Clicks in the XP VM at screen pixel (x, y) of the 640x480 desktop: click.py x y [right|double]."""
import json, socket, sys, time
s = socket.socket(socket.AF_UNIX); s.connect("/home/user/vm/qmp.sock"); f = s.makefile("rw")
def cmd(c, a=None):
    f.write(json.dumps({"execute": c, "arguments": a or {}}) + "\n"); f.flush()
    while True:
        r = json.loads(f.readline())
        if "return" in r or "error" in r: return r
f.readline(); cmd("qmp_capabilities")
W, H = 640, 480
x, y = int(sys.argv[1]), int(sys.argv[2]); btn = "right" if "right" in sys.argv else "left"
def ev(events): cmd("input-send-event", {"events": events})
ev([{"type": "abs", "data": {"axis": "x", "value": x * 32767 // (W - 1)}},
    {"type": "abs", "data": {"axis": "y", "value": y * 32767 // (H - 1)}}]); time.sleep(0.2)
for _ in range(2 if "double" in sys.argv else 1):
    ev([{"type": "btn", "data": {"down": True, "button": btn}}]); time.sleep(0.08)
    ev([{"type": "btn", "data": {"down": False, "button": btn}}]); time.sleep(0.08)
