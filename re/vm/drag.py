#!/usr/bin/env python3
"""Drags in the XP VM: drag.py x0 y0 x1 y1 (640x480 desktop)."""
import json, socket, sys, time
s = socket.socket(socket.AF_UNIX); s.connect("/home/user/vm/qmp.sock"); f = s.makefile("rw")
def cmd(c, a=None):
    f.write(json.dumps({"execute": c, "arguments": a or {}}) + "\n"); f.flush()
    while True:
        r = json.loads(f.readline())
        if "return" in r or "error" in r: return r
f.readline(); cmd("qmp_capabilities")
def move(x, y): cmd("input-send-event", {"events": [{"type": "abs", "data": {"axis": "x", "value": x * 32767 // 639}}, {"type": "abs", "data": {"axis": "y", "value": y * 32767 // 479}}]}); time.sleep(0.05)
def btn(d): cmd("input-send-event", {"events": [{"type": "btn", "data": {"down": d, "button": "left"}}]}); time.sleep(0.1)
x0, y0, x1, y1 = map(int, sys.argv[1:5])
move(x0, y0); time.sleep(0.2); btn(True)
n = 20
for i in range(1, n + 1): move(x0 + (x1 - x0) * i // n, y0 + (y1 - y0) * i // n)
time.sleep(0.2); btn(False)
