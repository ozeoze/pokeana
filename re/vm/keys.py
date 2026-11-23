#!/usr/bin/env python3
"""Presses keys in the XP VM over QMP. Each argument is a key or a chord (alt-p, ret,
backspace*5); "text:" types the line read from stdin (letters, digits; "-" moves on)."""
import json, socket, sys, time
s = socket.socket(socket.AF_UNIX); s.connect("/home/user/vm/qmp.sock"); f = s.makefile("rw")
def cmd(c, a=None):
    f.write(json.dumps({"execute": c, "arguments": a or {}}) + "\n"); f.flush()
    while True:
        r = json.loads(f.readline())
        if "return" in r or "error" in r: return r
f.readline(); cmd("qmp_capabilities")
def press(chord):
    cmd("send-key", {"keys": [{"type": "qcode", "data": k} for k in chord.split("-")]}); time.sleep(0.15)
for a in sys.argv[1:]:
    if a == "text:":
        for ch in sys.stdin.readline().strip().lower():
            if ch != "-": press(ch)
        continue
    k, _, n = a.partition("*")
    for _ in range(int(n or 1)): press(k)
