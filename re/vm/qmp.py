#!/usr/bin/env python3
"""Sends one QMP command to the XP VM: qmp.py screendump out.png | qmp.py <command> [json args]."""
import json, socket, sys
s = socket.socket(socket.AF_UNIX); s.connect("/home/user/vm/qmp.sock"); f = s.makefile("rw")
f.readline(); f.write(json.dumps({"execute": "qmp_capabilities"}) + "\n"); f.flush(); f.readline()
cmd = sys.argv[1]
args = {"filename": sys.argv[2], "format": "png"} if cmd == "screendump" else json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
f.write(json.dumps({"execute": cmd, "arguments": args}) + "\n"); f.flush()
while True:
    r = json.loads(f.readline())
    if "return" in r or "error" in r: print(json.dumps(r)); break
