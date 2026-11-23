#!/usr/bin/env python3
"""Runs LA Viewer once per trigger setting (Single Level, Direct Mode), logging the time of
each Run: trig_sweep.py "0:1" "0:X 1:up" ...  (channel:value, value 0 1 X up down any)."""
import subprocess, sys, time
def click(x, y): subprocess.run(["python3", "/home/user/vm/click.py", str(x), str(y)]); time.sleep(0.8)
def shot(name): subprocess.run(["python3", "/home/user/vm/qmp.py", "screendump", "/tmp/%s.png" % name], capture_output=True)
OPT = {"0": 18, "1": 36, "X": 55, "up": 73, "down": 91, "any": 109}
def button(ch):
    row, bit = divmod(ch, 8)
    return 490 - 50 * bit, 71 + int(33.5 * row)
log = open("/home/user/vm/cap/trig_sweep.log", "a")
for step in sys.argv[1:]:
    click(287, 50); time.sleep(4)          # Trigger: method dialog
    click(91, 328); time.sleep(4)          # OK -> direct mode
    for item in step.split():
        ch, v = item.split(":"); x, y = button(int(ch))
        click(x, y); time.sleep(1.5); click(x + 7, y + OPT[v]); time.sleep(1)
    shot("trig_" + step.replace(" ", "_").replace(":", "-"))
    click(331, 427); time.sleep(4)         # OK
    log.write("%.3f run %s\n" % (time.time(), step)); log.flush()
    click(178, 50); time.sleep(6)          # Run
    click(247, 50); time.sleep(8)          # Stop (no effect when already done)
    shot("after_" + step.replace(" ", "_").replace(":", "-"))
