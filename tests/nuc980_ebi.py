#!/usr/bin/env python3
"""Checks the parallel bus decoder on a NUC980's EBI (NuMaker-Server-NUC980, firmware in
tests/nuc980_ebi/), wired CON23 -> pokeana:
    D0-D15 -> CH0-15, A0-A3 -> CH18-21, nCS1 -> CH22, nWE -> CH24, nRE -> CH23
Each firmware cycle n writes 16 words (word k at A[3:0] = k) and reads them back. On this
board A2 does not move (it reads 1) and D13-D15 can't follow (0.1 uF on PC13-15), so only
D0-D12 are checked; the cycle number is in words 0 and 1 (13 bits each).

usage: nuc980_ebi.py load                  build the firmware and run it from DDR (sudo)
       nuc980_ebi.py once [OUT.sr]         one capture with pokeana.py, decoded and checked
       nuc980_ebi.py check DUMP.jsonl      check what the GUI decoded (POKEANA_GUI_DUMP)
"""
import json
import os
import subprocess
import sys
import time

SPEC = "parallel:data=0-12,addr=18-21,cs=22,rd=23,wr=24"
WORDS = 16
MASK = 0x1FFF                       # D0-D12
A2_STUCK = 4                        # A2 reads 1


def word(n, k):
    bit = 1 << (n & 15)
    if k < 4:
        return [n & 0xFFFF, n >> 13 & 0xFFFF, bit, ~bit & 0xFFFF][k]
    return ((n * 16 + k) * 1103515245 + 12345) % (1 << 32) >> 16


def bus_addr(k):
    return k | A2_STUCK


class Checker:
    def __init__(self):
        self.n = {"captures": 0, "write ok": 0, "read ok": 0, "cut at the edge": 0, "error": 0,
                  "cycles in a row": 0}
        self.errors = []

    def bad(self, where, text):
        self.n["error"] += 1
        if len(self.errors) < 200:
            self.errors.append("%s: %s" % (where, text))

    def capture(self, rec):
        self.n["captures"] += 1
        where = "capture %d" % self.n["captures"]
        bursts = rec["decs"][0]["msgs"]
        last_n = None
        for j, b in enumerate(bursts):
            words = b["words"]
            if len(words) != WORDS or b["op"] not in ("W", "R"):
                if j in (0, len(bursts) - 1):
                    self.n["cut at the edge"] += 1
                else:
                    self.bad(where, "%s burst of %d words" % (b["op"], len(words)))
                continue
            addrs = [w[0] for w in words]
            if addrs != [bus_addr(k) for k in range(WORDS)]:
                self.bad(where, "%s addresses %s" % (b["op"], addrs))
                continue
            if b["op"] == "R":          # nothing answers: only the strobes and addresses count
                self.n["read ok"] += 1
                continue
            vals = [w[1] for w in words]
            n = vals[0] & MASK | (vals[1] & MASK) << 13
            want = [word(n, k) & MASK for k in range(WORDS)]
            if vals != want:
                self.bad(where, "write n=%d: got %s, want %s" % (n, " ".join("%04X" % v for v in vals),
                                                                 " ".join("%04X" % v for v in want)))
                continue
            self.n["write ok"] += 1
            if last_n is not None:
                if n == last_n + 1:
                    self.n["cycles in a row"] += 1
                else:
                    self.bad(where, "cycle %d after %d" % (n, last_n))
            last_n = n

    def report(self):
        return ", ".join("%s %d" % kv for kv in self.n.items())


def check(path):
    c = Checker()
    with open(path) as f:
        for line in f:
            c.capture(json.loads(line))
    print(c.report())
    for e in c.errors[:50]:
        print("  " + e)
    return c


def here():
    return os.path.dirname(os.path.abspath(__file__))


def load():
    fw = os.path.join(here(), "nuc980_ebi")
    subprocess.run(["make", "-C", fw], check=True)
    subprocess.run(["sudo", "sh", "-c", "printf R > /dev/ttyACM0"])     # a running firmware: back to USB boot
    time.sleep(1.5)
    subprocess.run(["sudo", os.path.expanduser("~/Claude/tools/NUC980_NuWriter_CMD/install/bin/nuwriter"),
                    "run.ini"], cwd=fw, check=True)


def once(out):
    root = os.path.join(here(), "..")
    sys.path.insert(0, root)
    import decode as D
    r = subprocess.run([sys.executable, os.path.join(root, "pokeana.py"), "--rate", "100e6", "--depth", "1000000",
                        "--pre", "10000", "--trigger", "22=f", "-o", out], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    print(r.stdout.strip().splitlines()[-2])
    cap = D.Capture(*D.load_sr(out))
    res = D.run(cap, *D.parse_spec(SPEC))
    for _, _, text in res.log[:6]:
        print("  ", text[:110])
    c = Checker()
    c.capture(json.loads(json.dumps({"decs": [{"msgs": res.msgs}]})))
    print(c.report())
    for e in c.errors:
        print("  " + e)


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    if cmd == "load":
        load()
    elif cmd == "once":
        once(sys.argv[2] if len(sys.argv) > 2 else "/tmp/nuc980_ebi.sr")
    elif cmd == "check":
        check(sys.argv[2])
    else:
        print(__doc__)
