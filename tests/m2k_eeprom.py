#!/usr/bin/env python3
"""Four I2C EEPROMs (24FC256) driven at the same time by an ADALM2000's pattern generator, for
long runs of the GUI's I2C / EEPROM decoders on real traffic.

Each cycle, every bus writes 16 bytes and then reads them back (address, Sr, read). Buses run
at different clocks and start a little apart, so their traffic overlaps. The data says which
bus and cycle it belongs to, so a decoded transaction can be checked on its own:
    [0xB0 | bus, seq >> 16, seq >> 8, seq, 12 bytes from Random(seq * 4 + bus)]
written at address (seq % 2048) * 16.

usage: m2k_eeprom.py gen [SECONDS]        drive the buses (forever by default)
       m2k_eeprom.py check DUMP.jsonl     check what the GUI decoded (POKEANA_GUI_DUMP)
       m2k_eeprom.py once OUT.sr          one cycle, captured with pokeana.py, then checked
"""
import json
import os
import random
import subprocess
import sys
import time

# (SDA DIO, SCL DIO, I2C address, bit rate). DIO0 does not move in the M2K's pattern mode, so
# the first bus's SDA is on DIO8. Each cycle every bus starts 0..300 us late (random), so the
# buses overlap in a different way each time.
BUSES = [(8, 1, 0x53, 100000), (2, 3, 0x52, 250000), (4, 5, 0x51, 125000), (6, 7, 0x50, 200000)]
MAX_OFFSET_US = 300
FS = 20_000_000                     # pattern rate: a quarter bit is a whole number of samples
N = 16
WRITE_GAP_MS = 6                    # 24FC256 write cycle: 5 ms max
PAGES = 32768 // N


def payload(bus, seq):
    r = random.Random(seq * 4 + bus)
    return [0xB0 | bus, seq >> 16 & 0xFF, seq >> 8 & 0xFF, seq & 0xFF] + [r.randrange(256) for _ in range(N - 4)]


def address(seq):
    return (seq % PAGES) * N


def bus_levels(bus, seq, total=None):
    """(SDA, SCL) for each sample of one cycle on one bus; 1 = released (open drain)."""
    _, _, dev, rate = BUSES[bus]
    off = random.Random(seq * 4 + bus + 7).uniform(0, MAX_OFFSET_US)
    q = FS // (4 * rate)
    sda, scl = [], []

    def put(d, c, n):
        sda.extend([d] * n)
        scl.extend([c] * n)

    def start(repeated=False):          # (Sr: from SCL low, SDA up first) SDA down, then SCL down
        if repeated:
            put(1, 0, q)
        put(1, 1, q); put(0, 1, 2 * q); put(0, 0, q)

    def stop():
        put(0, 0, q); put(0, 1, q); put(1, 1, 2 * q)

    def bit(b):
        put(b, 0, q); put(b, 1, 2 * q); put(b, 0, q)

    def byte(v, ack_by_master=None):    # ack_by_master: None = the device acks (release SDA)
        for k in range(8):
            bit(v >> (7 - k) & 1)
        bit(1 if ack_by_master is None else ack_by_master)

    a = address(seq)
    data = payload(bus, seq)
    put(1, 1, int(off * FS / 1e6) + 4 * q)
    start()
    for v in [dev << 1, a >> 8, a & 0xFF] + data:
        byte(v)
    stop()
    put(1, 1, WRITE_GAP_MS * FS // 1000)
    start()
    for v in (dev << 1, a >> 8, a & 0xFF):
        byte(v)
    start(repeated=True)
    byte(dev << 1 | 1)
    for k in range(N):
        byte(0xFF, ack_by_master=1 if k == N - 1 else 0)
    stop()
    if total is not None:
        put(1, 1, max(0, total - len(sda)))
    return sda, scl


def cycle_pattern(seq):
    lv = [bus_levels(b, seq) for b in range(len(BUSES))]
    total = max(len(s) for s, _ in lv) + FS // 10000
    words = [0xFFFF] * total
    for (sda, scl), (sda_dio, scl_dio, *_) in zip(lv, BUSES):
        m = ~(1 << sda_dio | 1 << scl_dio) & 0xFFFF
        for i in range(len(sda)):
            words[i] = words[i] & m | sda[i] << sda_dio | scl[i] << scl_dio
    return words


def open_m2k():
    import libm2k
    ctx = libm2k.m2kOpen("ip:192.168.2.1")
    dig = ctx.getDigital()
    dig.setSampleRateOut(FS)
    dig.setCyclic(False)
    for sda, scl, *_ in BUSES:
        for ch in (sda, scl):
            dig.setDirection(ch, libm2k.DIO_OUTPUT)
            dig.setOutputMode(ch, libm2k.DIO_OPENDRAIN)
            dig.enableChannel(ch, True)
    return libm2k, ctx, dig


def gen(seconds=None):
    libm2k, ctx, dig = open_m2k()
    seq = int(time.time()) % 10000 * 1000          # a fresh range each run (seq fits in 24 bits)
    t0 = time.time()
    n = 0
    try:
        while seconds is None or time.time() - t0 < seconds:
            words = cycle_pattern(seq)
            dig.push(words)
            time.sleep(len(words) / FS + 0.002)
            dig.stopBufferOut()
            seq += 1
            n += 1
            if n % 500 == 0:
                print("%s %d cycles, seq %d" % (time.strftime("%H:%M:%S"), n, seq), flush=True)
    finally:
        dig.stopBufferOut()
        libm2k.contextClose(ctx)


class Checker:
    """Sorts decoded I2C transactions into good writes / good reads / errors."""

    def __init__(self):
        self.n = {"captures": 0, "write ok": 0, "read ok": 0, "cut at the edge": 0, "error": 0, "all 4 at once": 0}
        self.errors = []

    def transaction(self, bus, tr, where, first):
        """bytes are (value, start, end, ack). The first one on a bus may have begun before the
        capture (a read seen from its Sr on), the last may run past it (no STOP)."""
        segs = tr["segs"]
        dev = BUSES[bus][2]
        if tr["end"] is None:
            self.n["cut at the edge"] += 1
            return None
        bad = None
        if len(segs) == 1 and segs[0]["rw"] == 0 and len(segs[0]["bytes"]) == N + 2:
            kind, s, data = "write", segs[0], [b[0] for b in segs[0]["bytes"][2:]]
            acks = [segs[0]["aack"]] + [b[3] for b in segs[0]["bytes"]]
        elif len(segs) == 2 and segs[0]["rw"] == 0 and len(segs[0]["bytes"]) == 2 and segs[1]["rw"] == 1 \
                and len(segs[1]["bytes"]) == N:
            kind, s, data = "read", segs[0], [b[0] for b in segs[1]["bytes"]]
            acks = [segs[0]["aack"], segs[1]["aack"]] + [b[3] for b in segs[0]["bytes"]] + \
                   [b[3] for b in segs[1]["bytes"][:-1]] + [not segs[1]["bytes"][-1][3]]
        elif first:
            self.n["cut at the edge"] += 1
            return None
        else:
            kind, s, data, acks = "?", segs[0] if segs else None, [], []
            bad = "unexpected shape: %s" % [(g["addr"], g["rw"], len(g["bytes"])) for g in segs]
        if bad is None:
            seq = data[1] << 16 | data[2] << 8 | data[3]
            a = s["bytes"][0][0] << 8 | s["bytes"][1][0]
            if any(g["addr"] != dev for g in segs):
                bad = "address %s" % [hex(g["addr"]) for g in segs]
            elif not all(acks):
                bad = "missing ACK"
            elif data != payload(bus, seq):
                bad = "data %s" % bytes(data).hex(" ")
            elif a != address(seq):
                bad = "memory address 0x%04X for seq %d" % (a, seq)
        if bad:
            self.n["error"] += 1
            if len(self.errors) < 200:
                self.errors.append("%s bus %d %s: %s" % (where, bus, kind, bad))
            return None
        self.n[kind + " ok"] += 1
        return tr["start"], tr["end"]

    def capture(self, rec):
        self.n["captures"] += 1
        spans = []
        for bus, dec in enumerate(rec["decs"]):
            spans.append([s for s in (self.transaction(bus, tr, "capture %d" % self.n["captures"], k == 0)
                                      for k, tr in enumerate(dec["msgs"])) if s])
        # some moment where every bus is in the middle of a transaction (if the spans of all
        # buses share a moment, the latest of their starts is one)
        if all(spans) and any(all(any(a <= t <= b for a, b in sp) for sp in spans)
                              for t in [a for sp in spans for a, _ in sp]):
            self.n["all 4 at once"] += 1

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


def once(out):
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.join(here, "..")
    sys.path.insert(0, root)
    import decode as D
    libm2k, ctx, dig = open_m2k()
    cap = subprocess.Popen([sys.executable, os.path.join(root, "pokeana.py"), "--rate", "10e6", "--depth", "500000",
                            "--pre", "5000", "--trigger", "10=f", "-o", out], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    time.sleep(3)
    words = cycle_pattern(12345)
    dig.push(words)
    time.sleep(len(words) / FS + 0.01)
    dig.stopBufferOut()
    print(cap.communicate(timeout=30)[0].decode().strip().splitlines()[-2])
    libm2k.contextClose(ctx)
    capt = D.Capture(*D.load_sr(out))
    rec = {"decs": []}
    for sda_ch, scl_ch in ((10, 9), (11, 12), (14, 15), (16, 17)):
        res = D.run(capt, *D.parse_spec("i2c:scl=%d,sda=%d,layer=eeprom" % (scl_ch, sda_ch)))
        for _, _, text in res.log:
            print("  ", text)
        rec["decs"].append({"msgs": res.msgs})
    c = Checker()
    c.capture(json.loads(json.dumps(rec)))
    print(c.report())
    for e in c.errors:
        print("  " + e)


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    if cmd == "gen":
        gen(float(sys.argv[2]) if len(sys.argv) > 2 else None)
    elif cmd == "check":
        check(sys.argv[2])
    elif cmd == "once":
        once(sys.argv[2] if len(sys.argv) > 2 else "/tmp/m2k_once.sr")
    else:
        print(__doc__)
