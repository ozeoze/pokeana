#!/usr/bin/env python3
"""Lists the transfers of one USB device in a usbmon pcap (LINKTYPE_USB_LINUX_MMAPPED).

usage: usbdump.py file.pcap [devnum] [--hex N]   (devnum: the busiest device by default)
Consecutive bulk transfers of the same size and direction are folded into one line."""
import struct, sys, collections

def records(path):
    d = open(path, "rb").read()
    endian = "<" if d[:4] == b"\xd4\xc3\xb2\xa1" else ">"
    off = 24
    while off + 16 <= len(d):
        ts, tus, incl, orig = struct.unpack(endian + "IIII", d[off:off + 16]); off += 16
        yield ts + tus / 1e6, d[off:off + incl]; off += incl

def parse(p):
    (urb, typ, xfer, ep, dev, bus, setflag, dataflag, ts, tus, status, length, dlen) = struct.unpack("<QcBBBHccqiiII", p[:40])
    setup = p[40:48]
    return dict(urb=urb, typ=typ.decode(), xfer=xfer, ep=ep, dev=dev, setup=setup, setflag=setflag, status=status,
                length=length, data=p[64:64 + dlen])

args = [a for a in sys.argv[1:] if not a.startswith("--")]
nhex = int(sys.argv[sys.argv.index("--hex") + 1]) if "--hex" in sys.argv else 16
recs = [(t, parse(p)) for t, p in records(args[0])]
devs = collections.Counter(r["dev"] for _, r in recs)
dev = int(args[1]) if len(args) > 1 else devs.most_common(1)[0][0]
print("devices:", dict(devs), "-> dev", dev)
XF = {0: "ISO", 1: "INT", 2: "CTL", 3: "BULK"}
t0 = None; last = None; fold = 0
def flush():
    global fold
    if fold: print("      ... %d more of the same" % fold); fold = 0
for t, r in recs:
    if r["dev"] != dev: continue
    t0 = t0 or t
    out = r["ep"] & 0x80 == 0
    ep = r["ep"] & 0x7f
    x = XF[r["xfer"]]
    if x == "CTL" and r["typ"] == "S":
        bm, req, val, idx, ln = struct.unpack("<BBHHH", r["setup"])
        line = "CTL  bm=%02x req=%02x val=%04x idx=%04x len=%d %s" % (bm, req, val, idx, ln, r["data"][:nhex].hex())
    elif x == "CTL" and r["typ"] == "C":
        if not r["data"]: continue
        line = "CTL<  %d: %s" % (len(r["data"]), r["data"][:nhex].hex())
    elif r["typ"] == "S" and out:
        line = "%s OUT ep%d %6d: %s" % (x, ep, r["length"], r["data"][:nhex].hex())
    elif r["typ"] == "C" and not out:
        line = "%s IN  ep%d %6d: %s" % (x, ep, len(r["data"]), r["data"][:nhex].hex())
    else:
        continue
    key = line.split(":")[0] if x == "BULK" and r["length"] >= 512 else line
    if key == last:
        fold += 1; continue
    flush(); last = key
    print("%9.3f %s" % (t - t0, line))
flush()
