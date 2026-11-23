#!/usr/bin/env python3
"""Pulls the host-to-device sequence of one LA Viewer session out of a usbmon pcap, for
pokeana.py to send again: the start-up (FPGA load, set-up) and the run prelude.

usage: extract_replay.py capture.pcap devnum run_time out.bin
  run_time: seconds from the first transfer of the device to the first Run click; the
  ops before it are the start-up, the ops from it up to the first 128-byte run set-up
  are the prelude.
out.bin: zlib of lines "phase kind fields... hexdata"; kind: C (control, bm req val idx
len data) or B (bulk, ep len data; data only for OUT)."""
import struct, sys, zlib

def records(path):
    d = open(path, "rb").read()
    off = 24
    while off + 16 <= len(d):
        ts, tus, incl, orig = struct.unpack("<IIII", d[off:off + 16]); off += 16
        if off + incl > len(d): break
        yield ts + tus / 1e6, d[off:off + incl]; off += incl

path, dev, run_t, out = sys.argv[1], int(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
lines, t0, phase = [], None, "init"
for t, p in records(path):
    urb, typ, xfer, ep, devnum = struct.unpack("<QcBBB", p[:12])
    if devnum != dev or typ != b"S": continue
    length = struct.unpack("<I", p[32:36])[0]
    data = p[64:]
    t0 = t0 or t
    if phase == "init" and t - t0 >= run_t: phase = "prelude"
    if xfer == 2:
        bm, req, val, idx, ln = struct.unpack("<BBHHH", p[40:48])
        if bm & 0x60 == 0: continue                    # standard requests: libusb does these
        lines.append("%s C %d %d %d %d %d %s" % (phase, bm, req, val, idx, ln, data.hex() if not bm & 0x80 else ""))
    elif xfer == 3:
        if phase == "prelude" and ep == 2 and length == 128: break
        lines.append("%s B %d %d %s" % (phase, ep, length, data.hex() if not ep & 0x80 else ""))
open(out, "wb").write(zlib.compress("\n".join(lines).encode(), 9))
for ph in ("init", "prelude"):
    print(ph, sum(1 for l in lines if l.startswith(ph)))
