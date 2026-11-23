#!/usr/bin/env python3
"""Splits the bulk OUT commands of 08_volt.pcap into the windows of 08_times.txt."""
import sys, collections
sys.argv = ["x", "/home/user/vm/cap/08_volt.pcap"]
exec(open("/home/user/vm/cap/usbdump.py").read().split("args = [")[0])
HDR = bytes.fromhex("5a000f006600")
recs = [(t, parse(p)) for t, p in records("/home/user/vm/cap/08_volt.pcap")]
dev = collections.Counter(r["dev"] for _, r in recs).most_common(1)[0][0]
stream = bytearray(); when = []
for t, r in recs:
    if r["dev"] == dev and r["xfer"] == 3 and r["ep"] == 0x02 and r["typ"] == "S":
        when.append((len(stream), t)); stream += r["data"]
import bisect
pos = [w[0] for w in when]
cmds = []
k = stream.find(HDR)
while k >= 0:
    n = stream.find(HDR, k + 6)
    body = bytes(stream[k + 6:n if n >= 0 else len(stream)])
    t = when[bisect.bisect_right(pos, k) - 1][1]
    cmds.append((t, body)); k = n
times = [l.split() for l in open("/home/user/vm/cap/08_times.txt") if len(l.split()) == 3]
marks = [(float(a), b, c) for a, b, c in times]
def label(t):
    lab = "start"
    for tt, kind, name in marks:
        if t >= tt: lab = kind + ":" + name
    return lab
out = collections.OrderedDict()
for t, b in cmds:
    out.setdefault(label(t), []).append((t, b))
for lab, lst in out.items():
    c = collections.Counter(b[:2].hex() for _, b in lst)
    print("==", lab, dict(c))
    for t, b in lst:
        if b[:2].hex() not in ("2000", "0400", "0700", "0200"):
            print("   %.3f %s" % (t, b.hex()[:80]))
    seq = bytes(b[2] for _, b in lst if b[:2] == b"\x07\x00")
    if seq:
        open("/tmp/seq_%s.bin" % lab.replace(":", "_"), "wb").write(seq)
        print("   07 x%d, sha %s" % (len(seq), __import__("hashlib").sha1(seq).hexdigest()[:10]))
