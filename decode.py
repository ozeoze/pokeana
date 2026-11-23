#!/usr/bin/env python3
"""Protocol decoders (I2C, SPI, UART, CAN, parallel bus) for pokeana captures, without GTK.

usage: decode.py file.sr SPEC [SPEC ...] [--ann]

  SPEC: name:key=value,...   e.g.  i2c:scl=0,sda=1
                                   spi:clk=0,mosi=1,miso=2,cs=3,mode=0
                                   uart:rx=0,baud=115200,parity=none
                                   can:rx=0,bitrate=500000
                                   parallel:data=0-15,addr=18-21,cs=22,rd=23,wr=24
  (python3 decode.py --help-specs lists every option)

Prints one line per message (I2C transaction, SPI word or CS frame, UART line, CAN
frame) with its time from the trigger; --ann prints every annotation instead.
"""
import bisect
import copy
import sys
import zipfile

CHANNELS = 36


class Capture:
    """Samples and, per channel, the indices where it changes."""

    def __init__(self, samples, rate, trig):
        self.samples, self.rate, self.trig = samples, rate, trig
        self.first = samples[0] if samples else 0
        self.edges = [[] for _ in range(CHANNELS)]
        prev = self.first
        for i, s in enumerate(samples):
            x = s ^ prev
            if x:
                while x:
                    low = x & -x
                    self.edges[low.bit_length() - 1].append(i)
                    x ^= low
                prev = s

    def __len__(self):
        return len(self.samples)

    def level(self, ch, i):
        """The value of channel ch at sample i."""
        n = bisect.bisect_right(self.edges[ch], i)
        return (self.first >> ch & 1) ^ (n & 1)


def deglitch(cap, chans, ns):
    """cap as seen with pulses shorter than ns removed on chans (slow open-drain edges and
    crosstalk can cross the threshold twice; I2C parts ignore such spikes too)."""
    w = ns * 1e-9 * cap.rate
    if w <= 1:
        return cap
    view = copy.copy(cap)
    view.edges = list(cap.edges)
    for ch in chans:
        out = []
        for e in cap.edges[ch]:
            if out and e - out[-1] < w:
                out.pop()
            else:
                out.append(e)
        view.edges[ch] = out
    return view


def load_sr(path):
    with zipfile.ZipFile(path) as z:
        meta = z.read("metadata").decode().splitlines()
        kv = dict(line.split("=", 1) for line in meta if "=" in line)
        unit = int(kv.get("unitsize", "1"))
        names = {int(k[5:]): v for k, v in kv.items() if k.startswith("probe") and k[5:].isdigit()}
        r = kv["samplerate"].split()
        rate = float(r[0]) * {"Hz": 1, "kHz": 1e3, "MHz": 1e6, "GHz": 1e9}.get(r[1] if len(r) > 1 else "Hz", 1)
        data = b"".join(z.read(n) for n in sorted((n for n in z.namelist() if n.startswith("logic-1")),
                                                     key=lambda n: int(n.split("-")[-1])))
    samples = [int.from_bytes(data[i:i + unit], "little") for i in range(0, len(data), unit)]
    trig_bit = next((k - 1 for k, v in names.items() if v == "TRIG"), None)
    trig = 0
    if trig_bit is not None:
        trig = next((i for i, s in enumerate(samples) if s >> trig_bit & 1), len(samples))
        mask = (1 << CHANNELS) - 1
        samples = [s & mask for s in samples]
    return samples, rate, trig


# ---- results ----

class Ann:
    """One box on a decoder row: samples [start, end), texts from long to short, a kind
    for the colour (ctl, addr, data, ack, nack, err, info)."""
    __slots__ = ("start", "end", "texts", "kind")

    def __init__(self, start, end, texts, kind="data"):
        self.start, self.end, self.kind = start, end, kind
        self.texts = (texts,) if isinstance(texts, str) else tuple(texts)

    def __repr__(self):
        return "Ann(%d, %d, %r, %s)" % (self.start, self.end, self.texts[0], self.kind)


class Result:
    def __init__(self, row_names):
        self.rows = [(n, []) for n in row_names]
        self.log = []                       # (start, end, text)

    def add(self, row, *a, **k):
        self.rows[row][1].append(Ann(*a, **k))

    def note(self, start, end, text):
        self.log.append((start, end, text))


def _hex(v, bits=8):
    return "%0*X" % ((bits + 3) // 4, v)


def _char(v):
    return chr(v) if 32 <= v < 127 else {9: "\\t", 10: "\\n", 13: "\\r", 0: "\\0"}.get(v, "·")


# ---- I2C ----

def _i2c(cap, scl, sda):  # noqa: C901
    res = Result(["I2C"])
    res.msgs = txs = []     # transactions: {start, end, segs: [{start, addr, rw, aack, bytes: [(v, a, b, ack)]}]}
    tx = seg = None
    pend = None
    idx = sorted(set(cap.edges[scl]) | set(cap.edges[sda]))
    c = cap.first >> scl & 1
    d = cap.first >> sda & 1
    active = False
    start_ann = None        # the S / Sr box, ended by the next SCL fall
    bits = []
    byte_start = 0
    byte_no = 0
    value = 0
    tr = None               # [start, tokens]

    def flush(end):
        nonlocal tr
        if tr and tr[1]:
            res.note(tr[0], end, " ".join(tr[1]))
        tr = None

    for i in idx:
        nc, nd = cap.level(scl, i), cap.level(sda, i)
        if c == 1 and nc == 1 and nd != d:              # SDA moves with SCL high
            if nd == 0:                                 # START / repeated START
                if active:
                    flush(i)
                start_ann = Ann(i, i, ("Sr" if active else "Start", "Sr" if active else "S"), "ctl")
                res.rows[0][1].append(start_ann)
                if not active:
                    tx = {"start": i, "end": None, "segs": []}
                    txs.append(tx)
                seg = {"start": i, "addr": None, "rw": 0, "aack": None, "bytes": []}
                tx["segs"].append(seg)
                active, bits, byte_no = True, [], 0
                tr = [i, []]
            elif active:                                # STOP
                res.add(0, i, i, ("Stop", "P"), "ctl")
                flush(i)
                tx["end"] = i
                active, bits, start_ann = False, [], None
        elif active and nc != c:
            if nc == 1:                                 # rise: a bit
                bits.append(nd)
            else:                                       # fall
                if start_ann is not None:
                    start_ann.end = i
                    start_ann = None
                    byte_start = i
                elif len(bits) == 8:
                    value = 0
                    for b in bits[:8]:
                        value = value << 1 | b
                    pend = (value, byte_start)
                    if byte_no == 0:
                        rw = "R" if value & 1 else "W"
                        if value >> 3 == 0b11110:
                            texts = ("10-bit address %s: %X.." % (rw, value >> 1 & 3), "%s %X.." % (rw, value >> 1 & 3))
                        else:
                            texts = ("Address %s: 0x%02X" % ("read" if value & 1 else "write", value >> 1),
                                     "%s %02X" % (rw, value >> 1), "%02X" % (value >> 1))
                        res.add(0, byte_start, i, texts, "addr")
                        if tr is not None:
                            tr[1].append("0x%02X %s" % (value >> 1, rw))
                    else:
                        res.add(0, byte_start, i, ("Data 0x%02X" % value, "%02X" % value), "data")
                        if tr is not None:
                            tr[1].append("%02X" % value)
                    byte_start = i
                elif len(bits) == 9:
                    ack = bits[8] == 0
                    res.add(0, byte_start, i, ("ACK", "A") if ack else ("NACK", "N"), "ack" if ack else "nack")
                    if byte_no == 0:
                        seg["addr"], seg["rw"], seg["aack"] = pend[0] >> 1, pend[0] & 1, ack
                    else:
                        seg["bytes"].append((pend[0], pend[1], i, ack))
                    if not ack and tr is not None and tr[1]:
                        tr[1][-1] += "(NACK)"
                    byte_start = i
                    bits = []
                    byte_no += 1
        c, d = nc, nd
    if active:
        flush(len(cap))
        tx["end"] = len(cap)
    return res


def i2c(cap, scl, sda, layer="none", addr_bytes="auto", glitch=150):
    res = _i2c(deglitch(cap, (scl, sda), glitch), scl, sda)
    if layer != "none":
        i2c_layer(res, layer, (2 if layer == "eeprom" else 1) if addr_bytes == "auto" else addr_bytes)
    return res


def i2c_layer(res, layer, width):
    """Register / EEPROM view: in a write the first `width` bytes are the register (or
    memory address), the rest data; a write of the address alone and then a read
    (after Sr, or in the next transaction) is a read from there."""
    name = "EEPROM" if layer == "eeprom" else "Regs"
    res.rows.append(("I2C " + name, []))
    row = len(res.rows) - 1
    res.log = []
    ptr = {}                                            # device -> the next address (current-address reads)
    fmt_a = (lambda a: "0x%04X" % a) if width == 2 else (lambda a: "0x%02X" % a)

    def dump(bs, n=16):
        return " ".join("%02X" % b for b in bs[:n]) + (" …(%d)" % len(bs) if len(bs) > n else "")

    for tx in res.msgs:
        segs = tx["segs"]
        k = 0
        while k < len(segs):
            sg = segs[k]
            k += 1
            if sg["addr"] is None:
                continue
            dev = sg["addr"]
            a0 = sg["start"]
            a1 = sg["bytes"][-1][2] if sg["bytes"] else a0 + 1
            data = [b[0] for b in sg["bytes"]]
            if not sg["aack"]:
                text, kind = "0x%02X: no answer (NACK)" % dev, "nack"
            elif sg["rw"] == 0:
                if len(data) < width:
                    text, kind = "0x%02X write (no full address): %s" % (dev, dump(data)), "err"
                else:
                    reg = 0
                    for b in data[:width]:
                        reg = reg << 8 | b
                    body = data[width:]
                    nxt = segs[k] if k < len(segs) else None
                    if body:
                        word = "write" if layer == "eeprom" else "write reg"
                        text, kind = "0x%02X %s %s: %s" % (dev, word, fmt_a(reg), dump(body)), "data"
                        ptr[dev] = reg + len(body)
                    elif nxt is not None and nxt["addr"] == dev and nxt["rw"] == 1 and nxt["aack"]:
                        rd = [b[0] for b in nxt["bytes"]]
                        word = "read" if layer == "eeprom" else "read reg"
                        text, kind = "0x%02X %s %s: %s" % (dev, word, fmt_a(reg), dump(rd)), "addr"
                        ptr[dev] = reg + len(rd)
                        a1 = nxt["bytes"][-1][2] if nxt["bytes"] else nxt["start"] + 1
                        k += 1
                    else:
                        text, kind = "0x%02X set address %s" % (dev, fmt_a(reg)), "info"
                        ptr[dev] = reg
            else:
                where = "current address %s" % fmt_a(ptr[dev]) if dev in ptr else "current address"
                text, kind = "0x%02X read (%s): %s" % (dev, where, dump(data)), "addr"
                if dev in ptr:
                    ptr[dev] += len(data)
            short = text.split(": ")[0]
            res.rows[row][1].append(Ann(a0, a1, (text, short, "%02X" % dev), kind))
            res.note(a0, a1, text)


# ---- SPI ----

def _spi(cap, clk, mosi=None, miso=None, cs=None, mode=0, order="msb", bits=8, cs_active="low"):
    lines = [(n, ch) for n, ch in (("MOSI", mosi), ("MISO", miso)) if ch is not None]
    res = Result(["SPI " + n for n, _ in lines])
    res.msgs = []           # CS frames: {start, end, MOSI: [..], MISO: [..], pos: [(a, b)]}
    cpol, cpha = mode >> 1 & 1, mode & 1
    sample_level = 1 if cpol == cpha else 0         # the level just after a sampling edge
    act = 0 if cs_active == "low" else 1
    ev = [(i, 1, "c") for i in cap.edges[clk] if cap.level(clk, i) == sample_level]
    if cs is not None:
        for i in cap.edges[cs]:
            on = cap.level(cs, i) == act
            ev.append((i, 0 if on else 2, "s"))
    ev.sort()
    selected = cs is None or (cap.first >> cs & 1) == act
    word = []                                        # sample indices of the bits so far
    vals = [0] * len(lines)
    frame = None                                     # [start, [words per line]]

    def end_frame(i):
        nonlocal frame
        if frame and frame[1][0]:
            text = "  ".join("%s: %s" % (n, " ".join(_hex(v, bits) for v in w)) for (n, _), w in zip(lines, frame[1]))
            res.note(frame[0], i, text)
            m = {"start": frame[0], "end": i, "pos": frame[2]}
            m.update({n: w for (n, _), w in zip(lines, frame[1])})
            res.msgs.append(m)
        frame = None

    for i, _, kind in ev:
        if kind == "s":
            on = cap.level(cs, i) == act
            if on:
                selected, word = True, []
                frame = [i, [[] for _ in lines], []]
            else:
                selected, word = False, []
                end_frame(i)
            continue
        if not selected:
            continue
        if not word:
            vals = [0] * len(lines)
        word.append(i)
        for k, (_, ch) in enumerate(lines):
            b = cap.level(ch, i)
            if order == "msb":
                vals[k] = vals[k] << 1 | b
            else:
                vals[k] |= b << (len(word) - 1)
        if len(word) == bits:
            half = (word[-1] - word[0]) / (2 * (bits - 1)) if bits > 1 else 1
            a, b = int(word[0] - half), int(word[-1] + half)
            for k in range(len(lines)):
                res.add(k, a, b, ("0x" + _hex(vals[k], bits), _hex(vals[k], bits)), "data")
            if cs is None:
                res.note(a, b, "  ".join("%s: %s" % (n, _hex(v, bits)) for (n, _), v in zip(lines, vals)))
            elif frame is not None:
                for k, v in enumerate(vals):
                    frame[1][k].append(v)
                frame[2].append((a, b))
            word = []
    if frame:
        end_frame(len(cap))
    return res


def spi(cap, clk, mosi=None, miso=None, cs=None, mode=0, order="msb", bits=8, cs_active="low",
        layer="none", flash_addr=3):
    res = _spi(cap, clk, mosi, miso, cs, mode, order, bits, cs_active)
    if layer == "flash":
        spi_flash(res, flash_addr, cs is not None and bits == 8)
    return res


# SPI NOR flash: opcode -> (name, address bytes (None: none, "a": 3 or 4 as set, 4: always 4),
# dummy bytes, data: "in" (MISO), "out" (MOSI) or None)
FLASH_CMDS = {
    0x9F: ("Read JEDEC ID", None, 0, "in"), 0x90: ("Read manufacturer/device ID", 3, 0, "in"),
    0xAB: ("Release power-down / device ID", None, 3, "in"), 0x4B: ("Read unique ID", None, 4, "in"),
    0x5A: ("Read SFDP", 3, 1, "in"),
    0x03: ("Read", "a", 0, "in"), 0x0B: ("Fast read", "a", 1, "in"), 0x13: ("Read (4-byte)", 4, 0, "in"),
    0x0C: ("Fast read (4-byte)", 4, 1, "in"), 0x3B: ("Dual output read", "a", 1, "in"),
    0x6B: ("Quad output read", "a", 1, "in"),
    0x02: ("Page program", "a", 0, "out"), 0x12: ("Page program (4-byte)", 4, 0, "out"),
    0x32: ("Quad page program", "a", 0, "out"),
    0x20: ("Sector erase 4 KiB", "a", 0, None), 0x21: ("Sector erase 4 KiB (4-byte)", 4, 0, None),
    0x52: ("Block erase 32 KiB", "a", 0, None), 0xD8: ("Block erase 64 KiB", "a", 0, None),
    0xDC: ("Block erase 64 KiB (4-byte)", 4, 0, None),
    0xC7: ("Chip erase", None, 0, None), 0x60: ("Chip erase", None, 0, None),
    0x06: ("Write enable", None, 0, None), 0x04: ("Write disable", None, 0, None),
    0x50: ("Volatile SR write enable", None, 0, None),
    0x05: ("Read status register 1", None, 0, "in"), 0x35: ("Read status register 2", None, 0, "in"),
    0x15: ("Read status register 3", None, 0, "in"),
    0x01: ("Write status register 1", None, 0, "out"), 0x31: ("Write status register 2", None, 0, "out"),
    0x11: ("Write status register 3", None, 0, "out"),
    0xB9: ("Power down", None, 0, None), 0x66: ("Enable reset", None, 0, None), 0x99: ("Reset", None, 0, None),
    0xB7: ("Enter 4-byte address mode", None, 0, None), 0xE9: ("Exit 4-byte address mode", None, 0, None),
    0x75: ("Erase/program suspend", None, 0, None), 0x7A: ("Erase/program resume", None, 0, None),
}
FLASH_MAKERS = {0xEF: "Winbond", 0xC2: "Macronix", 0x20: "Micron/ST", 0xC8: "GigaDevice",
                0x01: "Spansion/Cypress/Infineon", 0x9D: "ISSI", 0xBF: "SST/Microchip", 0x1F: "Adesto/Atmel",
                0x68: "Boya", 0x85: "Puya", 0x0B: "XTX", 0x5E: "Zbit", 0xA1: "Fudan", 0x1C: "EON",
                0x62: "ON Semi", 0x89: "Intel"}


def _size(n):
    for unit, k in (("GiB", 1 << 30), ("MiB", 1 << 20), ("KiB", 1 << 10)):
        if n >= k and n % k == 0:
            return "%d %s" % (n // k, unit)
    return "%d B" % n


def spi_flash(res, flash_addr, ok):
    res.rows.append(("SPI flash", []))
    row = len(res.rows) - 1
    res.log = []
    if not ok:
        res.rows[row][1].append(Ann(0, 1, ("Needs CS and 8-bit words", "CS?"), "err"))
        return
    four = flash_addr == 4

    def dump(bs, n=16):
        return " ".join("%02X" % b for b in bs[:n]) + (" …(%d bytes)" % len(bs) if len(bs) > n else "")

    for m in res.msgs:
        tx, rx = m.get("MOSI") or [], m.get("MISO") or []
        if not tx and not rx:
            continue
        a0, a1 = m["start"], m["end"]
        if not tx:
            text, kind = "(no MOSI)", "err"
        else:
            op = tx[0]
            if op not in FLASH_CMDS:
                text, kind = "Unknown command 0x%02X: %s" % (op, dump(tx[1:])), "info"
            else:
                name, na, dummy, data = FLASH_CMDS[op]
                n = (4 if four else 3) if na == "a" else (na or 0)
                if op == 0xB7:
                    four = True
                elif op == 0xE9:
                    four = False
                text, kind = name, "ctl" if data is None and not n else "data"
                p = 1 + n + dummy
                if n:
                    if len(tx) < 1 + n:
                        text, kind = name + " (short)", "err"
                    else:
                        addr = 0
                        for b in tx[1:1 + n]:
                            addr = addr << 8 | b
                        text += " 0x%0*X" % (2 * n, addr)
                        kind = "addr" if data is None else kind
                body = (rx if data == "in" else tx)[p:] if data else []
                if op == 0x9F and len(body) >= 3:
                    mk = FLASH_MAKERS.get(body[0], "unknown maker")
                    cap_ = (" (%s)" % _size(1 << body[2])) if 10 <= body[2] <= 32 else ""
                    text += ": %s (%02X), type %02X, capacity %02X%s" % (mk, body[0], body[1], body[2], cap_)
                elif op == 0x90 and len(body) >= 2:
                    text += ": %s (%02X), device %02X" % (FLASH_MAKERS.get(body[0], "unknown maker"), body[0], body[1])
                elif op == 0x05 and body:
                    sr = body[-1]
                    text += ": 0x%02X (BUSY=%d WEL=%d BP=%d)" % (sr, sr & 1, sr >> 1 & 1, sr >> 2 & 7)
                elif op in (0x35, 0x15) and body:
                    text += ": 0x%02X" % body[-1]
                elif body:
                    text += ": " + dump(body)
                elif data == "in" and not rx:
                    text += " (no MISO)"
        res.rows[row][1].append(Ann(a0, a1, (text, text.split(":")[0], "%02X" % tx[0] if tx else "?"), kind))
        res.note(a0, a1, text)


# ---- UART ----

STD_BAUDS = [300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 31250, 38400, 57600, 74880,
             76800, 115200, 230400, 250000, 460800, 500000, 921600, 1000000, 1500000, 2000000,
             3000000, 4000000]


def min_pulse(cap, ch):
    """A robust shortest pulse on ch, in samples (for automatic bit rates)."""
    e = cap.edges[ch]
    if len(e) < 3:
        return None
    w = sorted(b - a for a, b in zip(e, e[1:]))
    # the shortest that a few others also reach (one glitch should not decide)
    k = min(len(w) - 1, max(2, len(w) // 50))
    lo = w[0]
    for x in w[:k + 1]:
        if x <= 1.5 * w[k // 2]:
            lo = x
            break
    group = [x for x in w if x <= lo * 1.3]
    return sum(group) / len(group)


def auto_rate(cap, ch, standard=STD_BAUDS):
    p = min_pulse(cap, ch)
    if not p:
        return None
    r = cap.rate / p
    best = min(standard, key=lambda s: abs(s - r) / s)
    return best if abs(best - r) / best < 0.06 else round(r)


def _uart(cap, rx, baud="auto", bits=8, parity="none", stop=1.0, order="lsb", invert="no"):
    res = Result(["UART"])
    res.msgs = []                                    # (start, end, value, errors)
    if baud == "auto":
        baud = auto_rate(cap, rx)
        if not baud:
            return res
    spb = cap.rate / float(baud)
    if spb < 2:
        res.add(0, 0, len(cap), "Too slow a sample rate for %g Bd" % baud, "err")
        return res
    inv = 1 if invert == "yes" else 0
    npar = 0 if parity == "none" else 1
    edges = cap.edges[rx]
    n = len(cap)

    def v(x):
        return cap.level(rx, int(x)) ^ inv

    j = 0
    group = None                                     # [start, end, bytes]

    def flush():
        nonlocal group
        if group:
            bs = group[2]
            text = "".join(_char(b) for b in bs)
            res.note(group[0], group[1], '"%s"  %s' % (text, " ".join(_hex(b, bits) for b in bs)))
        group = None

    frame_len = (1 + bits + npar + stop) * spb
    while j < len(edges):
        s = edges[j]
        j += 1
        if v(s) != 0:                                # not a fall to the start level
            continue
        if s + frame_len > n:
            break
        if v(s + 0.5 * spb) != 0:                    # a glitch
            continue
        val = 0
        for k in range(bits):
            b = v(s + (1.5 + k) * spb)
            if order == "lsb":
                val |= b << k
            else:
                val = val << 1 | b
        err = []
        if npar:
            p = v(s + (1.5 + bits) * spb)
            ones = bin(val).count("1") + p
            if (ones & 1) != (0 if parity == "even" else 1):
                err.append("PE")
        stop_ok = v(s + (1.5 + bits + npar) * spb) == 1
        if not stop_ok:
            err.append("FE")
        end = int(s + frame_len)
        if not stop_ok and val == 0:
            res.add(0, s, end, ("Break", "BRK"), "err")
            res.msgs.append((s, end, None, ["BRK"]))
            flush()
        else:
            h = _hex(val, bits)
            texts = ("%s '%s'" % (h, _char(val)), h) if bits <= 8 else (h,)
            if err:
                texts = tuple(t + " " + "/".join(err) for t in texts) + (h,)
            res.add(0, s, end, texts, "err" if err else "data")
            res.msgs.append((s, end, val, err))
            if group and s - group[1] > 3 * frame_len:
                flush()
            if group is None:
                group = [s, end, []]
            group[1] = end
            group[2].append(val)
            if val == 10 or len(group[2]) >= 64:
                flush()
        # go on from the middle of the stop bit
        resume = s + (1 + bits + npar + 0.5) * spb
        j = bisect.bisect_left(edges, resume, j)
    flush()
    res.frame_len = frame_len
    return res


def uart(cap, rx, baud="auto", bits=8, parity="none", stop=1.0, order="lsb", invert="no", layer="none"):
    res = _uart(cap, rx, baud, bits, parity, stop, order, invert)
    if layer == "modbus":
        modbus(res)
    return res


def modbus_crc(bs):
    crc = 0xFFFF
    for b in bs:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


MODBUS_FN = {1: "Read coils", 2: "Read discrete inputs", 3: "Read holding registers", 4: "Read input registers",
             5: "Write single coil", 6: "Write single register", 15: "Write multiple coils",
             16: "Write multiple registers", 7: "Read exception status", 8: "Diagnostics",
             17: "Report server ID", 23: "Read/write multiple registers"}
MODBUS_EXC = {1: "illegal function", 2: "illegal data address", 3: "illegal data value",
              4: "server device failure", 5: "acknowledge", 6: "server device busy",
              8: "memory parity error", 10: "gateway path unavailable", 11: "gateway target failed to respond"}


def modbus_frame(b):
    """Text for one RTU frame (address .. CRC); requests and replies told apart by length."""
    u16 = lambda k: b[k] << 8 | b[k + 1]  # noqa: E731
    dev, fn, body = b[0], b[1], b[2:-2]
    head = "ID %d " % dev if dev else "Broadcast "
    if fn & 0x80:
        code = body[0] if body else None
        return head + "%s: exception %s (%s)" % (MODBUS_FN.get(fn & 0x7F, "fn %d" % (fn & 0x7F)),
                                                 code, MODBUS_EXC.get(code, "?")), "err"
    name = MODBUS_FN.get(fn, "Function %d" % fn)
    n = len(b)
    if fn in (1, 2, 3, 4):
        if n >= 5 and n == 5 + b[2]:                       # reply: byte count, data
            d = body[1:]
            if fn in (3, 4):
                vals = " ".join("0x%04X" % (d[k] << 8 | d[k + 1]) for k in range(0, len(d) - 1, 2))
            else:
                vals = "%s = %s (first one first)" % (" ".join("%02X" % x for x in d),
                                                      "".join(format(x, "08b")[::-1] for x in d))
            return head + "%s -> %s" % (name, vals), "data"
        if n == 8:
            return head + "%s 0x%04X x%d" % (name, u16(2), u16(4)), "addr"
    elif fn in (5, 6) and n == 8:
        v = u16(4)
        val = {0xFF00: "ON", 0x0000: "OFF"}.get(v, "0x%04X" % v) if fn == 5 else "0x%04X (%d)" % (v, v)
        return head + "%s 0x%04X = %s" % (name, u16(2), val), "addr"
    elif fn in (15, 16):
        if n == 8:
            return head + "%s 0x%04X x%d: done" % (name, u16(2), u16(4)), "ack"
        if n >= 9 and n == 9 + b[6]:
            d = b[7:-2]
            if fn == 16:
                vals = " ".join("0x%04X" % (d[k] << 8 | d[k + 1]) for k in range(0, len(d) - 1, 2))
            else:
                vals = "".join(format(x, "08b")[::-1] for x in d)[:u16(4)]
            return head + "%s 0x%04X x%d: %s" % (name, u16(2), u16(4), vals), "addr"
    return head + "%s: %s" % (name, " ".join("%02X" % x for x in body)), "info"


def modbus(res):
    """Modbus RTU on the UART bytes: frames split at gaps over 2.5 characters, CRC checked."""
    res.rows.append(("Modbus", []))
    row = len(res.rows) - 1
    res.log = []
    gap = 2.5 * getattr(res, "frame_len", 0)
    frames, cur = [], []
    for m in res.msgs:
        if cur and (m[0] - cur[-1][1] > gap or m[2] is None):
            frames.append(cur)
            cur = []
        if m[2] is not None:
            cur.append(m)
    if cur:
        frames.append(cur)
    for f in frames:
        a0, a1 = f[0][0], f[-1][1]
        b = [m[2] for m in f]
        if len(b) < 4:
            text, kind = "Too short for RTU: %s" % " ".join("%02X" % x for x in b), "err"
        else:
            want = modbus_crc(b[:-2])
            if b[-2] | b[-1] << 8 != want:
                text, kind = "CRC bad (want %02X %02X): %s" % (want & 0xFF, want >> 8,
                                                              " ".join("%02X" % x for x in b)), "err"
            else:
                text, kind = modbus_frame(b)
        if any(m[3] for m in f):
            text += "  (UART errors)"
            kind = "err"
        res.rows[row][1].append(Ann(a0, a1, (text, text.split(":")[0].split(" -> ")[0], "ID %d" % b[0]), kind))
        res.note(a0, a1, text)


# ---- CAN ----

STD_BITRATES = [10000, 20000, 33333, 50000, 83333, 100000, 125000, 250000, 500000, 800000, 1000000]


def can_crc(bits):
    crc = 0
    for b in bits:
        nxt = b ^ (crc >> 14 & 1)
        crc = (crc << 1) & 0x7FFF
        if nxt:
            crc ^= 0x4599
    return crc


def can(cap, rx, bitrate="auto", sample_point=75, invert="no"):
    res = Result(["CAN fields", "CAN frames"])
    if bitrate == "auto":
        bitrate = auto_rate(cap, rx, STD_BITRATES)
        if not bitrate:
            return res
    bit = cap.rate / float(bitrate)
    if bit < 3:
        res.add(0, 0, len(cap), "Too slow a sample rate for %g bit/s" % bitrate, "err")
        return res
    inv = 1 if invert == "yes" else 0
    sp = sample_point / 100.0
    edges = cap.edges[rx]
    n = len(cap)

    def lv(x):
        return cap.level(rx, int(x)) ^ inv

    class Stop(Exception):
        pass

    j = 0
    while j < len(edges):
        sof = edges[j]
        j += 1
        if lv(sof) != 0:
            continue
        if j >= 2 and sof - edges[j - 2] < 6.5 * bit:        # not after a quiet bus: mid-frame
            continue
        st = {"t": float(sof), "run": 0, "last": None, "stuffing": True}

        def raw():
            t = st["t"]
            k = bisect.bisect_left(edges, t - 0.5 * bit)
            if k < len(edges) and edges[k] < t + 0.5 * bit:   # resynchronise on an edge
                t = float(edges[k])
            if t + bit > n:
                raise Stop
            b = lv(t + sp * bit)
            st["t"] = t + bit
            return b, t

        def get(count):
            """count destuffed bits: (value, start, end, bit list)."""
            val, start, out = 0, None, []
            for _ in range(count):
                b, t = raw()
                if st["stuffing"] and st["run"] == 5:
                    if b == st["last"]:
                        raise ValueError(t)
                    st["run"], st["last"] = 1, b
                    b, t = raw()
                if start is None:
                    start = t
                if st["stuffing"]:
                    if b == st["last"]:
                        st["run"] += 1
                    else:
                        st["run"], st["last"] = 1, b
                out.append(b)
                val = val << 1 | b
            return val, int(start), int(st["t"]), out

        fields = []
        crc_bits = []

        def field(count, label=None, kind="info"):
            v, a, b, out = get(count)
            crc_bits.extend(out)
            if label:
                fields.append((a, b, label(v) if callable(label) else label, kind))
            return v, a, b

        try:
            field(1, ("SOF", "S"), "ctl")
            ida, a_id, _ = field(11)
            rtr_srr, a_r, b_r = field(1)
            ide, a_ide, b_ide = field(1)
            if ide:
                idb, _, b_id = field(18)
                ident = ida << 18 | idb
                fields.append((a_id, b_id, ("ID 0x%08X (ext)" % ident, "%08X" % ident), "addr"))
                rtr, a_r, b_r = field(1)
                fdf, a_f, b_f = field(1)
                field(1)                               # r0
            else:
                ident = ida
                fields.append((a_id, a_r, ("ID 0x%03X" % ident, "%03X" % ident), "addr"))
                rtr = rtr_srr
                fdf, a_f, b_f = field(1)
            fields.append((a_r, b_r, ("RTR", "R") if rtr else ("Data frame", "D"), "info"))
            if fdf:
                fields.append((a_f, b_f, ("CAN FD (not decoded)", "FD"), "err"))
                res.rows[0][1].extend(Ann(*f) for f in fields)
                res.note(sof, b_f, "ID 0x%X  CAN FD (not decoded)" % ident)
                continue
            dlc, a_d, b_d = field(4)
            fields.append((a_d, b_d, ("DLC %d" % dlc, "%d" % dlc), "info"))
            data = []
            if not rtr:
                for _ in range(min(dlc, 8)):
                    v, a, b = field(8)
                    data.append(v)
                    fields.append((a, b, ("0x%02X" % v, "%02X" % v), "data"))
            want = can_crc(crc_bits)
            crc, a, b, _ = get(15)
            ok = crc == want
            fields.append((a, b, ("CRC 0x%04X %s" % (crc, "OK" if ok else "bad, want 0x%04X" % want),
                                  "CRC %s" % ("OK" if ok else "NG"), "CRC"), "ack" if ok else "err"))
            if st["run"] == 5:                         # a stuff bit after the CRC's last 5
                b5, t5 = raw()
                if b5 == st["last"]:
                    raise ValueError(t5)
            st["stuffing"] = False
            delim, a, b, _ = get(1)
            if not delim:
                fields.append((a, b, ("CRC delimiter error", "DE"), "err"))
            ack, a, b, _ = get(1)
            fields.append((a, b, ("ACK", "A") if ack == 0 else ("NACK", "N"), "ack" if ack == 0 else "nack"))
            get(1)                                     # ACK delimiter
            eof, a, end, _ = get(7)
            fields.append((a, end, ("EOF", "E") if eof == 0x7F else ("EOF error", "E!"), "ctl" if eof == 0x7F else "err"))
            res.rows[0][1].extend(Ann(*f) for f in fields)
            kind = ("ext " if ide else "") + ("RTR " if rtr else "")
            body = " ".join("%02X" % v for v in data)
            summary = "ID 0x%X %s[%d] %s" % (ident, kind, dlc, body)
            flags = ([] if ok else ["CRC NG"]) + ([] if ack == 0 else ["NACK"])
            if flags:
                summary += "  (" + ", ".join(flags) + ")"
            res.add(1, sof, end, (summary.replace("  ", " "), "0x%X [%d] %s" % (ident, dlc, body), "%X" % ident),
                    "data" if ok else "err")
            res.note(sof, end, summary)
            j = bisect.bisect_left(edges, end, j)
        except ValueError as e:
            t = int(e.args[0])
            res.rows[0][1].extend(Ann(*f) for f in fields)
            res.add(0, t, int(t + bit), ("Stuff error (error frame?)", "Stuff", "!"), "err")
            res.note(sof, int(t + bit), "stuff error (an error frame, or not CAN at this bit rate)")
            j = bisect.bisect_left(edges, t + bit, j)
        except Stop:
            break
    return res


# ---- parallel bus ----

def _ch_list(chs):
    return list(chs) if chs else []


def parallel(cap, data, addr=None, cs=None, rd=None, wr=None, clk=None, clk_edge="rise", active="low",
             glitch=0):
    """A bus of several lines read together: data (and address) lines, sampled when a read or
    write strobe ends (rd / wr), on a clock edge (clk), or else whenever they change. Lists
    are LSB first. cs, rd and wr are active low unless active=high."""
    data, addr = _ch_list(data), _ch_list(addr)
    used = data + addr + [c for c in (cs, rd, wr, clk) if c is not None]
    cap = deglitch(cap, used, glitch) if glitch else cap
    on = 0 if active == "low" else 1
    res = Result(["Bus"])
    res.msgs = []           # bursts: {start, end, op, words: [(addr or None, value, start, end)]}
    dw, aw = len(data), len(addr)

    def value(chs, i):
        return sum(cap.level(ch, i) << k for k, ch in enumerate(chs))

    def selected(i):
        return cs is None or cap.level(cs, i) == on

    acc = []                # (start, end, op, sample index)
    if rd is not None or wr is not None:
        for ch, op in ((rd, "R"), (wr, "W")):
            if ch is None:
                continue
            e = cap.edges[ch]
            for k, i in enumerate(e):
                if cap.level(ch, i) != on and i > 0 and cap.level(ch, i - 1) == on:
                    start = e[k - 1] if k > 0 else 0
                    if selected(i - 1):
                        acc.append((start, i, op, i - 1))
    elif clk is not None:
        e = cap.edges[clk]
        lv = 1 if clk_edge == "rise" else 0
        for k, i in enumerate(e):
            if cap.level(clk, i) == lv and i > 0 and selected(i - 1):
                start = e[k - 1] if k > 0 else max(0, 2 * i - (e[k + 1] if k + 1 < len(e) else i + 1))
                end = e[k + 1] if k + 1 < len(e) else len(cap)
                acc.append(((start + i) // 2, (i + end) // 2, "", i - 1))
    else:
        pts = sorted(set(i for ch in data + addr + ([cs] if cs is not None else []) for i in cap.edges[ch]))
        pts = [0] + pts + [len(cap)]
        last = None
        for a, b in zip(pts, pts[1:]):
            if b > a and selected(a):
                key = (value(data, a), value(addr, a))
                if acc and acc[-1][1] == a and key == last:        # a cs edge, same value
                    acc[-1] = (acc[-1][0], b, "", acc[-1][3])
                else:
                    acc.append((a, b, "", a))
                last = key
    acc.sort()

    names = {"R": "Read", "W": "Write", "": "Value"}
    burst = None

    def flush():
        if burst:
            m = burst
            words = " ".join(("%s:" % _hex(a, aw) if a is not None else "") + _hex(v, dw) for a, v, _, _ in m["words"])
            res.note(m["start"], m["end"], "%s %d: %s" % (names[m["op"]].lower(), len(m["words"]), words))
            res.msgs.append(m)

    for start, end, op, i in acc:
        v = value(data, i)
        a = value(addr, i) if addr else None
        long = "%s%s0x%s" % (names[op] + " " if op else "", "0x%s: " % _hex(a, aw) if a is not None else "", _hex(v, dw))
        mid = "%s%s%s" % (op + " " if op else "", "%s:" % _hex(a, aw) if a is not None else "", _hex(v, dw))
        res.add(0, start, end, (long, mid, _hex(v, dw)), "addr" if op == "R" else "data")
        if burst is None or burst["op"] != op or start - burst["end"] > 4 * max(2, burst["last"]):
            flush()
            burst = {"start": start, "end": end, "op": op, "words": [], "last": end - start}
        burst["words"].append((a, v, start, end))
        burst["end"], burst["last"] = end, end - start
    flush()
    for m in res.msgs:
        m.pop("last", None)
    return res


# ---- the table of decoders, specs ----
# option: (key, label, kind, default, choices); kinds: ch (a channel), ch? (or none),
# chs / chs? (a list of channels, LSB first: 0-7+9),
# choice, rate (a number or "auto"), int

DECODERS = {
    "i2c": ("I2C", i2c, [("scl", "SCL", "ch", 0, None), ("sda", "SDA", "ch", 1, None),
                         ("layer", "上位の解析", "choice", "none", ["none", "eeprom", "regs"]),
                         ("addr_bytes", "番地 / レジスタのバイト数", "choice", "auto", ["auto", 1, 2]),
                         ("glitch", "これより短いパルスは無視 (ns)", "int", 150, None)]),
    "spi": ("SPI", spi, [("clk", "CLK", "ch", 0, None), ("mosi", "MOSI", "ch?", 1, None),
                         ("miso", "MISO", "ch?", 2, None), ("cs", "CS", "ch?", 3, None),
                         ("mode", "モード (CPOL/CPHA)", "choice", 0, [0, 1, 2, 3]),
                         ("order", "ビット順", "choice", "msb", ["msb", "lsb"]),
                         ("bits", "ワードのビット数", "int", 8, None),
                         ("cs_active", "CS のアクティブ", "choice", "low", ["low", "high"]),
                         ("layer", "上位の解析", "choice", "none", ["none", "flash"]),
                         ("flash_addr", "フラッシュの番地のバイト数", "choice", 3, [3, 4])]),
    "uart": ("UART", uart, [("rx", "信号", "ch", 0, None),
                            ("baud", "ボーレート（auto で自動）", "rate", "auto", None),
                            ("bits", "データビット", "choice", 8, [5, 6, 7, 8, 9]),
                            ("parity", "パリティ", "choice", "none", ["none", "even", "odd"]),
                            ("stop", "ストップビット", "choice", 1.0, [1.0, 1.5, 2.0]),
                            ("order", "ビット順", "choice", "lsb", ["lsb", "msb"]),
                            ("invert", "反転", "choice", "no", ["no", "yes"]),
                            ("layer", "上位の解析", "choice", "none", ["none", "modbus"])]),
    "can": ("CAN", can, [("rx", "信号（RX/TX か CANL）", "ch", 0, None),
                         ("bitrate", "ビットレート（auto で自動）", "rate", "auto", None),
                         ("sample_point", "サンプル点 %", "int", 75, None),
                         ("invert", "反転（CANH を見るとき yes）", "choice", "no", ["no", "yes"])]),
    "parallel": ("Parallel", parallel, [("data", "データ線（下の桁から。0-7、0-7+9 など）", "chs", (0, 1, 2, 3, 4, 5, 6, 7), None),
                                        ("addr", "アドレス線（なくてもよい）", "chs?", None, None),
                                        ("cs", "CS", "ch?", None, None),
                                        ("rd", "RD（読み出しのストローブ）", "ch?", None, None),
                                        ("wr", "WR（書き込みのストローブ）", "ch?", None, None),
                                        ("clk", "クロック（RD / WR がないとき）", "ch?", None, None),
                                        ("clk_edge", "クロックのエッジ", "choice", "rise", ["rise", "fall"]),
                                        ("active", "CS / RD / WR のアクティブ", "choice", "low", ["low", "high"]),
                                        ("glitch", "これより短いパルスは無視 (ns)", "int", 0, None)]),
}


def _parse_value(kind, text, choices):
    text = str(text).strip()
    if kind in ("chs", "chs?"):
        if text.lower() in ("", "-", "none"):
            if kind == "chs?":
                return None
            raise ValueError("no channels")
        out = []
        for part in text.lower().replace("ch", "").split("+"):
            a, _, b = part.partition("-")
            out += range(int(a), int(b or a) + 1) if int(b or a) >= int(a) else range(int(a), int(b) - 1, -1)
        if not all(0 <= v < CHANNELS for v in out):
            raise ValueError("channels %s" % text)
        return tuple(out)
    if kind in ("ch", "ch?"):
        if kind == "ch?" and text.lower() in ("", "-", "none"):
            return None
        v = int(text.lower().lstrip("ch"))
        if not 0 <= v < CHANNELS:
            raise ValueError("channel %d" % v)
        return v
    if kind == "rate":
        if text.lower() == "auto":
            return "auto"
        t = text.lower().replace("m", "e6").replace("k", "e3")
        return float(t)
    if kind == "int":
        return int(text)
    for c in choices:
        if str(c) == text or (isinstance(c, float) and _isnum(text) and float(text) == c) \
                or (isinstance(c, int) and text.isdigit() and int(text) == c):
            return c
    raise ValueError("%s: one of %s" % (text, ", ".join(map(str, choices))))


def _isnum(t):
    try:
        float(t)
        return True
    except ValueError:
        return False


def parse_spec(spec):
    """'i2c:scl=0,sda=1' -> ('i2c', {options with defaults})."""
    name, _, rest = spec.partition(":")
    name = name.strip().lower()
    if name not in DECODERS:
        raise ValueError("decoder %r: one of %s" % (name, ", ".join(DECODERS)))
    opts = {k: d for k, _, _, d, _ in DECODERS[name][2]}
    table = {k: (kind, ch) for k, _, kind, _, ch in DECODERS[name][2]}
    for kv in filter(None, (x.strip() for x in rest.split(","))):
        k, _, v = kv.partition("=")
        if k not in table:
            raise ValueError("%s: no option %r (%s)" % (name, k, ", ".join(table)))
        opts[k] = _parse_value(table[k][0], v, table[k][1])
    return name, opts


def ch_ranges(chs):
    """(0, 1, 2, 3, 9) -> "0-3+9" (runs up or down, in the given order)."""
    out, k = [], 0
    while k < len(chs):
        j = k
        step = 1 if k + 1 < len(chs) and chs[k + 1] == chs[k] + 1 else -1
        while j + 1 < len(chs) and chs[j + 1] == chs[j] + step:
            j += 1
        out.append("%d" % chs[k] if j == k else "%d-%d" % (chs[k], chs[j]))
        k = j + 1
    return "+".join(out)


def spec_str(name, opts):
    def fmt(v):
        if v is None:
            return "none"
        if isinstance(v, float):
            return "%g" % v
        if isinstance(v, (tuple, list)):
            return ch_ranges(v)
        return str(v)
    return name + ":" + ",".join("%s=%s" % (k, fmt(opts[k])) for k, *_ in DECODERS[name][2])


def run(cap, name, opts):
    return DECODERS[name][1](cap, **opts)


def main(argv):
    if "--help-specs" in argv:
        for name, (title, _, options) in DECODERS.items():
            print("%s (%s):" % (name, title))
            for k, label, kind, d, ch in options:
                print("  %-12s %-6s default %-8s %s%s" % (k, kind, d, label, " " + "/".join(map(str, ch)) if ch else ""))
        return 0
    args = [a for a in argv if not a.startswith("--")]
    if len(args) < 2:
        print(__doc__)
        return 2
    cap = Capture(*load_sr(args[0]))
    for spec in args[1:]:
        name, opts = parse_spec(spec)
        res = run(cap, name, opts)
        print("# %s  (%d samples at %g Hz)" % (spec_str(name, opts), len(cap), cap.rate))
        t = lambda i: (i - cap.trig) / cap.rate * 1e6  # noqa: E731
        if "--ann" in argv:
            for rname, anns in res.rows:
                for a in anns:
                    print("%12.3f us %-10s %-5s %s" % (t(a.start), rname, a.kind, a.texts[0]))
        else:
            for a, _, text in res.log:
                print("%12.3f us  %s" % (t(a), text))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
