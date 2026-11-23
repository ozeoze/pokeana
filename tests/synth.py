#!/usr/bin/env python3
"""Synthetic I2C / SPI / UART / CAN waveforms, written as .sr files, for checking
decode.py against sigrok-cli's decoders.

usage: synth.py OUTDIR
"""
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import pokeana as P  # noqa: E402


class Wave:
    """Levels of a few channels over time, sampled at rate."""

    def __init__(self, rate, idle):
        self.rate, self.t = rate, 0.0
        self.state = dict(idle)
        self.ev = [(0.0, dict(idle))]

    def set(self, **kv):
        st = dict(self.state)
        st.update({int(k[2:]): v for k, v in kv.items()})
        self.state = st
        self.ev.append((self.t, st))

    def wait(self, secs):
        self.t += secs

    def samples(self, jitter=0.0):
        n = int(self.t * self.rate) + 1
        out = []
        k = 0
        cur = 0
        for i in range(n):
            t = i / self.rate
            while k < len(self.ev) and self.ev[k][0] + random.uniform(0, jitter) <= t:
                cur = sum(v << ch for ch, v in self.ev[k][1].items())
                k += 1
            out.append(cur)
        return out


def i2c(w, scl, sda, f, msgs):
    q = 1 / f / 4
    s = {"ch%d" % scl: 1, "ch%d" % sda: 1}
    w.set(**s)
    w.wait(10 * q)
    for addr, rw, data, acks in msgs:
        w.set(**{"ch%d" % sda: 0}); w.wait(2 * q)                    # START (or Sr)
        w.set(**{"ch%d" % scl: 0}); w.wait(q)
        for byte, ack in zip([addr << 1 | rw] + data, acks):
            for b in [byte >> (7 - k) & 1 for k in range(8)] + [0 if ack else 1]:
                w.set(**{"ch%d" % sda: b}); w.wait(q)
                w.set(**{"ch%d" % scl: 1}); w.wait(2 * q)
                w.set(**{"ch%d" % scl: 0}); w.wait(q)
        w.set(**{"ch%d" % sda: 1}); w.wait(q)
        w.set(**{"ch%d" % scl: 1}); w.wait(q)
    w.set(**{"ch%d" % scl: 0}); w.wait(q)
    w.set(**{"ch%d" % sda: 0}); w.wait(q)
    w.set(**{"ch%d" % scl: 1}); w.wait(q)
    w.set(**{"ch%d" % sda: 1}); w.wait(10 * q)                       # STOP


def spi(w, clk, mosi, miso, cs, f, mode, frames):
    cpol, cpha = mode >> 1, mode & 1
    h = 1 / f / 2
    w.set(**{"ch%d" % clk: cpol, "ch%d" % cs: 1}); w.wait(4 * h)
    for tx, rx in frames:
        w.set(**{"ch%d" % cs: 0}); w.wait(h)
        for a, b in zip(tx, rx):
            for k in range(8):
                ba, bb = a >> (7 - k) & 1, b >> (7 - k) & 1
                if cpha == 0:
                    w.set(**{"ch%d" % mosi: ba, "ch%d" % miso: bb}); w.wait(h)
                    w.set(**{"ch%d" % clk: 1 - cpol}); w.wait(h)
                    w.set(**{"ch%d" % clk: cpol})
                else:
                    w.set(**{"ch%d" % clk: 1 - cpol, "ch%d" % mosi: ba, "ch%d" % miso: bb}); w.wait(h)
                    w.set(**{"ch%d" % clk: cpol}); w.wait(h)
        w.wait(h)
        w.set(**{"ch%d" % cs: 1}); w.wait(4 * h)


def uart(w, ch, baud, data, parity="none", stop=1):
    b = 1 / baud
    w.set(**{"ch%d" % ch: 1}); w.wait(3 * b)
    for v in data:
        bits = [0] + [v >> k & 1 for k in range(8)]
        if parity != "none":
            p = bin(v).count("1") & 1
            bits.append(p if parity == "even" else 1 - p)
        bits += [1] * stop
        for x in bits:
            w.set(**{"ch%d" % ch: x}); w.wait(b)
        w.wait(random.choice([0, 0, 0.3 * b, 2 * b]))
    w.wait(5 * b)


def can_bits(ident, data, ext=False, rtr=False, dlc=None, ack=True):
    def bits(v, n):
        return [v >> (n - 1 - k) & 1 for k in range(n)]
    dlc = len(data) if dlc is None else dlc
    if ext:
        f = [0] + bits(ident >> 18, 11) + [1, 1] + bits(ident & 0x3FFFF, 18) + [int(rtr), 0, 0]
    else:
        f = [0] + bits(ident, 11) + [int(rtr), 0, 0]
    f += bits(dlc, 4)
    if not rtr:
        for d in data:
            f += bits(d, 8)
    crc = 0
    for b in f:
        nxt = b ^ (crc >> 14 & 1)
        crc = (crc << 1) & 0x7FFF
        if nxt:
            crc ^= 0x4599
    f += bits(crc, 15)
    out, run, last = [], 0, None
    for b in f:
        out.append(b)
        run = run + 1 if b == last else 1
        last = b
        if run == 5:
            out.append(1 - b)
            run, last = 1, 1 - b
    return out + [1, 0 if ack else 1, 1] + [1] * 7


def can(w, ch, bitrate, frames):
    b = 1 / bitrate
    w.set(**{"ch%d" % ch: 1}); w.wait(15 * b)
    for fr in frames:
        for x in can_bits(**fr):
            w.set(**{"ch%d" % ch: x}); w.wait(b)
        w.wait(random.choice([3, 5, 20]) * b)


def main(out):
    random.seed(1)
    os.makedirs(out, exist_ok=True)

    def save(name, w, jitter=0.0):
        s = w.samples(jitter)
        P.write_sr(os.path.join(out, name + ".sr"), s, w.rate, 36, 0)
        print(name, len(s))

    w = Wave(10e6, {0: 1, 1: 1})
    i2c(w, 0, 1, 100e3, [(0x50, 0, [0x00, 0x10, 0xA5], [1, 1, 1, 1]), (0x50, 1, [0xDE, 0xAD], [1, 1, 0])])
    i2c(w, 0, 1, 400e3, [(0x3C, 0, [0x12], [0, 1])])
    save("i2c", w, jitter=0.2e-6)

    for mode in range(4):
        w = Wave(20e6, {0: mode >> 1, 1: 0, 2: 0, 3: 1})
        spi(w, 0, 1, 2, 3, 1e6, mode, [([0x9F, 0x00, 0x00], [0xFF, 0xEF, 0x40]),
                                       ([random.randrange(256) for _ in range(5)],
                                        [random.randrange(256) for _ in range(5)])])
        save("spi%d" % mode, w)

    w = Wave(10e6, {0: 1, 1: 1})
    uart(w, 0, 115200, b"Hello pokeana\r\n" + bytes([0x00, 0x55, 0xFF, 0x80]))
    save("uart", w, jitter=0.1e-6)
    w = Wave(20e6, {0: 1})
    uart(w, 0, 1e6, bytes(range(0x30, 0x40)), parity="even", stop=2)
    save("uart8e2", w)

    # upper layers: an I2C EEPROM, an SPI flash, Modbus RTU
    w = Wave(4e6, {0: 1, 1: 1})
    i2c(w, 0, 1, 100e3, [(0x50, 0, [0x00, 0x10, 0xA5, 0xA6], [1] * 5)])
    i2c(w, 0, 1, 100e3, [(0x50, 0, [0x00, 0x10], [1] * 3), (0x50, 1, [0xA5, 0xA6], [1, 1, 0])])
    i2c(w, 0, 1, 100e3, [(0x50, 1, [0x77], [1, 0])])
    i2c(w, 0, 1, 100e3, [(0x51, 0, [0x00], [0, 1])])
    save("i2c_eeprom", w)

    w = Wave(20e6, {0: 0, 1: 0, 2: 0, 3: 1})
    spi(w, 0, 1, 2, 3, 1e6, 0, [
        ([0x9F, 0, 0, 0], [0xFF, 0xEF, 0x40, 0x18]),
        ([0x06], [0xFF]),
        ([0x05, 0], [0xFF, 0x03]),
        ([0x20, 0x01, 0x00, 0x00], [0xFF] * 4),
        ([0x02, 0x00, 0x20, 0x00, 0xA5, 0x5A], [0xFF] * 6),
        ([0x03, 0x00, 0x10, 0x00, 0, 0, 0, 0], [0xFF] * 4 + [0x11, 0x22, 0x33, 0x44]),
        ([0x0B, 0x00, 0x00, 0x10, 0x00, 0, 0], [0xFF] * 5 + [0xAA, 0xBB]),
        ([0x90, 0, 0, 0, 0, 0], [0xFF] * 4 + [0xEF, 0x17]),
        ([0xB9], [0xFF])])
    save("spi_flash", w)

    def mb(*b):
        c = 0xFFFF
        for x in b:
            c ^= x
            for _ in range(8):
                c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
        return bytes(b) + bytes([c & 0xFF, c >> 8])
    w = Wave(2e6, {0: 1})
    for f in (mb(1, 3, 0, 0, 0, 2), mb(1, 3, 4, 0x12, 0x34, 0x56, 0x78), mb(1, 6, 0, 1, 0, 3),
              mb(1, 0x10, 0, 0x10, 0, 2, 4, 0, 0x0A, 1, 2), mb(1, 0x10, 0, 0x10, 0, 2), mb(1, 0x83, 2),
              mb(2, 1, 0, 0x13, 0, 0x0A), mb(2, 1, 2, 0xCD, 0x01), bytes([1, 3, 0, 0, 0, 2, 0, 0])):
        uart(w, 0, 19200, f, parity="even")
        w.wait(10 * 11 / 19200)
    save("modbus", w)

    w = Wave(10e6, {0: 1})
    can(w, 0, 500e3, [dict(ident=0x123, data=[0x11, 0x22, 0x33]),
                      dict(ident=0x7FF, data=[0xFF] * 8),
                      dict(ident=0x000, data=[0x00] * 8),
                      dict(ident=0x1ABCDE12, data=[0xCA, 0xFE], ext=True),
                      dict(ident=0x456, data=[], rtr=True, dlc=4),
                      dict(ident=0x0F0, data=[0x0F, 0xF0, 0x1F], ack=False),
                      dict(ident=0x555, data=[random.randrange(256) for _ in range(8)])])
    save("can", w)
    w = Wave(10e6, {0: 1})
    can(w, 0, 500e3, [dict(ident=0x123, data=[0x11, 0x22, 0x33])] * 3)
    save("can_jitter", w, jitter=0.15e-6)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "synth")
