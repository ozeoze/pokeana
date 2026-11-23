#!/usr/bin/env python3
"""Puts protocol test signals on the AtomS3U (MicroPython) wired to pokeana CH0..3
(GPIO14, 17, 42, 40), for trying decode.py on real captures.

usage: sudo python3 atom_signals.py MODE [/dev/ttyACM0]
  MODE  mix   CH0 SCL, CH1 SDA (I2C 100 kHz; an EEPROM-like 0x50: address 0x00nn, then a
              read after Sr), CH2 UART 115200 8N1, CH3 CAN 500 kbit/s
        spi   CH0 CLK, CH1 MOSI, CH2 MISO, CH3 CS: a 1 MHz hardware SPI burst (mode 0,
              MISO not driven), then a software one in each of modes 0..3, then SPI
              flash commands in mode 0 (JEDEC ID, status, read)
        uart  CH0 115200 8N1, CH1 Modbus RTU 19200 8E1 (a request, its reply, a write)
        can   CH0 500 kbit/s, CH1 125 kbit/s (standard, extended, remote, NACK frames)
        clk   the old square waves: 1 kHz, 10 kHz, 100 kHz, 1 MHz
Each burst repeats every 20 ms. The mode is kept in the board's mode.txt.
"""
import sys
import time

import serial

MAIN = r'''
import time, esp32
from machine import Pin, PWM, I2C, SPI, UART
P0, P1, P2, P3 = 14, 17, 42, 40
try:
    MODE = open("mode.txt").read().strip()
except OSError:
    MODE = "clk"

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

def pulses(bits, ticks):
    d, n, cur = [], 0, bits[0]
    for b in bits:
        if b == cur:
            n += ticks
        else:
            d.append(n)
            n, cur = ticks, b
    d.append(n)
    return d

def can_rmt(ch, pin, bitrate):
    # 80 MHz / div: 20 ticks a bit
    return esp32.RMT(ch, pin=Pin(pin), clock_div=80_000_000 // (bitrate * 20), idle_level=True)

def can_send(rmt, frame):
    rmt.write_pulses(pulses(can_bits(**frame), 20), False)
    rmt.wait_done(timeout=50)

class BitI2C:
    """Bit-banged master that also plays the device's ACKs (one wire, so the waveform
    is what a real device would make)."""
    def __init__(self, scl, sda):
        self.c = Pin(scl, Pin.OUT, value=1)
        self.d = Pin(sda, Pin.OUT, value=1)
    def bit(self, b):
        self.d(b); self.c(1); self.c(0)
    def start(self):
        self.d(1); self.c(1); self.d(0); self.c(0)
    def stop(self):
        self.d(0); self.c(1); self.d(1)
    def byte(self, v, ack=True):
        for k in range(8):
            self.bit(v >> (7 - k) & 1)
        self.bit(0 if ack else 1)

def bit_spi(clk, mosi, miso, cs, mode, tx, rx):
    cpol, cpha = mode >> 1, mode & 1
    c, o, i, s = (Pin(p, Pin.OUT) for p in (clk, mosi, miso, cs))
    c(cpol); s(0)
    for a, b in zip(tx, rx):
        for k in range(8):
            ba, bb = a >> (7 - k) & 1, b >> (7 - k) & 1
            if cpha == 0:
                o(ba); i(bb); c(1 - cpol); c(cpol)
            else:
                c(1 - cpol); o(ba); i(bb); c(cpol)
    s(1)

if MODE == "clk":
    pw = [PWM(Pin(p), freq=f, duty_u16=32768) for p, f in ((P0, 1000), (P1, 10000), (P2, 100000), (P3, 1000000))]
elif MODE == "mix":
    u = UART(1, baudrate=115200, tx=P2, rx=21)
    rmt = can_rmt(0, P3, 500000)
    n = 0
    while True:
        i2c = I2C(0, scl=Pin(P0), sda=Pin(P1), freq=100000)
        try:
            i2c.writeto(0x3C, b"\x00\xAF")               # no device: an address NACK
        except OSError:
            pass
        bi = BitI2C(P0, P1)                              # with ACKs: write, Sr, read
        bi.start(); bi.byte(0x50 << 1); bi.byte(0x00); bi.byte(n & 0xFF)
        bi.start(); bi.byte(0x50 << 1 | 1); bi.byte(0xDE); bi.byte(0xAD, ack=False); bi.stop()
        u.write("pokeana %d\r\n" % n)
        can_send(rmt, dict(ident=0x123, data=[n & 0xFF, 0x22, 0x33]))
        n += 1
        time.sleep_ms(20)
elif MODE == "spi":
    cs = Pin(P3, Pin.OUT, value=1)
    n = 0
    while True:
        spi = SPI(1, baudrate=1000000, polarity=0, phase=0, sck=Pin(P0), mosi=Pin(P1), miso=Pin(21))
        cs(0); spi.write(bytes([0x9F, 0x00, 0x00, n & 0xFF])); cs(1)
        spi.deinit()
        for m in range(4):
            bit_spi(P0, P1, P2, P3, m, [0xA0 | m, 0x5A, n & 0xFF], [0x0F, 0xC3, (~n) & 0xFF])
        # an SPI flash, as seen by its master: JEDEC ID, status, read
        bit_spi(P0, P1, P2, P3, 0, [0x9F, 0, 0, 0], [0xFF, 0xEF, 0x40, 0x18])
        bit_spi(P0, P1, P2, P3, 0, [0x05, 0], [0xFF, 0x00])
        bit_spi(P0, P1, P2, P3, 0, [0x03, 0x00, 0x10, n & 0xFF, 0, 0, 0, 0], [0xFF] * 4 + [0xDE, 0xAD, 0xBE, 0xEF])
        Pin(P0, Pin.OUT, value=0)
        n += 1
        time.sleep_ms(20)
elif MODE == "uart":
    u1 = UART(1, baudrate=115200, tx=P0, rx=21)
    u2 = UART(2, baudrate=19200, bits=8, parity=0, stop=1, tx=P1, rx=38)     # Modbus RTU, 8E1

    def mb(*b):
        c = 0xFFFF
        for x in b:
            c ^= x
            for _ in range(8):
                c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
        return bytes(b) + bytes([c & 0xFF, c >> 8])
    n = 0
    while True:
        u1.write("Hello pokeana %d\r\n" % n)
        u2.write(mb(1, 3, 0, 0, 0, 2))                    # read 2 holding registers from 0
        time.sleep_ms(10)
        u2.write(mb(1, 3, 4, 0x12, 0x34, n >> 8 & 0xFF, n & 0xFF))
        time.sleep_ms(10)
        u2.write(mb(1, 6, 0, 1, 0, n & 0xFF))             # write register 1
        n += 1
        time.sleep_ms(20)
elif MODE == "can":
    r0 = can_rmt(0, P0, 500000)
    r1 = can_rmt(1, P1, 125000)
    frames = [dict(ident=0x123, data=[0x11, 0x22, 0x33]),
              dict(ident=0x18DAF110, data=[0x02, 0x10, 0x03], ext=True),
              dict(ident=0x7DF, data=[], rtr=True, dlc=8),
              dict(ident=0x0F0, data=[0xFF] * 8, ack=False)]
    n = 0
    while True:
        for f in frames:
            can_send(r0, f)
            time.sleep_us(50)
        can_send(r1, dict(ident=0x2A0 + (n & 0xF), data=[n & 0xFF]))
        n += 1
        time.sleep_ms(20)
'''


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "clk"
    port = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyACM0"
    s = serial.Serial(port, 115200, timeout=1)
    s.write(b"\r\x03\x03")
    time.sleep(0.5)
    s.read(5000)
    s.write(b"\x01")                                   # raw REPL
    time.sleep(0.3)
    s.read(5000)

    def run(code):
        s.write(code.encode() + b"\x04")
        out = b""
        t = time.time()
        while time.time() - t < 5 and out.count(b"\x04") < 2:
            out += s.read(s.in_waiting or 1)
        if b"Traceback" in out:
            raise SystemExit(out.decode(errors="replace"))
        return out

    run("f=open('main.py','w')")
    for k in range(0, len(MAIN), 400):
        run("f.write(%r)" % MAIN[k:k + 400])
    run("f.close();open('mode.txt','w').write(%r)" % mode)
    s.write(b"\x02")                                   # friendly REPL; Ctrl-D: soft reset, main.py runs
    time.sleep(0.2)
    s.write(b"\x04")
    time.sleep(1.5)
    out = s.read(s.in_waiting or 1).decode(errors="replace")
    if "Traceback" in out:
        raise SystemExit(out)
    print("mode %s running" % mode)


if __name__ == "__main__":
    main()
