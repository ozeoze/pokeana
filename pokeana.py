#!/usr/bin/env python3
"""Linux driver for the ULA-4G36-72M ("Pokeana 36", an Acute PKLA OEM) logic analyser,
from the USB traffic of the vendor's LA Viewer 1.90B07: no vendor code runs here.

The USB side is a Genesys bridge (05e3:013b):
  vendor request 0x04, value 0x82: announces the next bulk transfer
      (8 bytes: 01 00 82 00 len32 = host to device, 00 00 82 00 len32 = device to host);
  vendor request 0x0c, value 0x8e, index 0x20, IN 1 byte: ready (01), asked after each;
  vendor request 0x0c, value 0x83, OUT 1 byte: a bridge register (LA Viewer writes a5 f2
      f6 f0 a7 ab f2 a7 before a run, f2 a7 after a read);
  bulk OUT ep 2 and bulk IN ep 1 carry the analyser's own commands and data.

Each command of the analyser is a record "5a 00 0f 00 66 00" + 16-bit number + arguments:
  22 v32      sets the value the next register writes take
  0f          address: where writing starts, -pre (a run); where reading starts (a read)
  06          samples to record after the trigger
  24, 25      sample clock divider: 200 MHz / (n + 1)
  29, 0a      0 in every capture so far
  02 table    the trigger table (784 bytes, see trigger_table)
  21 codes    input thresholds (see set_thresholds), between 96 and 00 to the bridge
  01          start; 00 stop; 03 read (after 30); 04 status (36 bytes back, after 20 0)
  07 a b c    two serial lines (bits 7 and 6 of a), some 1460 bits: LA Viewer sends
              them before its first run; not decoded, sent again from the replay
Samples come back 8 bytes each, little endian: bit n is channel n.

The start-up (FPGA configuration, about 6.8 MB, and the front-end set-up) is not
decoded: it is sent again as LA Viewer sent it, from a replay file that
extract_replay.py pulls out of a capture (it holds the vendor's FPGA image, so it stays
out of any repository).

usage: pokeana.py [--rate HZ] [--depth N] [--pre N] [--trigger SPEC] [--threshold V]
                  [--replay FILE] [--channels N] [-o out.vcd]
  V: volts (-6 to 6) or ttl (1.6), ttl33 (1.49), ecl (-1.315); "V0,V1" for channels
  0-17 and 18-35 apart. Default ttl.
  SPEC: comma separated channel=cond, cond 0 1 r(ise) f(all) c(hange); "none"
  triggers at once. With no trigger in --timeout seconds, what is in memory is read
  (the trigger mark then sits at the end). The trigger is armed only once --pre
  samples are in memory (the analyser itself would take it from the first sample).
Output, by the name given to -o:
  .sr   sigrok session (PulseView, sigrok-cli): the sample rate exact, the trigger
        marked by an extra channel TRIG, 1 from the trigger on;
  else  VCD (GTKWave), time 0 at the trigger (sigrok's VCD input skips negative times).
--view opens the file (PulseView for .sr, GTKWave for VCD) when there is a display.
Needs read-write access to the USB device (root, or a udev rule for 05e3:013b).
"""
import argparse
import ctypes
import os
import struct
import sys
import time
import zipfile
import subprocess
import zlib

VID, PID = 0x05E3, 0x013B
EP_OUT, EP_IN = 0x02, 0x81
HDR = bytes.fromhex("5a000f006600")
BASE_CLOCK = 200_000_000
ADDR_MASK = (1 << 21) - 1          # 2 M samples of memory, a ring
LAG, STALE = 3, 5                  # see the read in main()
HERE = os.path.dirname(os.path.abspath(__file__))


class Usb:
    """The few libusb-1.0 calls needed, through ctypes (no pyusb on the work PC)."""

    def __init__(self):
        self.lib = lib = ctypes.CDLL("libusb-1.0.so.0")
        lib.libusb_open_device_with_vid_pid.restype = ctypes.c_void_p
        lib.libusb_open_device_with_vid_pid.argtypes = [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint16]
        lib.libusb_control_transfer.argtypes = [ctypes.c_void_p, ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint16,
                                                ctypes.c_uint16, ctypes.c_char_p, ctypes.c_uint16, ctypes.c_uint]
        lib.libusb_bulk_transfer.argtypes = [ctypes.c_void_p, ctypes.c_uint8, ctypes.c_char_p, ctypes.c_int,
                                             ctypes.POINTER(ctypes.c_int), ctypes.c_uint]
        for f in ("libusb_claim_interface", "libusb_release_interface", "libusb_set_configuration",
                  "libusb_kernel_driver_active", "libusb_detach_kernel_driver"):
            getattr(lib, f).argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.libusb_close.argtypes = [ctypes.c_void_p]
        self.ctx = ctypes.c_void_p()
        if lib.libusb_init(ctypes.byref(self.ctx)):
            raise OSError("libusb_init failed")
        self.h = lib.libusb_open_device_with_vid_pid(self.ctx, VID, PID)
        if not self.h:
            raise OSError("no %04x:%04x, or no access to it (root, or a udev rule)" % (VID, PID))
        if lib.libusb_kernel_driver_active(self.h, 0) == 1:
            lib.libusb_detach_kernel_driver(self.h, 0)
        # Set the configuration only when not yet set: setting it again resets the host's
        # data toggles but not the device's, and the first packet in was then lost (a
        # status read with no answer, every run but the first).
        cfg = ctypes.c_int(0)
        lib.libusb_get_configuration.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]
        if lib.libusb_get_configuration(self.h, ctypes.byref(cfg)) or cfg.value != 1:
            lib.libusb_set_configuration(self.h, 1)
        r = lib.libusb_claim_interface(self.h, 0)
        if r:
            raise OSError("claim interface: %d (in use by a VM or another program?)" % r)

    def close(self):
        self.lib.libusb_release_interface(self.h, 0)
        self.lib.libusb_close(self.h)

    def ctl(self, bm, req, val, idx, data=b"", length=0, timeout=2000):
        if bm & 0x80:
            buf = ctypes.create_string_buffer(length)
            n = self.lib.libusb_control_transfer(self.h, bm, req, val, idx, buf, length, timeout)
            if n < 0:
                raise OSError("control in %02x %04x: %d" % (req, val, n))
            return buf.raw[:n]
        n = self.lib.libusb_control_transfer(self.h, bm, req, val, idx, bytes(data), len(data), timeout)
        if n < 0:
            raise OSError("control out %02x %04x: %d" % (req, val, n))
        return n

    def bulk(self, ep, data=None, length=0, timeout=5000):
        done = ctypes.c_int()
        if ep & 0x80:
            buf = ctypes.create_string_buffer(length)
            r = self.lib.libusb_bulk_transfer(self.h, ep, buf, length, ctypes.byref(done), timeout)
            if r:
                raise OSError("bulk in: %d" % r)
            return buf.raw[:done.value]
        r = self.lib.libusb_bulk_transfer(self.h, ep, bytes(data), len(data), ctypes.byref(done), timeout)
        if r or done.value != len(data):
            raise OSError("bulk out: %d (%d of %d)" % (r, done.value, len(data)))
        return done.value


def rec(cmd, *words):
    return HDR + struct.pack("<%dH" % (1 + len(words)), cmd, *words)


def setv(v):
    return HDR + struct.pack("<HI", 0x22, v & 0xFFFFFFFF)


THRESHOLDS = {"ttl": 1.6, "ttl33": 1.49, "ecl": -1.315}     # LA Viewer's presets, volts


def threshold_code(volts):
    """The 8-bit DAC code of a threshold: 13 is -6 V, 235 is +6 V, in a straight line
    (LA Viewer: 0x99 TTL 1.6 V, 0x97 TTL 3.3 1.49 V, 0x64 ECL -1.315 V)."""
    if not -6 <= volts <= 6:
        raise ValueError("threshold %g V: -6 to 6 V" % volts)
    return round(13 + (volts + 6) * 222 / 12)


def set_thresholds(v0, v1):
    """Command 21: 32 bytes, the codes of channels 0-17 and 18-35, twice, then 0."""
    a, b = threshold_code(v0), threshold_code(v1)
    return HDR + struct.pack("<H", 0x21) + bytes([a, b, a, b]) + bytes(28)


class Pokeana:
    def __init__(self):
        self.usb = Usb()
        self.drain()

    def drain(self):
        """Reads and drops what the device still holds for ep 1: after a read it can keep
        a packet back, which the next status read would take for its own."""
        for _ in range(64):
            try:
                self.usb.bulk(EP_IN, length=512, timeout=30)
            except OSError:
                return

    def ready(self):
        return self.usb.ctl(0xC0, 0x0C, 0x8E, 0x20, length=1)

    def bridge(self, b):
        self.usb.ctl(0x40, 0x0C, 0x83, 0, bytes([b]))
        self.ready()

    def send(self, payload):
        self.usb.ctl(0x40, 0x04, 0x82, 0, struct.pack("<BBBBI", 1, 0, 0x82, 0, len(payload)))
        self.ready()
        self.usb.bulk(EP_OUT, payload)

    def recv(self, n):
        self.usb.ctl(0x40, 0x04, 0x82, 0, struct.pack("<BBBBI", 0, 0, 0x82, 0, n))
        self.ready()
        out = b""
        while len(out) < n:
            chunk = self.usb.bulk(EP_IN, length=max(512, (n - len(out) + 511) // 512 * 512))
            if not chunk:
                continue
            out += chunk
        return out[:n]

    def status(self):
        for attempt in range(4):
            try:
                self.send(rec(0x20, 0) + rec(0x04))
                s = self.recv(36)
                break
            except OSError:
                if attempt == 3:
                    raise
                print("status: no answer, asked again", file=sys.stderr)
                self.drain()
        return dict(flags=s[0], addr=s[2] | s[3] << 8 | s[4] << 16,
                    left=struct.unpack("<I", s[6:10])[0] & 0x3FFFFF, live=struct.unpack("<Q", s[10:18])[0] & ((1 << 36) - 1),
                    raw=s)

    def replay(self, path, phase):
        """Sends one phase (init or prelude) of a replay file as LA Viewer sent it."""
        for line in zlib.decompress(open(path, "rb").read()).decode().split("\n"):
            f = line.split()
            if f[0] != phase:
                continue
            if f[1] == "C":
                bm, req, val, idx, ln = map(int, f[2:7])
                if bm & 0x80:
                    self.usb.ctl(bm, req, val, idx, length=ln)
                else:
                    self.usb.ctl(bm, req, val, idx, bytes.fromhex(f[7] if len(f) > 7 else ""))
            else:
                ep, ln = int(f[2]), int(f[3])
                if ep & 0x80:
                    try:
                        self.usb.bulk(ep, length=ln, timeout=1000)
                    except OSError:
                        pass                     # nothing to read back this time
                elif phase == "prelude" and ln == 792:
                    continue                     # the trigger table: ours goes in run()
                else:
                    self.usb.bulk(ep, bytes.fromhex(f[4]))

    def run(self, divider, pre, post, table):
        for b in bytes.fromhex("a5f2f6f0a7abf2a7"):
            self.bridge(b)
        self.send(table)
        self.send(rec(0x23, 0))
        for b in (0xF2, 0xA7):
            self.bridge(b)
        self.send(setv(-pre) + rec(0x0F, 0) + setv(post) + rec(0x06, 0) + setv(divider) + rec(0x24, 0)
                  + rec(0x25, 0) + setv(0) + rec(0x29, 0) + setv(0) + rec(0x0A, 0) + rec(0x01))

    def stop(self):
        self.send(rec(0x00))
        return self.status()

    def read(self, start, count):
        self.send(setv(0) + rec(0x0A, 0) + rec(0x24, 0) + rec(0x06, 0) + rec(0x25, 0)
                  + setv(start & ADDR_MASK) + rec(0x0F, 0) + rec(0x30) + rec(0x03))
        data = self.recv(count * 8)
        for b in (0xF2, 0xA7):
            self.bridge(b)
        self.drain()
        return [struct.unpack_from("<Q", data, 8 * i)[0] for i in range(count)]

    def capture(self, rate, depth, pre, conds, thresholds, timeout=None, init=False,
                replay=os.path.join(HERE, "replay_05.bin"), cancel=None,
                log=lambda m: print(m, file=sys.stderr)):
        """One acquisition: returns (samples, sample rate, index of the trigger).
        conds: {channel: "0" "1" "r" "f" "c"}; thresholds: (volts 0-17, volts 18-35).
        With no trigger by timeout seconds (None: none), or cancel() true, what is in
        memory is read (the trigger index then at the end)."""
        divider = max(0, round(BASE_CLOCK / rate) - 1)
        rate = BASE_CLOCK / (divider + 1)
        pre = max(0, min(pre, depth - 1))
        post = depth - pre
        if init:
            t = time.time()
            self.replay(replay, "init")
            log("start-up sent (%.1f s)" % (time.time() - t))
        self.replay(replay, "prelude")
        self.bridge(0x96)                       # as LA Viewer: 96 to the bridge around 21
        self.send(set_thresholds(*thresholds))
        self.bridge(0x00)
        # The analyser takes a trigger from the first sample of a run: it runs with a
        # table that never fires until the samples before the trigger are written, then
        # gets the real one (a table can be sent while running).
        self.run(divider, pre, post, trigger_table(conds, armed=False))
        t_run = time.time()
        time.sleep((pre + 64) / rate + 0.002)
        early = self.status()["flags"] & 1
        self.send(trigger_table(conds))
        t0 = time.time()
        while True:
            s = self.status()
            if s["flags"] & 1:                  # triggered (at once with no condition) and done
                break
            if timeout is not None and time.time() - t0 > timeout:
                log("no trigger in %g s: reading what is there" % timeout)
                break
            if cancel and cancel():
                log("stopped: reading what is there")
                break
            time.sleep(0.005)
        done = s["flags"] & 1
        s = self.stop()
        end = s["addr"]
        # The samples land in memory LAG samples late, and the first words of a read are
        # stale: the window is moved back by LAG and read with STALE words more.
        # Not done: what this run wrote, from -pre to end (all of the ring once it went round).
        written = ((end - LAG - STALE - ((-pre) & ADDR_MASK)) & ADDR_MASK
                   if (time.time() - t_run) * rate < ADDR_MASK else depth)
        count = depth if done else max(0, min(depth, written))
        start = end - LAG - count
        samples = self.read(start - STALE, count + STALE)[STALE:] if count else []
        trig = count - post if done else count
        # Writing starts at -pre: should the trigger still have come early, the memory
        # before the first address written holds an older capture, dropped here.
        fresh = (((-pre) & ADDR_MASK) - start + LAG) & ADDR_MASK
        if early and 0 < fresh < count:
            log("only %d samples before the trigger are from this run" % (trig - fresh))
            samples, trig = samples[fresh:], trig - fresh
        log("status %02x end %06x, %d samples at %g Hz, trigger at %d" % (s["flags"], end, len(samples), rate, trig))
        return samples, rate, trig


# The trigger table: 784 bytes after the "02" record. Two comparators, each a pair of
# patterns (the sample before, the sample now) with a mask; bit n of each 6-byte field
# is channel n. A level sets both patterns; an edge sets "before" and "now" apart; a
# change uses one comparator for the rise and the other for the fall.
T_PREV1, T_NOW1, T_PREV2, T_NOW2 = 92, 98, 188, 194
M_PREV1, M_NOW1, M_PREV2, M_NOW2 = 476, 482, 572, 578


def trigger_table(conds, armed=True):
    """Byte 779: bit 7 lets the table fire, bits 5..4 keep the edge detector going
    (with them off, the first sample after arming compares with a stale one); byte 783
    likewise for the second pair of patterns ("change"). armed=False: 30, the table runs
    but never fires (so until the samples before the trigger are in memory)."""
    t = bytearray(792)
    t[:8] = rec(0x02)
    edge = any(c in "rfc" for c in conds.values())
    t[779] = (0xF0 if edge else 0xC0) if armed else 0x30
    if any(c == "c" for c in conds.values()):
        t[783] = 0xF0 if armed else 0x30

    def put(off, ch):
        t[off + ch // 8] |= 1 << (ch % 8)

    for ch, c in conds.items():
        for off in (M_PREV1, M_NOW1, M_PREV2, M_NOW2):
            put(off, ch)
        on = {"1": (T_PREV1, T_NOW1, T_PREV2, T_NOW2), "0": (), "r": (T_NOW1, T_NOW2),
              "f": (T_PREV1, T_PREV2), "c": (T_NOW1, T_PREV2)}[c]
        for off in on:
            put(off, ch)
    return bytes(t)


def write_vcd(path, samples, rate, channels, trigger_index):
    tick = 1e12 / rate                       # ps per sample
    with open(path, "w") as f:
        f.write("$comment ULA-4G36-72M, %d samples at %g Hz, trigger at sample %d $end\n"
                % (len(samples), rate, trigger_index))
        f.write("$timescale 1 ps $end\n$scope module la $end\n")
        ids = [chr(33 + i) for i in range(channels)]
        for i in range(channels):
            f.write("$var wire 1 %s CH%02d $end\n" % (ids[i], i))
        f.write("$upscope $end\n$enddefinitions $end\n")
        last = None
        for n, s in enumerate(samples):
            if s != last:
                f.write("#%d\n" % round((n - trigger_index) * tick))
                for i in range(channels):
                    b = s >> i & 1
                    if last is None or b != last >> i & 1:
                        f.write("%d%s\n" % (b, ids[i]))
                last = s
        f.write("#%d\n" % round((len(samples) - trigger_index) * tick))


def write_sr(path, samples, rate, channels, trigger_index):
    """A sigrok session file (version 2): metadata and the samples, 5 bytes each (36
    channels and TRIG)."""
    trig_bit = 1 << channels
    data = b"".join((v | (trig_bit if n >= trigger_index else 0)).to_bytes(5, "little")
                    for n, v in enumerate(samples))
    meta = ["[global]", "sigrok version=0.5.2", "", "[device 1]", "capturefile=logic-1",
            "total probes=%d" % (channels + 1), "samplerate=%d Hz" % round(rate),
            "total analog=0"]
    meta += ["probe%d=CH%02d" % (i + 1, i) for i in range(channels)]
    meta += ["probe%d=TRIG" % (channels + 1), "unitsize=5", ""]
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("version", "2")
        z.writestr("metadata", "\n".join(meta))
        z.writestr("logic-1-1", data)


def view(path):
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        print("--view: no display here (log in to the desktop, or ssh -X)", file=sys.stderr)
        return
    cmd = ["pulseview", "-i", path] if path.endswith(".sr") else ["gtkwave", path]
    subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--rate", type=float, default=2e6, help="sample rate, Hz (200 MHz / n)")
    ap.add_argument("--depth", type=int, default=2500, help="samples in all")
    ap.add_argument("--pre", type=int, default=None, help="samples before the trigger (default depth/25)")
    ap.add_argument("--trigger", default="none")
    ap.add_argument("--threshold", default="ttl",
                    help="volts or ttl/ttl33/ecl; two (comma) for channels 0-17 and 18-35")
    ap.add_argument("--channels", type=int, default=36)
    ap.add_argument("--replay", default=os.path.join(HERE, "replay_05.bin"))
    ap.add_argument("--timeout", type=float, default=10)
    ap.add_argument("--skip-init", action="store_true", help="the FPGA is loaded already")
    ap.add_argument("-o", "--out", default="capture.sr", help=".sr (sigrok) or VCD")
    ap.add_argument("--view", action="store_true", help="open the file when done")
    a = ap.parse_args()

    th = [THRESHOLDS[v] if v in THRESHOLDS else float(v) for v in a.threshold.lower().split(",")]
    th = (th * 2)[:2]
    for v in th:
        threshold_code(v)                       # range check before touching the device
    rate = a.rate
    pre = a.pre if a.pre is not None else max(16, a.depth // 25)
    conds = {}
    if a.trigger != "none":
        for item in a.trigger.split(","):
            ch, c = item.split("=")
            conds[int(ch)] = c[0]

    la = Pokeana()
    try:
        samples, rate, trig = la.capture(rate, a.depth, pre, conds, th, timeout=a.timeout,
                                         init=not a.skip_init, replay=a.replay)
    finally:
        la.usb.close()
    if a.channels > 39:
        raise SystemExit("--channels: 39 at most")
    (write_sr if a.out.endswith(".sr") else write_vcd)(a.out, samples, rate, a.channels, trig)
    print(a.out)
    if a.view:
        view(a.out)


if __name__ == "__main__":
    main()
