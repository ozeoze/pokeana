#!/usr/bin/env python3
"""GUI for the ULA-4G36-72M logic analyser (GTK 4): the settings, Run / Repeat / Stop,
and the waveforms, on top of pokeana.py.

usage: pokeana_gui.py [file.sr]     (a file: opened for viewing; the device is used on Run)

Waveforms: the wheel zooms about the mouse, shift + wheel or a drag moves; a left click
puts cursor A, a right click cursor B. A click on a channel name steps its trigger
condition (none, rise, fall, change, 1, 0); a right click there clears it. Time 0 is
the trigger. F fits the whole capture.

Protocol decoders (decode.py: I2C, SPI, UART, CAN): "解析を追加" adds one; its rows
go below the channels, and a click on a row's name edits or removes it. "一覧" shows
the decoded messages; selecting one shows it in the waveforms.
"""
import bisect
import json
import os
import sys
import tempfile
import threading
import time

import gi
gi.require_version("Gtk", "4.0")
gi.require_version("Gdk", "4.0")
gi.require_foreign("cairo")       # the draw function's context (python3-gi-cairo)
from gi.repository import Gdk, Gio, GLib, Gtk  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import decode as D  # noqa: E402
import pokeana as P  # noqa: E402
from decode import Capture, load_sr  # noqa: E402

CHANNELS = 36
RATES = [200e6, 100e6, 50e6, 40e6, 25e6, 20e6, 10e6, 5e6, 2e6, 1e6, 500e3, 200e3, 100e3,
         50e3, 20e3, 10e3, 5e3, 2e3, 1e3]
DEPTHS = [1000, 2500, 10000, 25000, 100000, 250000, 1000000, 2000000]
THRESHOLD_PRESETS = [("TTL", 1.6), ("TTL 3.3", 1.49), ("ECL", -1.315), ("任意", None)]
CONDS = ["", "r", "f", "c", "1", "0"]
COND_TEXT = {"": "", "r": "↑", "f": "↓", "c": "↕", "1": "1", "0": "0"}
NAME_W, HEAD_H = 112, 26
# decoder boxes, by kind: fill, edge
KIND_COL = {"ctl": (0.55, 0.45, 0.85), "addr": (0.95, 0.65, 0.25), "data": (0.3, 0.6, 0.95),
            "ack": (0.35, 0.75, 0.45), "nack": (0.9, 0.55, 0.3), "err": (0.9, 0.3, 0.3),
            "info": (0.6, 0.6, 0.65)}
DEC_BG = (0.13, 0.13, 0.2)
ROLE = (0.6, 0.75, 0.95)
PLACES = ["解析を隠す", "波形に重ねる", "信号のすぐ下", "いちばん下"]
HIDE, ON_LINE, UNDER, BOTTOM = range(4)
LAYER_TITLE = {"eeprom": "EEPROM", "regs": "I2C", "flash": "Flash", "modbus": "Modbus"}
LOG_MAX = 5000                              # lines in the side list
BG, GRID, TRACE, TRIG, CUR_A, CUR_B, TEXT = ((0.07, 0.07, 0.08), (0.22, 0.22, 0.25), (0.35, 0.9, 0.45),
                                              (0.95, 0.3, 0.3), (0.95, 0.85, 0.3), (0.35, 0.8, 0.95),
                                              (0.85, 0.85, 0.85))


def fmt_hz(f):
    for unit, k in (("GHz", 1e9), ("MHz", 1e6), ("kHz", 1e3)):
        if f >= k:
            return ("%.4g " % (f / k)) + unit
    return "%.4g Hz" % f


def fmt_time(t):
    a = abs(t)
    for unit, k in (("s", 1), ("ms", 1e-3), ("µs", 1e-6), ("ns", 1e-9)):
        if a >= k or unit == "ns":
            return ("%.4g " % (t / k)) + unit


def nice_step(x):
    """The 1-2-5 step at or above x."""
    e = 10 ** int(f"{x:e}".split("e")[1])
    for m in (1, 2, 5, 10):
        if m * e >= x:
            return m * e


class Window(Gtk.ApplicationWindow):
    def __init__(self, app, path=None):
        super().__init__(application=app, title="ポケアナ")
        self.set_default_size(1280, 820)
        self.cap = None
        self.la = None
        self.inited = False
        self.busy = False
        self.cancel = False
        self.conds = {}
        self.t0, self.spp = 0.0, 1.0
        self.cur_a = self.cur_b = None
        self.mouse = None
        self.drag_t0 = None
        self.need_fit = False

        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        self.set_child(box)
        bar = Gtk.Box(spacing=6, margin_top=6, margin_bottom=6, margin_start=6, margin_end=6)
        box.append(bar)

        self.run_btn = Gtk.Button(label="Run")
        self.run_btn.connect("clicked", lambda *_: self.start())
        self.repeat_btn = Gtk.ToggleButton(label="連続")
        self.stop_btn = Gtk.Button(label="Stop", sensitive=False)
        self.stop_btn.connect("clicked", lambda *_: self.stop())
        for w in (self.run_btn, self.repeat_btn, self.stop_btn):
            bar.append(w)
        bar.append(Gtk.Separator(orientation=Gtk.Orientation.VERTICAL))

        bar.append(Gtk.Label(label="速さ"))
        self.rate_dd = Gtk.DropDown.new_from_strings([fmt_hz(r) for r in RATES])
        self.rate_dd.set_selected(RATES.index(20e6))
        bar.append(self.rate_dd)
        bar.append(Gtk.Label(label="サンプル数"))
        self.depth_dd = Gtk.DropDown.new_from_strings(["%d" % d for d in DEPTHS])
        self.depth_dd.set_selected(DEPTHS.index(100000))
        bar.append(self.depth_dd)
        bar.append(Gtk.Label(label="トリガ前 %"))
        self.pre_spin = Gtk.SpinButton.new_with_range(0, 99, 1)
        self.pre_spin.set_value(10)
        bar.append(self.pre_spin)
        bar.append(Gtk.Separator(orientation=Gtk.Orientation.VERTICAL))

        self.th = []
        for label in ("CH0-17", "CH18-35"):
            bar.append(Gtk.Label(label=label))
            dd = Gtk.DropDown.new_from_strings([n for n, _ in THRESHOLD_PRESETS])
            spin = Gtk.SpinButton.new_with_range(-6, 6, 0.05)
            spin.set_digits(2)
            spin.set_value(1.6)
            spin.set_sensitive(False)
            dd.connect("notify::selected", self.on_threshold_preset, spin)
            bar.append(dd)
            bar.append(spin)
            self.th.append((dd, spin))
        bar.append(Gtk.Separator(orientation=Gtk.Orientation.VERTICAL))

        bar.append(Gtk.Label(label="表示"))
        self.show_spin = Gtk.SpinButton.new_with_range(1, CHANNELS, 1)
        self.show_spin.set_value(8)
        self.show_spin.connect("value-changed", lambda *_: self.area.queue_draw())
        bar.append(self.show_spin)
        fit = Gtk.Button(label="全体")
        fit.connect("clicked", lambda *_: self.fit())
        bar.append(fit)
        save = Gtk.Button(label="保存")
        save.connect("clicked", lambda *_: self.save())
        bar.append(save)
        pv = Gtk.Button(label="PulseView")
        pv.connect("clicked", lambda *_: self.pulseview())
        bar.append(pv)
        bar.append(Gtk.Separator(orientation=Gtk.Orientation.VERTICAL))
        dec = Gtk.Button(label="解析を追加")
        dec.connect("clicked", lambda *_: self.decoder_dialog(None))
        bar.append(dec)
        self.place_dd = Gtk.DropDown.new_from_strings(PLACES)
        self.place_dd.set_tooltip_text("解析の結果を出す所: 隠す（右の一覧だけ） / データの線（SDA、MOSI、RX など）に重ねる / "
                                       "その線のすぐ下 / いちばん下にまとめる")
        self.place_dd.connect("notify::selected", lambda *_: self.area.queue_draw())
        bar.append(self.place_dd)
        self.list_btn = Gtk.ToggleButton(label="一覧")
        self.list_btn.connect("toggled", lambda b: self.side.set_visible(b.get_active()))
        bar.append(self.list_btn)

        self.area = Gtk.DrawingArea(hexpand=True, vexpand=True, focusable=True)
        self.area.set_draw_func(self.draw)
        self.decs = []                          # [{"name", "opts", "res", "rows"}]
        self.log_entries = []
        self.loglist = Gtk.ListBox(selection_mode=Gtk.SelectionMode.SINGLE)
        self.loglist.connect("row-activated", lambda _l, row: self.goto(row.entry))
        self.loglist.connect("row-selected", lambda _l, row: row and self.goto(row.entry))
        sw = Gtk.ScrolledWindow(vexpand=True, hscrollbar_policy=Gtk.PolicyType.NEVER)
        sw.set_child(self.loglist)
        self.side = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, width_request=380)
        self.side.append(sw)
        self.side.set_visible(False)
        paned = Gtk.Paned(orientation=Gtk.Orientation.HORIZONTAL, vexpand=True)
        paned.set_start_child(self.area)
        paned.set_end_child(self.side)
        paned.set_resize_end_child(False)
        paned.set_shrink_end_child(False)
        box.append(paned)
        self.status = Gtk.Label(xalign=0, margin_start=8, margin_end=8, margin_top=4, margin_bottom=4)
        self.info = Gtk.Label(xalign=1, margin_start=8, margin_end=8, hexpand=True)
        sb = Gtk.Box()
        sb.append(self.status)
        sb.append(self.info)
        box.append(sb)

        scroll = Gtk.EventControllerScroll.new(Gtk.EventControllerScrollFlags.BOTH_AXES)
        scroll.connect("scroll", self.on_scroll)
        self.area.add_controller(scroll)
        motion = Gtk.EventControllerMotion()
        motion.connect("motion", self.on_motion)
        motion.connect("leave", lambda *_: self.set_mouse(None))
        self.area.add_controller(motion)
        drag = Gtk.GestureDrag(button=1)
        drag.connect("drag-begin", self.on_drag_begin)
        drag.connect("drag-update", self.on_drag_update)
        drag.connect("drag-end", self.on_drag_end)
        self.area.add_controller(drag)
        right = Gtk.GestureClick(button=3)
        right.connect("pressed", self.on_right)
        self.area.add_controller(right)
        keys = Gtk.EventControllerKey()
        keys.connect("key-pressed", self.on_key)
        self.add_controller(keys)

        self.say("チャンネル名をクリックするとトリガ条件が変わります。Run で取り込みます。")
        if path:
            self.set_capture(Capture(*load_sr(path)))
            self.say("%s を開きました" % path)

    # ---- settings ----

    def on_threshold_preset(self, dd, _pspec, spin):
        v = THRESHOLD_PRESETS[dd.get_selected()][1]
        spin.set_sensitive(v is None)
        if v is not None:
            spin.set_value(v)

    def settings(self):
        rate = RATES[self.rate_dd.get_selected()]
        depth = DEPTHS[self.depth_dd.get_selected()]
        pre = int(depth * self.pre_spin.get_value() / 100)
        th = tuple(spin.get_value() for _, spin in self.th)
        return rate, depth, pre, dict(self.conds), th

    # ---- acquisition ----

    def say(self, text):
        self.status.set_text(text)

    def start(self):
        if self.busy:
            return
        self.busy, self.cancel = True, False
        self.run_btn.set_sensitive(False)
        self.stop_btn.set_sensitive(True)
        args = self.settings()
        cond_text = ", ".join("CH%02d %s" % (c, COND_TEXT[v]) for c, v in sorted(args[3].items())) or "なし（すぐ）"
        self.say("トリガ待ち: %s" % cond_text)
        threading.Thread(target=self.worker, args=args, daemon=True).start()

    def stop(self):
        self.cancel = True
        self.repeat_btn.set_active(False)

    def worker(self, rate, depth, pre, conds, th):
        try:
            if self.la is None:
                self.la = P.Pokeana()
            t = time.time()
            samples, rate, trig = self.la.capture(rate, depth, pre, conds, th, init=not self.inited,
                                                  cancel=lambda: self.cancel,
                                                  log=lambda m: GLib.idle_add(self.say, m))
            self.inited = True
            cap = Capture(samples, rate, trig)
            GLib.idle_add(self.done, cap, time.time() - t, None)
        except Exception as e:                   # noqa: BLE001  (shown, and the device let go)
            if self.la is not None:
                try:
                    self.la.usb.close()
                except Exception:                # noqa: BLE001
                    pass
            self.la, self.inited = None, False
            GLib.idle_add(self.done, None, 0, e)

    def done(self, cap, secs, err):
        self.busy = False
        self.run_btn.set_sensitive(True)
        self.stop_btn.set_sensitive(False)
        if err is not None:
            self.say("エラー: %s" % err)
            self.repeat_btn.set_active(False)
            return False
        keep = self.cap is not None and len(self.cap) == len(cap) and self.cap.rate == cap.rate
        self.set_capture(cap, keep_view=keep)
        self.say("%d サンプル、%s、トリガは %d サンプル目（%.2f 秒）" % (len(cap), fmt_hz(cap.rate), cap.trig, secs))
        if self.repeat_btn.get_active() and not self.cancel:
            GLib.timeout_add(100, lambda: (self.start(), False)[1])
        return False

    def set_capture(self, cap, keep_view=False):
        self.cap = cap
        if self.decs:
            self.run_decoders()
            self.dump()
        if not keep_view:
            self.cur_a = self.cur_b = None
            self.fit()
        self.area.queue_draw()
        self.update_info()

    # ---- view ----

    def plot_w(self):
        return max(1, self.area.get_width() - NAME_W)

    def fit(self):
        self.need_fit = True                    # done in draw(), when the width is known
        self.area.queue_draw()

    def clamp(self):
        if not self.cap:
            return
        n = len(self.cap)
        self.spp = min(max(self.spp, 1 / 64), max(n / self.plot_w() * 1.2, 1 / 64))
        self.t0 = min(max(self.t0, -0.1 * self.plot_w() * self.spp), n - 0.9 * self.plot_w() * self.spp)

    def x_to_sample(self, x):
        return self.t0 + (x - NAME_W) * self.spp

    def sample_to_x(self, i):
        return NAME_W + (i - self.t0) / self.spp

    def on_scroll(self, ctl, dx, dy):
        if not self.cap:
            return True
        state = ctl.get_current_event_state()
        if dx or state & Gdk.ModifierType.SHIFT_MASK:
            self.t0 += (dx or dy) * 60 * self.spp
        else:
            x = self.mouse[0] if self.mouse else NAME_W + self.plot_w() / 2
            at = self.x_to_sample(x)
            self.spp *= 1.25 ** dy
            self.clamp()
            self.t0 = at - (x - NAME_W) * self.spp
        self.clamp()
        self.area.queue_draw()
        return True

    def set_mouse(self, pos):
        self.mouse = pos
        self.update_info()

    def on_motion(self, _ctl, x, y):
        self.set_mouse((x, y))

    def on_drag_begin(self, _g, x, y):
        self.drag_t0 = self.t0
        self.area.grab_focus()

    def on_drag_update(self, _g, dx, dy):
        if self.cap and abs(dx) > 3:
            self.t0 = self.drag_t0 - dx * self.spp
            self.clamp()
            self.area.queue_draw()

    def on_drag_end(self, g, dx, dy):
        if abs(dx) > 3 or abs(dy) > 3:
            return
        ok, x, y = g.get_start_point()
        if x < NAME_W:
            e = self.row_at(y)
            di = self.decoder_on(e[1]) if e and e[0] == "ch" else None
            if di is not None and 44 <= x < NAME_W - 20:          # the role (SDA ...): its decoder
                self.decoder_dialog(di)
            elif e and e[0] == "ch":
                ch = e[1]
                c = CONDS[(CONDS.index(self.conds.get(ch, "")) + 1) % len(CONDS)]
                if c:
                    self.conds[ch] = c
                else:
                    self.conds.pop(ch, None)
            elif e:
                self.decoder_dialog(e[1])
        elif self.cap:
            self.cur_a = round(self.x_to_sample(x))
        self.area.queue_draw()
        self.update_info()

    def on_right(self, _g, _n, x, y):
        if x < NAME_W:
            e = self.row_at(y)
            if e and e[0] == "ch":
                self.conds.pop(e[1], None)
        elif self.cap:
            self.cur_b = round(self.x_to_sample(x))
        self.area.queue_draw()
        self.update_info()

    def on_key(self, _ctl, keyval, _code, _state):
        name = Gdk.keyval_name(keyval) or ""
        if name in ("f", "F"):
            self.fit()
        elif name == "space":
            self.start() if not self.busy else self.stop()
        else:
            return False
        return True

    def decoder_on(self, ch):
        """The first decoder that reads channel ch, or None."""
        for di, d in enumerate(self.decs):
            if ch in self.dec_chs(di):
                return di
        return None

    def dec_chs(self, di):
        """The channels a decoder reads, in option order."""
        d = self.decs[di]
        out = []
        for k, _, kind, _, _ in D.DECODERS[d["name"]][2]:
            v = d["opts"].get(k)
            if kind in ("ch", "ch?") and v is not None:
                out.append(v)
            elif kind in ("chs", "chs?") and v:
                out += list(v)
        return out

    def overlay_ch(self, di, ri):
        """The data line a decoder row belongs on (SDA, MOSI / MISO, RX), or None."""
        d = self.decs[di]
        name = d["res"].rows[ri][0] if "res" in d else ""
        o = d["opts"]
        if d["name"] == "spi":
            return o.get("mosi") if name.endswith("MOSI") else o.get("miso") if name.endswith("MISO") else None
        if d["name"] == "parallel":      # on the strobe (CS, then WR, RD, clock), else the lowest data line
            return next((o[k] for k in ("cs", "wr", "rd", "clk") if o.get(k) is not None), o["data"][0])
        if ri == 0:
            return o.get({"i2c": "sda", "uart": "rx", "can": "rx"}[d["name"]])
        return None

    def layout(self):
        """The rows from the top and their height: ("ch", channel, [(decoder, row) drawn on it])
        or ("dec", decoder, row). 解析を隠す: none (the list still has them); 波形に重ねる: a
        decoder's rows go on its data line and the rest just under it; 信号のすぐ下: all of
        them just under that line; いちばん下 (or when the line is hidden): at the bottom."""
        show = int(self.show_spin.get_value())
        place = self.place_dd.get_selected()
        if place == HIDE:
            ent = [("ch", ch, []) for ch in range(show)]
            return ent, max(12.0, (self.area.get_height() - HEAD_H) / max(1, len(ent)))
        over, after, bottom = {}, {}, []
        for di, d in enumerate(self.decs):
            rows = range(len(d["res"].rows)) if "res" in d else []
            on = {ri: self.overlay_ch(di, ri) for ri in rows}
            on = {ri: ch for ri, ch in on.items() if ch is not None and ch < show and place != BOTTOM}
            if place == ON_LINE:
                for ri, ch in on.items():
                    over.setdefault(ch, []).append((di, ri))
            for ri in rows:
                if place == UNDER and ri in on:         # just under its own line (MOSI, MISO ...)
                    after.setdefault(on[ri], []).append(("dec", di, ri))
                elif place == UNDER and on:
                    after.setdefault(max(on.values()), []).append(("dec", di, ri))
                elif ri not in on or place != ON_LINE:
                    (after.setdefault(max(on.values()), []) if on else bottom).append(("dec", di, ri))
        ent = []
        for ch in range(show):
            ent.append(("ch", ch, over.get(ch, [])))
            ent += after.get(ch, [])
        ent += bottom
        h = max(12.0, (self.area.get_height() - HEAD_H) / max(1, len(ent)))
        return ent, h

    def row_at(self, y):
        ent, h = self.layout()
        r = int((y - HEAD_H) // h)
        return ent[r] if 0 <= r < len(ent) and y >= HEAD_H else None

    def update_info(self):
        cap = self.cap
        if not cap:
            self.info.set_text("")
            return
        tt = lambda i: fmt_time((i - cap.trig) / cap.rate)  # noqa: E731
        parts = []
        if self.mouse and self.mouse[0] >= NAME_W:
            parts.append("マウス %s" % tt(self.x_to_sample(self.mouse[0])))
        if self.cur_a is not None:
            parts.append("A %s" % tt(self.cur_a))
        if self.cur_b is not None:
            parts.append("B %s" % tt(self.cur_b))
        if self.cur_a is not None and self.cur_b is not None and self.cur_a != self.cur_b:
            d = (self.cur_b - self.cur_a) / cap.rate
            parts.append("B−A %s（%s）" % (fmt_time(d), fmt_hz(1 / abs(d))))
        self.info.set_text("   ".join(parts))

    # ---- protocol decoders ----

    def roles(self):
        """channel -> the names it has in the decoders (SCL, MOSI, ...)."""
        out = {}
        for d in self.decs:
            for k, _, kind, _, _ in D.DECODERS[d["name"]][2]:
                ch = d["opts"].get(k)
                if kind in ("ch", "ch?") and ch is not None:
                    out.setdefault(ch, []).append(D.DECODERS[d["name"]][0] if k == "rx" else k.upper())
                elif kind in ("chs", "chs?") and ch:
                    for n, c in enumerate(ch):          # data -> D0, D1 ...; addr -> A0 ...
                        out.setdefault(c, []).append("%s%d" % (k[0].upper(), n))
        return {ch: "/".join(v) for ch, v in out.items()}

    def dec_tag(self, di):
        """The channels a decoder reads, to tell buses apart: "2/3", "10-13", "5"."""
        chs = self.dec_chs(di)
        if len(chs) > 4:                                # a parallel bus: "0-12+18-24"
            return D.ch_ranges(sorted(set(chs)))
        if len(chs) >= 3 and chs == list(range(chs[0], chs[0] + len(chs))):
            return "%d-%d" % (chs[0], chs[-1])
        return "/".join(str(c) for c in chs)

    def row_name(self, di, ri):
        """The row's name and its channel tag (I2C, 2/3)."""
        return self.decs[di]["res"].rows[ri][0], self.dec_tag(di)

    def add_decoder(self, spec):
        name, opts = D.parse_spec(spec)
        self.decs.append({"name": name, "opts": opts})
        self.run_decoders()

    def run_decoders(self):
        cap = self.cap if self.cap else Capture([], 1e6, 0)
        t = time.time()
        for d in self.decs:
            res = D.run(cap, d["name"], d["opts"])
            d["rows"] = []
            for _, anns in res.rows:
                anns.sort(key=lambda a: a.start)
                d["rows"].append(([a.start for a in anns], max((a.end - a.start for a in anns), default=0)))
            d["res"] = res
        self.rebuild_log()
        self.area.queue_draw()
        return time.time() - t

    def rebuild_log(self):
        self.loglist.remove_all()
        entries = []
        for di, d in enumerate(self.decs):
            title = LAYER_TITLE.get(d["opts"].get("layer"), D.DECODERS[d["name"]][0])
            title += " " + self.dec_tag(di)
            entries += [(a, b, title, text) for a, b, text in d["res"].log]
        entries.sort(key=lambda e: e[0])
        self.log_entries = entries
        cap = self.cap
        for e in entries[:LOG_MAX]:
            row = Gtk.ListBoxRow()
            row.entry = e
            t = fmt_time((e[0] - cap.trig) / cap.rate) if cap else ""
            lab = Gtk.Label(xalign=0, wrap=True, margin_start=6, margin_end=6, margin_top=2, margin_bottom=2)
            lab.set_markup("<tt><span foreground='#8ab'>%s</span> <b>%s</b> %s</tt>"
                           % (GLib.markup_escape_text("%10s" % t), e[2], GLib.markup_escape_text(e[3])))
            row.set_child(lab)
            self.loglist.append(row)
        if len(entries) > LOG_MAX:
            self.loglist.append(Gtk.Label(label="（ほか %d 件）" % (len(entries) - LOG_MAX)))

    def goto(self, e):
        """Shows the samples of a log entry, in the middle, at a size to read."""
        if not self.cap:
            return
        a, b = e[0], max(e[1], e[0] + 1)
        pw = self.plot_w()
        view = pw * self.spp
        if view < (b - a) * 1.1 or view > (b - a) * 4:
            self.spp = (b - a) / 0.7 / pw
            self.need_fit = False
        self.t0 = (a + b) / 2 - pw * self.spp / 2
        self.clamp()
        self.area.queue_draw()

    def decoder_dialog(self, di):
        """Adds (di None) or edits decoder di."""
        win = Gtk.Window(transient_for=self, modal=True, title="解析の設定" if di is not None else "解析を追加",
                         resizable=False)
        outer = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8, margin_top=12, margin_bottom=12,
                        margin_start=12, margin_end=12)
        win.set_child(outer)
        grid = Gtk.Grid(column_spacing=10, row_spacing=6)
        outer.append(grid)
        names = list(D.DECODERS)
        proto = Gtk.DropDown.new_from_strings([D.DECODERS[n][0] for n in names])
        grid.attach(Gtk.Label(label="種類", xalign=0), 0, 0, 1, 1)
        grid.attach(proto, 1, 0, 1, 1)
        if di is not None:
            proto.set_selected(names.index(self.decs[di]["name"]))
            proto.set_sensitive(False)
        err = Gtk.Label(xalign=0)
        err.add_css_class("error")
        outer.append(err)
        fields = []                             # (key, getter)

        def build(*_):
            while grid.get_child_at(0, 1) is not None:
                grid.remove_row(1)
            fields.clear()
            name = names[proto.get_selected()]
            opts = self.decs[di]["opts"] if di is not None else D.parse_spec(name)[1]
            for r, (k, label, kind, _, choices) in enumerate(D.DECODERS[name][2], 1):
                v = opts[k]
                grid.attach(Gtk.Label(label=label, xalign=0), 0, r, 1, 1)
                if kind in ("ch", "ch?"):
                    items = (["なし"] if kind == "ch?" else []) + ["CH%02d" % c for c in range(CHANNELS)]
                    w = Gtk.DropDown.new_from_strings(items)
                    w.set_selected(0 if v is None else v + (kind == "ch?"))
                    off = 1 if kind == "ch?" else 0
                    get = (lambda w=w, off=off: "none" if off and w.get_selected() == 0
                           else str(w.get_selected() - off))
                elif kind in ("chs", "chs?"):
                    w = Gtk.Entry(text=D.ch_ranges(v) if v else "", width_chars=14)
                    get = w.get_text
                elif kind == "choice":
                    w = Gtk.DropDown.new_from_strings([str(c) for c in choices])
                    w.set_selected(choices.index(v))
                    get = (lambda w=w, choices=choices: str(choices[w.get_selected()]))
                else:
                    w = Gtk.Entry(text=v if isinstance(v, str) else "%g" % v, width_chars=10)
                    get = w.get_text
                grid.attach(w, 1, r, 1, 1)
                fields.append((k, get))

        proto.connect("notify::selected", build)
        build()
        btns = Gtk.Box(spacing=6, halign=Gtk.Align.END)
        outer.append(btns)
        if di is not None:
            rm = Gtk.Button(label="削除")
            rm.add_css_class("destructive-action")
            btns.append(rm)

            def remove(*_):
                del self.decs[di]
                self.run_decoders()
                win.close()
            rm.connect("clicked", remove)
        cancel = Gtk.Button(label="キャンセル")
        cancel.connect("clicked", lambda *_: win.close())
        ok = Gtk.Button(label="OK")
        ok.add_css_class("suggested-action")
        btns.append(cancel)
        btns.append(ok)

        def accept(*_):
            name = names[proto.get_selected()]
            try:
                _, opts = D.parse_spec(name + ":" + ",".join("%s=%s" % (k, g()) for k, g in fields))
            except ValueError as e:
                err.set_text("設定が正しくありません: %s" % e)
                return
            if di is None:
                self.decs.append({"name": name, "opts": opts})
            else:
                self.decs[di]["opts"] = opts
            secs = self.run_decoders()
            if self.cap:
                n = sum(len(a) for _, a in self.decs[-1 if di is None else di]["res"].rows)
                self.say("%s: %d 個（%.2f 秒）" % (D.DECODERS[name][0], n, secs))
            if not self.list_btn.get_active() and self.decs:
                self.list_btn.set_active(True)
            win.close()
        ok.connect("clicked", accept)
        win.present()
        self.dialog = win

    # ---- drawing ----

    def draw(self, _area, cr, w, h):
        cr.set_source_rgb(*BG)
        cr.paint()
        ent, rh = self.layout()
        roles = self.roles()
        cr.select_font_face("monospace")
        for r, e in enumerate(ent):
            y = HEAD_H + r * rh
            cr.set_source_rgb(*(DEC_BG if e[0] == "dec" else GRID if r % 2 else (0.15, 0.15, 0.17)))
            cr.rectangle(0, y, NAME_W - 2, rh - 1)
            cr.fill()
            cr.set_font_size(min(13, rh * 0.6))
            cr.set_source_rgb(*TEXT)
            cr.move_to(6, y + rh / 2 + 4)
            if e[0] == "ch":
                cr.show_text("CH%02d" % e[1])
                if e[1] in roles:
                    cr.set_source_rgb(*ROLE)
                    cr.set_font_size(min(10, rh * 0.5))
                    cr.move_to(46, y + rh / 2 + 4)
                    cr.show_text(roles[e[1]][:9])
                c = self.conds.get(e[1])
                if c:
                    cr.set_font_size(min(13, rh * 0.6))
                    cr.set_source_rgb(*TRIG)
                    cr.move_to(NAME_W - 18, y + rh / 2 + 4)
                    cr.show_text(COND_TEXT[c])
            else:
                name, tag = self.row_name(e[1], e[2])
                fs = min(13, rh * 0.6)
                ts = min(10, rh * 0.5)
                for _ in range(6):                     # shrink until name + tag fit
                    cr.set_font_size(fs)
                    nw = cr.text_extents(name + " ").x_advance
                    cr.set_font_size(ts)
                    if 6 + nw + cr.text_extents(tag).x_advance <= NAME_W - 6:
                        break
                    fs, ts = fs * 0.9, ts * 0.9
                cr.set_font_size(fs)
                cr.set_source_rgb(*TEXT)
                cr.move_to(6, y + rh / 2 + 4)
                cr.show_text(name)
                cr.set_font_size(ts)
                cr.set_source_rgb(*ROLE)
                cr.move_to(6 + nw, y + rh / 2 + 4)
                cr.show_text(tag)
        cap = self.cap
        if not cap or not len(cap):
            return
        pw = self.plot_w()
        if self.need_fit:
            self.spp, self.t0, self.need_fit = len(cap) / pw, 0.0, False
        cr.rectangle(NAME_W, 0, pw, h)
        cr.clip()
        # time axis, relative to the trigger
        span = pw * self.spp / cap.rate
        step = nice_step(span / 8)
        t_left = (self.t0 - cap.trig) / cap.rate
        k = int(t_left // step)
        cr.set_font_size(11)
        while True:
            t = k * step
            x = self.sample_to_x(cap.trig + t * cap.rate)
            if x > w:
                break
            cr.set_source_rgb(*GRID)
            cr.move_to(x, HEAD_H - 6)
            cr.line_to(x, h)
            cr.set_line_width(1)
            cr.stroke()
            cr.set_source_rgb(*TEXT)
            cr.move_to(x + 3, HEAD_H - 9)
            cr.show_text(fmt_time(t) if abs(t) > step / 1000 else "0")
            k += 1
        a_idx = max(0, int(self.t0) - 1)
        b_idx = min(len(cap), int(self.t0 + pw * self.spp) + 2)
        for r, e in enumerate(ent):
            y = HEAD_H + r * rh
            if e[0] == "ch":
                for di, ri in e[2]:                     # decoded boxes behind the line, text on top
                    d = self.decs[di]
                    self.draw_anns(cr, d["res"].rows[ri][1], d["rows"][ri], y, rh, a_idx, b_idx, pw, "box")
                self.draw_trace(cr, e[1], y, rh, a_idx, b_idx, pw)
                for di, ri in e[2]:
                    d = self.decs[di]
                    self.draw_anns(cr, d["res"].rows[ri][1], d["rows"][ri], y, rh, a_idx, b_idx, pw, "text")
            else:
                d = self.decs[e[1]]
                self.draw_anns(cr, d["res"].rows[e[2]][1], d["rows"][e[2]], y, rh, a_idx, b_idx, pw)
        # markers
        for idx, col in ((cap.trig, TRIG), (self.cur_a, CUR_A), (self.cur_b, CUR_B)):
            if idx is None:
                continue
            x = self.sample_to_x(idx)
            cr.set_source_rgb(*col)
            cr.set_line_width(1.5)
            cr.move_to(x, HEAD_H - 4)
            cr.line_to(x, h)
            cr.stroke()

    def draw_trace(self, cr, ch, y, rh, a_idx, b_idx, pw):
        cap = self.cap
        cr.set_line_width(1.2)
        hi, lo = y + rh * 0.2, y + rh * 0.8
        edges = cap.edges[ch]
        i0 = bisect.bisect_right(edges, a_idx)
        i1 = bisect.bisect_right(edges, b_idx)
        cr.set_source_rgb(*TRACE)
        if i1 - i0 < pw:                        # few edges: draw each
            v = cap.level(ch, a_idx)
            x = self.sample_to_x(a_idx)
            cr.move_to(x, hi if v else lo)
            for e in edges[i0:i1]:
                xe = self.sample_to_x(e)
                cr.line_to(xe, hi if v else lo)
                v ^= 1
                cr.line_to(xe, hi if v else lo)
            cr.line_to(self.sample_to_x(b_idx), hi if v else lo)
            cr.stroke()
        else:                                   # many: a block where a pixel holds several
            x = NAME_W
            while x < NAME_W + pw:
                s0 = self.x_to_sample(x)
                j0 = bisect.bisect_right(edges, s0)
                j1 = bisect.bisect_right(edges, s0 + self.spp)
                if j1 - j0 > 1:
                    cr.move_to(x + 0.5, hi)
                    cr.line_to(x + 0.5, lo)
                else:
                    v = cap.level(ch, int(s0))
                    cr.move_to(x, hi if v else lo)
                    cr.line_to(x + 1, hi if v else lo)
                    if j1 > j0:
                        cr.move_to(x + 0.5, hi)
                        cr.line_to(x + 0.5, lo)
                x += 1
            cr.stroke()

    def draw_anns(self, cr, anns, index, y, rh, a_idx, b_idx, pw, part="all"):
        """Boxes with pointed ends; the longest text that fits; neighbours closer than
        a pixel or two merged (so a long capture stays quick to draw). part: "box" or
        "text" alone, for drawing a waveform in between."""
        starts, longest = index
        k = bisect.bisect_left(starts, a_idx - longest)
        top, bot, mid = y + rh * 0.15, y + rh * 0.85, y + rh / 2
        cr.set_font_size(min(12, rh * 0.5))
        cr.set_line_width(1)
        last_x = -1e9
        while k < len(anns):
            a = anns[k]
            k += 1
            if a.start > b_idx:
                break
            if a.end < a_idx:
                continue
            x0 = self.sample_to_x(a.start)
            x1 = max(self.sample_to_x(a.end), x0 + 1)
            if x1 < last_x + 1.5:               # inside what is drawn already
                continue
            x0 = max(x0, last_x)
            last_x = x1
            fill = KIND_COL.get(a.kind, KIND_COL["info"])
            if part != "text":
                tip = min(4.0, (x1 - x0) / 3)
                cr.move_to(x0, mid)
                cr.line_to(x0 + tip, top)
                cr.line_to(x1 - tip, top)
                cr.line_to(x1, mid)
                cr.line_to(x1 - tip, bot)
                cr.line_to(x0 + tip, bot)
                cr.close_path()
                cr.set_source_rgba(*fill, 0.35)
                cr.fill_preserve()
                cr.set_source_rgb(*fill)
                cr.stroke()
            room = min(x1, NAME_W + pw) - max(x0, NAME_W) - 6
            if room > 6 and part != "box":
                for t in a.texts:
                    ext = cr.text_extents(t)
                    if ext.x_advance <= room:
                        tx = (max(x0, NAME_W) + min(x1, NAME_W + pw)) / 2 - ext.x_advance / 2
                        if part == "text":              # on a waveform: a dark patch under the text
                            cr.set_source_rgba(*fill, 0.9)
                            cr.rectangle(tx - 2, mid - 6, ext.x_advance + 4, 12)
                            cr.fill()
                            cr.set_source_rgba(*BG, 0.55)
                            cr.rectangle(tx - 2, mid - 6, ext.x_advance + 4, 12)
                            cr.fill()
                        cr.set_source_rgb(0.95, 0.95, 0.95)
                        cr.move_to(tx, mid + 4)
                        cr.show_text(t)
                        break

    # ---- files ----

    def save(self):
        if not self.cap:
            return
        dlg = Gtk.FileDialog(initial_name="capture.sr")

        def chosen(d, res):
            try:
                f = d.save_finish(res)
            except GLib.Error:
                return
            path = f.get_path()
            (P.write_sr if path.endswith(".sr") else P.write_vcd)(path, self.cap.samples, self.cap.rate,
                                                                  CHANNELS, self.cap.trig)
            self.say("保存しました: %s" % path)
        dlg.save(self, None, chosen)

    def pulseview(self):
        if not self.cap:
            return
        fd, path = tempfile.mkstemp(prefix="pokeana_", suffix=".sr")
        os.close(fd)
        P.write_sr(path, self.cap.samples, self.cap.rate, CHANNELS, self.cap.trig)
        P.view(path)
        self.say("PulseView で開きます: %s" % path)

    def dump(self):
        """Testing: with POKEANA_GUI_DUMP set, each capture's decoded messages go to that file
        as one JSON line (tests/m2k_eeprom.py check reads it)."""
        path = os.environ.get("POKEANA_GUI_DUMP")
        if not path or not self.cap:
            return
        rec = {"t": time.time(), "n": len(self.cap), "rate": self.cap.rate, "trig": self.cap.trig,
               "decs": [{"spec": D.spec_str(d["name"], d["opts"]), "msgs": d["res"].msgs} for d in self.decs]}
        with open(path, "a") as f:
            f.write(json.dumps(rec, default=str) + "\n")

    def shot(self, path, window=None):
        """Writes the window as a PNG (for checking the GUI without a screen)."""
        child = (window or self).get_child()
        w, h = child.get_width(), child.get_height()
        snap = Gtk.Snapshot()
        Gtk.WidgetPaintable.new(child).snapshot(snap, w, h)
        node = snap.to_node()
        if node is None:                        # not drawn yet: again a little later
            GLib.timeout_add(300, lambda: (self.shot(path), False)[1])
            return False
        self.get_native().get_renderer().render_texture(node, None).save_to_png(path)
        return True


class App(Gtk.Application):
    def __init__(self):
        super().__init__(application_id="local.pokeana.gui", flags=Gio.ApplicationFlags.NON_UNIQUE)

    def do_activate(self):
        path = next((a for a in sys.argv[1:] if not a.startswith("-")), None)
        win = Window(self, path)
        win.present()
        shot = os.environ.get("POKEANA_GUI_SHOT")       # testing: actions, a picture, quit
        if shot:
            def go():
                for a in os.environ.get("POKEANA_GUI_DO", "").split(";"):
                    if a.startswith("cond "):
                        _, ch, c = a.split()
                        win.conds[int(ch)] = c
                    elif a.startswith("rate "):
                        win.rate_dd.set_selected(RATES.index(float(a.split()[1])))
                    elif a.startswith("depth "):
                        win.depth_dd.set_selected(DEPTHS.index(int(a.split()[1])))
                    elif a.startswith("pre "):
                        win.pre_spin.set_value(int(a.split()[1]))
                    elif a.startswith("show "):
                        win.show_spin.set_value(int(a.split()[1]))
                    elif a.startswith("zoom "):
                        win.spp = len(win.cap) / win.plot_w() / float(a.split()[1])
                        win.need_fit = False
                        win.t0 = win.cap.trig - win.plot_w() * win.spp / 2
                    elif a.startswith("cursors "):
                        _, ca, cb = a.split()
                        win.cur_a, win.cur_b = int(ca), int(cb)
                        win.update_info()
                    elif a.startswith("dec "):
                        win.add_decoder(a[4:].strip())
                    elif a.startswith("place "):        # 0 hidden, 1 on the line, 2 under it, 3 at the bottom
                        win.place_dd.set_selected(int(a.split()[1]))
                    elif a == "list":
                        win.list_btn.set_active(True)
                    elif a.startswith("goto "):
                        t = time.time()                 # let the list take its width first
                        while time.time() - t < 0.5:
                            GLib.MainContext.default().iteration(False)
                        win.goto(win.log_entries[int(a.split()[1])])
                    elif a.startswith("dialog"):
                        win.decoder_dialog(int(a.split()[1]) if len(a.split()) > 1 else None)
                    elif a == "run":
                        win.start()
                        while win.busy:
                            GLib.MainContext.default().iteration(True)
                    elif a.startswith("repeat "):       # repeat SECS [EVERY]: Repeat, a picture every EVERY s
                        secs, every = (float(x) for x in (a.split()[1:] + ["0"])[:2])
                        t0 = last = time.time()
                        win.repeat_btn.set_active(True)
                        win.start()
                        while time.time() - t0 < secs:
                            GLib.MainContext.default().iteration(False)
                            time.sleep(0.005)
                            if every and time.time() - last >= every:
                                last = time.time()
                                win.shot(shot.replace(".png", time.strftime("_%m%d_%H%M%S.png")))
                        win.repeat_btn.set_active(False)
                        while win.busy:
                            GLib.MainContext.default().iteration(True)
                win.area.queue_draw()
                def snap():
                    dlg = getattr(win, "dialog", None)
                    if dlg is not None and dlg.get_visible():
                        if not win.shot(shot.replace(".png", "_dialog.png"), dlg):
                            return True
                    if win.shot(shot):
                        self.quit()
                        return False
                    return True
                GLib.timeout_add(500, snap)
                return False
            GLib.timeout_add(800, go)


if __name__ == "__main__":
    App().run([sys.argv[0]])
