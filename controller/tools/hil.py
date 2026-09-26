#!/usr/bin/env python3
"""Hardware-in-the-loop tool for the reflow controller.

Drives the device over its UART console, records the controller's @-lines
(see controller/main/reflow_ctrl.h) and analyses them. Standard library plus
pyserial only, so it runs in the ESP-IDF Python environment:

    . ~/esp/esp-idf-v6.1/export.sh
    python controller/tools/hil.py run "SAC305 Lead-Free"
    python controller/tools/hil.py step 1.0 600 200 --duration 900
    python controller/tools/hil.py hold 150 --duration 600
    python controller/tools/hil.py cmd "ctl show"
    python controller/tools/hil.py fit  controller/tools/logs/<file>.log
    python controller/tools/hil.py analyze controller/tools/logs/<file>.log

`idf.py monitor` must be closed while this runs (one owner per serial port).
Ctrl-C during a recording sends `stop`, so the heater is switched off.
"""
import argparse
import glob
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(HERE, "logs")
PROFILE_DIR = os.path.join(HERE, "..", "spiffs_image", "profiles")

T_FIELDS = ["ms", "state", "pt", "sp", "sp_ff", "temp", "tf", "slope",
            "u_ff", "p", "i", "u", "want", "forced", "plug", "pending"]


# ── Serial ──────────────────────────────────────────────────────────────────

def default_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*")
                   + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    return ports[0] if ports else None


def open_port(port):
    import serial  # pyserial, present in the ESP-IDF Python environment
    s = serial.Serial()
    s.port = port
    s.baudrate = 115200
    s.timeout = 0.2
    # The port is the ESP32-C6's USB-Serial-JTAG, which resets the chip on
    # some DTR/RTS transitions. Leave both as the OS asserts them on open, and
    # clear HUPCL so closing the port does not drop them either.
    s.open()
    try:
        import termios
        attrs = termios.tcgetattr(s.fd)
        attrs[2] &= ~termios.HUPCL
        termios.tcsetattr(s.fd, termios.TCSANOW, attrs)
    except (ImportError, AttributeError, OSError):
        pass
    return s


def send(ser, line):
    ser.write((line + "\r").encode())
    ser.flush()


def read_lines(ser):
    buf = b""
    while True:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                yield raw.decode(errors="replace").rstrip("\r")
        else:
            yield None


# ── Recording ───────────────────────────────────────────────────────────────

def record(args, command, kind):
    port = args.port or default_port()
    if not port:
        sys.exit("no serial port found; pass --port")
    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(LOG_DIR, time.strftime("%Y%m%d-%H%M%S") + "_" + kind + ".log")
    ser = open_port(port)
    print(f"port {port} -> {path}")
    t0 = time.time()
    ended_at = None
    stopped = False
    status_at = 0.0
    with open(path, "w") as log:
        log.write(f"# cmd: {command}\n")
        time.sleep(0.3)
        ser.reset_input_buffer()
        send(ser, "log on")
        send(ser, command)
        try:
            for line in read_lines(ser):
                now = time.time()
                if line is not None:
                    log.write(line + "\n")
                    if line.startswith("@T"):
                        f = line.split(",")
                        if now - status_at >= 5:
                            status_at = now
                            print(f"  {now - t0:6.0f}s {f[2]:8s} pt={f[3]:>6s} sp={f[4]:>6s} "
                                  f"T={f[6]:>7s} u={f[12]:>5s} plug={f[15]}", flush=True)
                    elif line.startswith("@") or "Refused" in line or "not found" in line:
                        print("  " + line, flush=True)
                        if line.startswith("@E") and ",end," in line and ended_at is None:
                            ended_at = now
                        if "Refused" in line or "not found" in line:
                            break
                if ended_at is not None and now - ended_at >= args.after:
                    break
                if args.duration and now - t0 >= args.duration and not stopped:
                    print("  duration reached: stop")
                    send(ser, "stop")
                    stopped = True
                    if ended_at is None:
                        ended_at = now
        except KeyboardInterrupt:
            print("\n  interrupted: stop")
            send(ser, "stop")
            deadline = time.time() + 3
            for line in read_lines(ser):
                if line is not None:
                    log.write(line + "\n")
                if time.time() > deadline:
                    break
        send(ser, "log off")
    ser.close()
    print(f"saved {path}")
    return path


def cmd_simple(args):
    port = args.port or default_port()
    ser = open_port(port)
    time.sleep(0.3)
    ser.reset_input_buffer()
    send(ser, args.line)
    deadline = time.time() + args.wait
    for line in read_lines(ser):
        if line is not None and not line.startswith("@T"):
            print(line)
        if time.time() > deadline:
            break
    ser.close()


# ── Log parsing ─────────────────────────────────────────────────────────────

class Run:
    def __init__(self, path):
        self.path = path
        self.rows, self.events, self.params, self.start = [], [], {}, None
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line.startswith("@T"):
                    parts = line.split(",")[1:]
                    if len(parts) != len(T_FIELDS):
                        continue
                    r = dict(zip(T_FIELDS, parts))
                    for k in T_FIELDS:
                        if k != "state":
                            r[k] = float(r[k])
                    self.rows.append(r)
                elif line.startswith("@E"):
                    self.events.append(line.split(",")[1:])
                elif line.startswith("@P"):
                    for kv in line.split(",")[1:]:
                        k, _, v = kv.partition("=")
                        self.params[k] = float(v)
                elif line.startswith("@S"):
                    self.start = line.split(",")[1:]
        if not self.rows:
            sys.exit(f"{path}: no @T lines")
        self.t0 = self.rows[0]["ms"]
        for r in self.rows:
            r["t"] = (r["ms"] - self.t0) / 1000.0
        self.profile = None
        if self.start and self.start[1] == "profile":
            self.profile = load_profile(self.start[2])
        # Profile time at the start. Always 0 now; older logs from a firmware
        # that joined the curve part-way record it as start_pt.
        self.start_pt = 0.0
        for field in (self.start or [])[2:]:
            k, _, v = field.partition("=")
            if k == "start_pt":
                self.start_pt = float(v)

    def active(self):
        return [r for r in self.rows if r["state"] in ("running", "cooling")]


def load_profile(name):
    for p in glob.glob(os.path.join(PROFILE_DIR, "*.json")):
        with open(p) as f:
            d = json.load(f)
        if d.get("name") == name:
            return d
    return None


def curve_at(prof, t):
    w = prof["waypoints"]
    if t <= w[0]["time"]:
        return w[0]["temp"]
    for a, b in zip(w, w[1:]):
        if t < b["time"]:
            return a["temp"] + (b["temp"] - a["temp"]) * (t - a["time"]) / (b["time"] - a["time"])
    return w[-1]["temp"]


def segment_label(prof, t):
    w = prof["waypoints"]
    for b in w[1:]:
        if t < b["time"]:
            return b["label"]
    return w[-1]["label"]


def time_above(prof, level):
    """Seconds the piecewise-linear curve spends at or above level."""
    total = 0.0
    w = prof["waypoints"]
    for a, b in zip(w, w[1:]):
        t0, t1, y0, y1 = a["time"], b["time"], a["temp"], b["temp"]
        if y0 >= level and y1 >= level:
            total += t1 - t0
        elif y0 >= level or y1 >= level:
            f = (level - y0) / (y1 - y0)
            tc = t0 + f * (t1 - t0)
            total += (t1 - tc) if y1 >= level else (tc - t0)
    return total


def liquidus_for(name, override):
    if override:
        return override
    n = (name or "").lower()
    if "sn63" in n or "leaded" in n:
        return 183.0
    if "sac" in n or "lead-free" in n:
        return 217.0
    return None


# ── Analysis ────────────────────────────────────────────────────────────────

def switching_stats(run):
    rows = [r for r in run.rows if r["plug"] >= 0]
    on_d, off_d, switches = [], [], 0
    last_t, last_s = None, None
    for r in rows:
        s = int(r["plug"])
        if last_s is None:
            last_s, last_t = s, r["t"]
            continue
        if s != last_s:
            (on_d if last_s == 1 else off_d).append(r["t"] - last_t)
            switches += 1
            last_s, last_t = s, r["t"]
    # The first and last intervals are cut off by the log: drop them.
    lat, rejects, fails = [], 0, 0
    pending_cmd = None
    for e in run.events:
        ms, kind = float(e[0]), e[1]
        if kind == "cmd":
            if e[3] == "ok":
                pending_cmd = ms
            else:
                rejects += 1
        elif kind == "plug":
            if e[2] != "ok":
                fails += 1
            # Events of one tick print after its command line: only a result
            # stamped after the command belongs to it.
            if pending_cmd is not None and ms >= pending_cmd:
                lat.append((ms - pending_cmd) / 1000.0)
                pending_cmd = None
    return switches, on_d[1:], off_d[1:], lat, rejects, fails


def fmt_stats(xs):
    if not xs:
        return "n/a"
    xs = sorted(xs)
    return f"min {xs[0]:.1f} / median {xs[len(xs) // 2]:.1f} / max {xs[-1]:.1f} s (n={len(xs)})"


def phase_table(title, prof, pairs):
    """pairs: (profile time, error). Prints RMS/mean/min/max per phase."""
    segs = {}
    for pt, e in pairs:
        segs.setdefault(segment_label(prof, pt), []).append(e)
    print(title)
    allv = []
    for w in prof["waypoints"][1:]:
        v = segs.get(w["label"])
        if not v:
            continue
        allv += v
        rms = math.sqrt(sum(x * x for x in v) / len(v))
        print(f"    {w['label']:10s} rms {rms:5.2f}  mean {sum(v) / len(v):+6.2f}  "
              f"min {min(v):+6.2f}  max {max(v):+6.2f} C")
    if allv:
        rms = math.sqrt(sum(x * x for x in allv) / len(allv))
        print(f"    {'all':10s} rms {rms:5.2f}  max|e| {max(abs(x) for x in allv):.2f} C")


def find_holds(act):
    """Stretches where the profile clock ran slower than real time. Returns
    (start row, end row, seconds held) per stretch."""
    holds, cur = [], None
    for a, b in zip(act, act[1:]):
        dt, dpt = b["t"] - a["t"], b["pt"] - a["pt"]
        if dt > 0 and dpt < 0.9 * dt:
            if cur and a["t"] - act[cur[1]]["t"] <= 1.0:
                cur[1] = act.index(b, cur[1])
                cur[2] += dt - dpt
            else:
                if cur:
                    holds.append(tuple(cur))
                i = act.index(a)
                cur = [i, i + 1, dt - dpt]
    if cur:
        holds.append(tuple(cur))
    return [h for h in holds if h[2] >= 1.0]


def heater_not_heating(act, min_s=10.0):
    """Stretches of at least min_s with the plug confirmed on while the
    temperature falls: the heat source itself is off (e.g. its own thermostat
    has cut out), not the plug."""
    out, start = [], None
    for r in act:
        bad = r["plug"] == 1 and r["slope"] < 0
        if bad and start is None:
            start = r
        elif not bad and start is not None:
            if r["t"] - start["t"] >= min_s:
                out.append((start, r))
            start = None
    if start is not None and act[-1]["t"] - start["t"] >= min_s:
        out.append((start, act[-1]))
    return out


def cmd_analyze(args):
    run = Run(args.log)
    act = run.active()
    print(f"{os.path.basename(run.path)}: {run.start[1] if run.start else '?'} "
          f"{run.start[2] if run.start else ''}")
    ends = [e for e in run.events if e[1] == "end"]
    if ends:
        print(f"  ended: {ends[-1][2]} {ends[-1][3] if len(ends[-1]) > 3 else ''}")
    if not act:
        sys.exit("  no active samples")
    t0 = act[0]["t"]
    dur = act[-1]["t"] - t0
    print(f"  duration {dur:.0f} s real time, profile clock {act[0]['pt']:.0f} -> "
          f"{act[-1]['pt']:.0f} s" + (f" (old firmware: joined at {run.start_pt:.0f} s)"
                                      if run.start_pt > 0 else ""))
    if run.profile:
        prof = run.profile
        w = prof["waypoints"]
        cool_start = w[-1]["time"]
        for i in range(len(w) - 1, 0, -1):
            if w[i - 1]["temp"] > w[i]["temp"]:
                cool_start = w[i - 1]["time"]
            else:
                break
        heating = [r for r in act if r["temp"] > -100]

        # 1. Against the PLAN in real time: what the profile asked for at this
        #    moment. Holds and a slow heat source show up here.
        # A hot start waits (heater off) until the curve reaches the iron;
        # that catch-up is not tracking error.
        pairs, caught_up = [], None
        for r in heating:
            plan_t = run.start_pt + (r["t"] - t0)
            if caught_up is None and r["temp"] <= curve_at(prof, plan_t) + 1.0:
                caught_up = r["t"] - t0
            if caught_up is not None and plan_t < cool_start:
                pairs.append((plan_t, r["temp"] - curve_at(prof, plan_t)))
        if caught_up:
            print(f"  started at {act[0]['temp']:.1f} C, above the curve: heater off until "
                  f"the curve caught up at {caught_up:.0f} s")
        phase_table("  against the plan, real time (measured - planned), heating phases:",
                    prof, pairs)

        holds = find_holds(act)
        # 2. Against the target the controller USED (profile clock): how well
        #    the loop tracked what it aimed for. Differs from 1 only if the
        #    clock was held (max_hold > 0).
        if holds:
            pairs = [(r["pt"], r["temp"] - r["sp"]) for r in heating if r["state"] == "running"]
            phase_table("  against the target used (profile clock):", prof, pairs)

        total = sum(h[2] for h in holds)
        print(f"  profile clock held {total:.0f} s in total" + (":" if holds else ""))
        for i, j, held in holds:
            a, b = act[i], act[j]
            seg = act[i:j + 1]
            on = sum(1 for r in seg if r["plug"] == 1) / len(seg)
            print(f"    at {a['t'] - t0:5.0f}-{b['t'] - t0:5.0f} s ({segment_label(prof, a['pt'])}, "
                  f"pt {a['pt']:.0f}): held {held:4.0f} s, T {a['temp']:.1f} -> "
                  f"min {min(r['temp'] for r in seg):.1f} -> {b['temp']:.1f} C, plug on {on:.0%}")
        peak_t = max(x["temp"] for x in w)
        peak = max(r["temp"] for r in act)
        print(f"  peak {peak:.1f} C vs target {peak_t:.1f} C ({peak - peak_t:+.1f})")
        liq = liquidus_for(prof.get("name"), args.liquidus)
        if liq:
            tal = sum(0.5 for r in run.rows if r["temp"] >= liq)
            print(f"  time above {liq:.0f} C: {tal:.0f} s (profile {time_above(prof, liq):.0f} s)")
    for a, b in heater_not_heating(act):
        print(f"  HEAT SOURCE NOT HEATING at {a['t'] - t0:.0f}-{b['t'] - t0:.0f} s: plug confirmed on, "
              f"T {a['temp']:.1f} -> {b['temp']:.1f} C (its own thermostat?)")
    slopes = [r["slope"] for r in act]
    print(f"  ramp rate: max {max(slopes):+.2f}, min {min(slopes):+.2f} C/s")
    sw, on_d, off_d, lat, rej, fails = switching_stats(run)
    print(f"  switches {sw} ({sw / max(dur, 1) * 60:.1f}/min); min dwell set "
          f"{run.params.get('min_dwell', float('nan')):g} s")
    print(f"    on  {fmt_stats(on_d)}")
    print(f"    off {fmt_stats(off_d)}")
    print(f"    cmd->confirm {fmt_stats(lat)}; rejected cmds {rej}; failed changes {fails}")
    for e in run.events:
        if e[1] == "hold_limit":
            print(f"  hold limit reached in {e[2]} at {(float(e[0]) - run.t0) / 1000 - t0:.0f} s: "
                  "phase continued in real time")
    anomalies = [e for e in run.events if e[1] == "anomaly"]
    if anomalies:
        print(f"  heating-while-off corrections: {len(anomalies)} at "
              + ", ".join(f"{(float(e[0]) - run.t0) / 1000 - t0:.0f} s" for e in anomalies))
    duty = [r["u"] for r in act]
    plug_on = [r["plug"] == 1 for r in act if r["plug"] >= 0]
    if plug_on:
        print(f"  mean commanded duty {sum(duty) / len(duty):.3f}, "
              f"delivered {sum(plug_on) / len(plug_on):.3f}")
    svg = os.path.splitext(run.path)[0] + ".svg"
    write_svg(run, svg)
    print(f"  chart: {svg}")


def write_svg(run, path):
    W, H, L, R, T, B = 960, 420, 50, 20, 20, 60
    rows = run.rows
    tmax = max(r["t"] for r in rows) or 1
    ys = [r["temp"] for r in rows if r["temp"] > -100] + [r["sp"] for r in rows if r["sp"] > 0]
    ymax = max(50, math.ceil(max(ys) / 25) * 25 + 25)
    ymin = max(0, math.floor(min(ys) / 25) * 25 - 25)
    px = lambda t: L + (W - L - R) * t / tmax
    py = lambda y: T + (H - T - B) * (1 - (y - ymin) / (ymax - ymin))

    def poly(pts, color, extra=""):
        if len(pts) < 2:
            return ""
        d = " ".join(f"{px(t):.1f},{py(y):.1f}" for t, y in pts)
        return f'<polyline fill="none" stroke="{color}" stroke-width="1.6" {extra} points="{d}"/>'

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
           f'font-family="sans-serif" font-size="11"><rect width="100%" height="100%" fill="#fff"/>']
    for y in range(int(ymin), int(ymax) + 1, 25):
        out.append(f'<line x1="{L}" x2="{W - R}" y1="{py(y):.1f}" y2="{py(y):.1f}" stroke="#eee"/>'
                   f'<text x="{L - 6}" y="{py(y) + 4:.1f}" text-anchor="end" fill="#888">{y}</text>')
    step = 30 if tmax <= 600 else 60 if tmax <= 1500 else 300
    for t in range(0, int(tmax) + 1, step):
        out.append(f'<line x1="{px(t):.1f}" x2="{px(t):.1f}" y1="{T}" y2="{H - B}" stroke="#f3f3f3"/>'
                   f'<text x="{px(t):.1f}" y="{H - B + 14}" text-anchor="middle" fill="#888">{t}</text>')
    # Heater: confirmed state as a band under the plot.
    band_y = H - B + 24
    for a, b in zip(rows, rows[1:]):
        if a["plug"] == 1:
            out.append(f'<rect x="{px(a["t"]):.1f}" y="{band_y}" width="{max(px(b["t"]) - px(a["t"]), 0.5):.1f}" '
                       f'height="10" fill="#c05c28"/>')
        if a["u"] > 0:
            out.append(f'<rect x="{px(a["t"]):.1f}" y="{band_y + 14 + 10 * (1 - a["u"]):.1f}" '
                       f'width="{max(px(b["t"]) - px(a["t"]), 0.5):.1f}" height="{10 * a["u"]:.1f}" fill="#999"/>')
    out.append(f'<text x="{L - 6}" y="{band_y + 9}" text-anchor="end" fill="#888">plug</text>'
               f'<text x="{L - 6}" y="{band_y + 23}" text-anchor="end" fill="#888">u</text>')
    act = run.active()
    if run.profile and act:
        t0 = act[0]["t"]
        plan = [(r["t"], curve_at(run.profile, run.start_pt + r["t"] - t0))
                for r in rows if r["t"] >= t0]
        out.append(poly(plan, "#8aa4cc", 'stroke-dasharray="5,4"'))
    if not run.profile or find_holds(act):
        # Target the controller used; only differs from the plan when held.
        sp = [(r["t"], r["sp"]) for r in rows if r["sp"] > 0 and r["state"] in ("running", "cooling")]
        out.append(poly(sp, "#2558a0"))
    out.append(poly([(r["t"], r["temp"]) for r in rows if r["temp"] > -100], "#c05c28"))
    title = " ".join(run.start[1:3]) if run.start else os.path.basename(run.path)
    out.append(f'<text x="{L}" y="14" fill="#333">{title} — measured (orange), target used (blue), plan in real time (dashed)</text>')
    out.append("</svg>")
    with open(path, "w") as f:
        f.write("\n".join(out))


# ── Model fit ───────────────────────────────────────────────────────────────

def lstsq(X, y):
    """Normal equations for a handful of columns."""
    n = len(X[0])
    A = [[sum(r[i] * r[j] for r in X) for j in range(n)] for i in range(n)]
    b = [sum(r[i] * v for r, v in zip(X, y)) for i in range(n)]
    for c in range(n):  # Gauss-Jordan
        p = max(range(c, n), key=lambda k: abs(A[k][c]))
        A[c], A[p], b[c], b[p] = A[p], A[c], b[p], b[c]
        if abs(A[c][c]) < 1e-12:
            return None
        for k in range(n):
            if k != c:
                f = A[k][c] / A[c][c]
                A[k] = [x - f * z for x, z in zip(A[k], A[c])]
                b[k] -= f * b[c]
    return [b[i] / A[i][i] for i in range(n)]


def simulate(ts, x, T0, a, b, c, t_room):
    T, out = T0, [T0]
    for k in range(1, len(ts)):
        dt = ts[k] - ts[k - 1]
        d = T - t_room
        T += dt * (a * x[k - 1] - b * d - c * d * max(d, 0))
        out.append(T)
    return out


def element_input(ts, u_at, theta, tau_e):
    """Heater input as the soleplate sees it: the plug state delayed by theta,
    then through a first-order lag tau_e (the element heating the plate)."""
    x, v = [], 0.0
    for k, t in enumerate(ts):
        u = u_at(t - theta)
        if k and tau_e > 0:
            dt = ts[k] - ts[k - 1]
            v += (u - v) * dt / (tau_e + dt)
        else:
            v = u if tau_e <= 0 else v
        x.append(v)
    return x


def cmd_fit(args):
    ts, temps, plug = [], [], []
    t_off = 0.0
    for path in args.logs:
        run = Run(path)
        known = None
        for r in run.rows:
            # Unknown (-1) means a read went unanswered, not that the relay
            # moved: carry the last confirmed state forward.
            if r["plug"] >= 0:
                known = r["plug"]
            if r["temp"] < -100 or known is None:
                continue
            ts.append(r["t"] + t_off)
            temps.append(r["temp"])
            plug.append(known)
        t_off = (ts[-1] + 1000.0) if ts else 0.0   # keep separate logs apart
    t_room = args.t_room

    def u_at(t):
        # Confirmed plug state at time t (step function), 0 before the log.
        lo, hi = 0, len(ts) - 1
        if t < ts[0]:
            return 0.0
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if ts[mid] <= t:
                lo = mid
            else:
                hi = mid - 1
        return plug[lo]

    # Smoothed derivative: regression over +-3 s, never across a log boundary.
    half = 6
    deriv = []
    for k in range(len(ts)):
        lo, hi = max(0, k - half), min(len(ts), k + half + 1)
        tt, vv = ts[lo:hi], temps[lo:hi]
        keep = [i for i, x in enumerate(tt) if abs(x - ts[k]) < 10]
        tt, vv = [tt[i] for i in keep], [vv[i] for i in keep]
        mt, mv = sum(tt) / len(tt), sum(vv) / len(vv)
        den = sum((x - mt) ** 2 for x in tt)
        deriv.append(sum((x - mt) * (v - mv) for x, v in zip(tt, vv)) / den if den else 0.0)

    best = None
    theta = 0.0
    while theta <= args.max_theta + 1e-9:
        for tau_e in [0, 1, 2, 3, 4, 5, 6, 8, 10, 12, 15, 20]:
            x = element_input(ts, u_at, theta, tau_e)
            X, y = [], []
            for k in range(half, len(ts) - half):
                d = temps[k] - t_room
                row = [x[k], -d]
                if args.quad:
                    row.append(-d * max(d, 0))
                X.append(row)
                y.append(deriv[k])
            sol = lstsq(X, y)
            if sol and args.quad and sol[1] < 1e-4:
                # Loss is ~quadratic here. Pin the linear term to a small
                # floor (the firmware model divides by it) and refit the rest.
                b_fix = 1e-4
                y2 = [v + b_fix * (-r[1]) for v, r in zip(y, X)]
                s2 = lstsq([[r[0], r[2]] for r in X], y2)
                sol = [s2[0], b_fix, s2[1]] if s2 else None
            if args.debug and sol:
                print(f"    theta {theta:.1f} tau_e {tau_e:g}: " + " ".join(f"{v:.6g}" for v in sol))
            if not sol or sol[0] <= 0 or sol[1] <= 0:
                continue
            a, b = sol[0], sol[1]
            c = sol[2] if args.quad else 0.0
            # Simulate each log separately from its own first sample.
            err, n, k0 = 0.0, 0, 0
            for k in range(1, len(ts) + 1):
                if k == len(ts) or ts[k] - ts[k - 1] > 100:
                    sim = simulate(ts[k0:k], x[k0:k], temps[k0], a, b, c, t_room)
                    err += sum((s_ - m) ** 2 for s_, m in zip(sim, temps[k0:k]))
                    n += k - k0
                    k0 = k
            rmse = math.sqrt(err / n)
            if best is None or rmse < best[0]:
                best = (rmse, theta, tau_e, a, b, c)
        theta += 0.5
    if not best:
        sys.exit("fit failed: not enough excitation (need heating and cooling)")
    rmse, theta, tau_e, a, b, c = best
    K, tau = a / b, 1 / b
    loss_quad = c / b
    th_eff = theta + tau_e          # FOPDT-equivalent dead time for tuning
    tc = max(th_eff, 1.0)
    kp = tau / (K * (tc + th_eff))
    ti = min(tau, 4 * (tc + th_eff))
    print(f"Model fit over {len(ts)} samples, t_room {t_room:g} C:")
    print(f"  gross heating K/tau = {a:.3f} C/s at full power, loss 1/tau = {b:.5f} /s")
    print(f"  K = {K:.0f} C   tau = {tau:.0f} s   dead time {theta:.1f} s + element lag "
          f"{tau_e:g} s   loss_quad = {loss_quad:.6f}   sim RMSE {rmse:.2f} C")
    print(f"  SIMC on theta_eff = {th_eff:.1f} s (tau_c = theta_eff): kp = {kp:.4f}  ti = {ti:.0f} s")
    print("Apply with:")
    for k, v in (("K", K), ("tau", tau), ("theta", th_eff), ("loss_quad", loss_quad),
                 ("t_room", t_room), ("kp", kp), ("ti", ti), ("lookahead", th_eff),
                 ("coast", tau_e)):
        print(f'  python controller/tools/hil.py cmd "ctl set {k} {v:.5g}"')


# ── CLI ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    sub = ap.add_subparsers(dest="cmd", required=True)

    def rec(p):
        p.add_argument("--duration", type=float, default=0, help="send stop after this many s")
        p.add_argument("--after", type=float, default=5, help="keep logging this many s after the run ends")

    p = sub.add_parser("run", help="run a profile and record it")
    p.add_argument("profile")
    rec(p)
    p = sub.add_parser("step", help="open-loop duty step and record it")
    p.add_argument("duty", type=float)
    p.add_argument("secs", type=float)
    p.add_argument("max_temp", type=float)
    rec(p)
    p = sub.add_parser("hold", help="hold a temperature and record it")
    p.add_argument("temp", type=float)
    rec(p)
    p = sub.add_parser("log", help="record idle telemetry (e.g. a cool-down)")
    rec(p)
    p = sub.add_parser("cmd", help="send one console command, print the reply")
    p.add_argument("line")
    p.add_argument("--wait", type=float, default=1.5)
    p = sub.add_parser("analyze", help="summarise a recorded run")
    p.add_argument("log")
    p.add_argument("--liquidus", type=float)
    p = sub.add_parser("fit", help="fit the FOPDT model to one or more recordings")
    p.add_argument("logs", nargs="+")
    p.add_argument("--t-room", type=float, default=22.0)
    p.add_argument("--max-theta", type=float, default=40.0)
    p.add_argument("--quad", action="store_true", help="also fit a quadratic loss term")
    p.add_argument("--debug", action="store_true", help="print every candidate solution")
    args = ap.parse_args()

    if args.cmd == "run":
        record(args, f"start {args.profile}", "run")
    elif args.cmd == "step":
        record(args, f"step {args.duty} {args.secs} {args.max_temp}", "step")
    elif args.cmd == "hold":
        record(args, f"hold {args.temp}", "hold")
    elif args.cmd == "log":
        record(args, "status", "log")
    elif args.cmd == "cmd":
        cmd_simple(args)
    elif args.cmd == "analyze":
        cmd_analyze(args)
    elif args.cmd == "fit":
        cmd_fit(args)


if __name__ == "__main__":
    main()
