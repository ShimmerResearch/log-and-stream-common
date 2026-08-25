#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# Combine the outputs of bmp581_plot.py and bmp390_plot.py into ONE figure.
#
# Run the two live scripts (ideally started together) during a fridge test:
#     python bmp581_plot.py COM<a>      -> bmp581_stream.csv
#     python bmp390_plot.py COM<b>      -> bmp390_stream.csv
# then run this to overlay them:
#     python bmp_compare.py
#     python bmp_compare.py <bmp581_csv> <bmp390_csv>     # explicit paths
#
# The result is FIVE stacked panels, each on its OWN independent y-scale, sharing
# the elapsed-time axis:
#   1) BMP581 temperature
#   2) BMP581 original pressure (raw comp=3)
#   3) BMP581 corrected pressure
#   4) BMP390 temperature
#   5) BMP390 pressure (host-compensated)
# Independent scales mean a well-corrected BMP581 and the BMP390 both look flat
# even though the two chips sit at different absolute offsets ("not very off").
#
# Requires matplotlib (pip install matplotlib).

import csv
import os
import sys

DEFAULT_581 = "bmp581_stream.csv"
DEFAULT_390 = "bmp390_stream.csv"

COL_RAW = "tab:blue"
COL_COR = "tab:green"
COL_390 = "tab:red"


def _ensure_matplotlib():
    try:
        import matplotlib  # noqa: F401
        return True
    except Exception:
        print("matplotlib not found - install it with:  pip install matplotlib")
        return False


HAVE_MPL = _ensure_matplotlib()
if HAVE_MPL:
    import matplotlib
    try:
        matplotlib.use("TkAgg")
    except Exception:
        pass
    import matplotlib.pyplot as plt


def find_csv(name):
    """Look in cwd first, then next to this script."""
    if os.path.isfile(name):
        return os.path.abspath(name)
    here = os.path.join(os.path.dirname(os.path.abspath(__file__)), name)
    return here if os.path.isfile(here) else None


def read_stream_csv(path):
    """Read a bmpXXX_stream.csv into {column_name: [floats]} keyed by header."""
    cols = {}
    with open(path, "r") as f:
        reader = csv.reader(f)
        header = [h.strip() for h in next(reader)]
        for h in header:
            cols[h] = []
        for row in reader:
            if len(row) < len(header):
                continue
            vals = []
            ok = True
            for v in row[:len(header)]:
                try:
                    vals.append(float(v))
                except ValueError:
                    ok = False
                    break
            if not ok:
                continue
            for h, v in zip(header, vals):
                cols[h].append(v)
    return cols


def col(cols, *names):
    for n in names:
        if n in cols and cols[n]:
            return cols[n]
    return None


def linfit(x, y):
    n = min(len(x), len(y))
    if n < 2:
        return 0.0, 0.0
    x = x[:n]; y = y[:n]
    mx = sum(x) / n
    my = sum(y) / n
    sxx = sum((xi - mx) ** 2 for xi in x)
    sxy = sum((xi - mx) * (yi - my) for xi, yi in zip(x, y))
    syy = sum((yi - my) ** 2 for yi in y)
    if sxx == 0 or syy == 0:
        return 0.0, 0.0
    return sxy / sxx, (sxy * sxy) / (sxx * syy)


def report(label, temp, press_kpa):
    if not temp or not press_kpa:
        return
    slope, r2 = linfit(temp, [p * 1000.0 for p in press_kpa])
    span = (max(press_kpa) - min(press_kpa)) * 1000.0
    mean = sum(press_kpa) / len(press_kpa)
    print("  %-24s slope %+8.2f Pa/degC  R2=%.4f  span=%6.0f Pa  mean=%.3f kPa"
          % (label, slope, r2, span, mean))


def pressure_agreement(t581, p581_kpa, t390, p390_kpa, out_png, no_show):
    """Pressure-tracking check (for a room-temperature pressure test, e.g. a climb
    up/down): time-align the two streams via cross-correlation, then verify the
    BMP581 tracks the BMP390 1:1. Prints slope/R2/offset and saves a 2-panel figure
    (pressure vs aligned time + BMP581-vs-BMP390 scatter). Uses RAW BMP581 pressure."""
    import numpy as np
    a_t = np.asarray(t581, float); a_p = np.asarray(p581_kpa, float) * 1000.0   # Pa
    b_t = np.asarray(t390, float); b_p = np.asarray(p390_kpa, float) * 1000.0
    dt = 0.2  # resample to 5 Hz - ample for pressure steps, keeps xcorr cheap
    ga = np.arange(a_t[0], a_t[-1], dt); pa = np.interp(ga, a_t, a_p)
    gb = np.arange(b_t[0], b_t[-1], dt); pb = np.interp(gb, b_t, b_p)
    # cross-correlate (mean-removed) to find the BMP581->BMP390 time lag, searching a
    # bounded +/-60 s window around zero (the two scripts are started together)
    a0 = pa - pa.mean(); b0 = pb - pb.mean()
    full = np.correlate(b0, a0, mode="full")
    zero = len(a0) - 1
    w = int(60.0 / dt)
    lo_i = max(0, zero - w); hi_i = min(len(full), zero + w + 1)
    lag = (lo_i + int(np.argmax(full[lo_i:hi_i])) - zero) * dt   # add to a_t to align to b
    # pair on the overlapping, aligned grid
    ta = ga + lag
    lo = max(ta[0], gb[0]); hi = min(ta[-1], gb[-1])
    grid = np.arange(lo, hi, dt)
    P581 = np.interp(grid, ta, pa)
    P390 = np.interp(grid, gb, pb)
    # fit BMP581 = slope*BMP390 + offset
    slope, intercept = np.polyfit(P390, P581, 1)
    resid = P581 - (slope * P390 + intercept)
    ss_res = float(np.sum(resid ** 2)); ss_tot = float(np.sum((P581 - P581.mean()) ** 2))
    r2 = (1.0 - ss_res / ss_tot) if ss_tot else 0.0
    rng = float(P390.max() - P390.min())
    print("\nPressure agreement (BMP581 vs BMP390, time-aligned, lag %.1f s):" % lag)
    print("  slope        = %.4f   (1.000 = BMP581 tracks real pressure 1:1)" % slope)
    print("  R2           = %.5f" % r2)
    print("  offset       = %+.0f Pa  (BMP581 - BMP390)" % intercept)
    print("  residual sd  = %.1f Pa    applied pressure range = %.0f Pa" % (resid.std(), rng))
    if rng < 30.0:
        print("  NOTE: applied range < 30 Pa - climb more / bigger steps for a reliable slope.")
    if not HAVE_MPL:
        return
    fig, ax = plt.subplots(2, 1, figsize=(9, 8))
    ax[0].plot(grid, P581 - P581.mean(), color="tab:blue", lw=0.7, label="BMP581 (raw)")
    ax[0].plot(grid, P390 - P390.mean(), color="tab:green", lw=0.7, label="BMP390 (ref)")
    ax[0].set_xlabel("elapsed (s, aligned)"); ax[0].set_ylabel("Pressure, mean-centred (Pa)")
    ax[0].grid(alpha=0.3); ax[0].legend(fontsize=8)
    ax[0].set_title("Pressure vs time - BMP581 should step in lockstep with BMP390")
    ax[1].scatter(P390, P581, s=2, color="tab:blue")
    xs = np.array([P390.min(), P390.max()])
    ax[1].plot(xs, slope * xs + intercept, color="tab:red", lw=1.2,
               label="fit: slope=%.3f, R2=%.4f" % (slope, r2))
    ax[1].plot(xs, xs + (P581.mean() - P390.mean()), color="0.6", ls="--", lw=1.0,
               label="ideal (slope 1)")
    ax[1].set_xlabel("BMP390 pressure (Pa)"); ax[1].set_ylabel("BMP581 pressure (Pa)")
    ax[1].grid(alpha=0.3); ax[1].legend(fontsize=8)
    ax[1].set_title("BMP581 vs BMP390 pressure - slope ~1 = correct magnitude", fontsize=9)
    fig.tight_layout(); fig.savefig(out_png, dpi=120)
    print("  plot saved -> %s" % out_png)
    if not no_show:
        plt.show()


# --- Load ------------------------------------------------------------------
NO_SHOW = ("--no-show" in sys.argv) or bool(os.environ.get("BMP_NOSHOW"))
NO_CORR = ("--no-corr" in sys.argv)   # drop the BMP581 corrected-pressure panel + its temp cal
PRESSURE_MODE = ("--pressure" in sys.argv)  # pressure-tracking check (climb test) instead of TCO
args = [a for a in sys.argv[1:] if a and not a.startswith("-")]
p581 = args[0] if len(args) >= 1 else DEFAULT_581
p390 = args[1] if len(args) >= 2 else DEFAULT_390
path581 = find_csv(p581)
path390 = find_csv(p390)

if not path581 and not path390:
    print("No input CSVs found. Expected %s and/or %s in this folder." % (DEFAULT_581, DEFAULT_390))
    print("Run bmp581_plot.py / bmp390_plot.py first, or pass paths as arguments.")
    sys.exit(1)

d581 = read_stream_csv(path581) if path581 else {}
d390 = read_stream_csv(path390) if path390 else {}

t581 = col(d581, "elapsed_s")
p581_raw = col(d581, "pressure_kPa")
p581_cor = col(d581, "pressure_corr_kPa") or p581_raw   # fall back if not corrected
# Prefer the calibrated temp, unless --no-corr (then the 2678-tuned cal doesn't apply)
temp581 = (col(d581, "temperature_C") if NO_CORR
           else col(d581, "temperature_cal_C", "temperature_C"))

t390 = col(d390, "elapsed_s")
p390_c = col(d390, "pressure_kPa")
temp390 = col(d390, "temperature_C")

print("\nInputs:")
if path581:
    print("  BMP581: %s  (%d samples)" % (path581, len(t581 or [])))
if path390:
    print("  BMP390: %s  (%d samples)" % (path390, len(t390 or [])))

if PRESSURE_MODE:
    if not (t581 and p581_raw and t390 and p390_c):
        print("\n--pressure needs both the BMP581 and BMP390 streams.")
        sys.exit(1)
    pressure_agreement(t581, p581_raw, t390, p390_c,
                       os.path.abspath("bmp_pressure_agreement.png"), NO_SHOW)
    sys.exit(0)

print("\nPressure vs temperature (flatness):")
report("BMP581 raw comp=3", temp581, p581_raw)
if col(d581, "pressure_corr_kPa") and not NO_CORR:
    report("BMP581 corrected", temp581, p581_cor)
report("BMP390 host-compensated", temp390, p390_c)

# Absolute offset of the corrected BMP581 vs the BMP390 at the 25 C reference.
# Weather cancels (co-located), so this is the run-to-run repeatability metric:
# tabulate it across reboots - near-constant => a fixed offset holds.
def value_at(temp, p_kpa, tref=25.0):
    """Linear-fit pressure (Pa) evaluated at tref."""
    pa = [p * 1000.0 for p in p_kpa]
    s, _ = linfit(temp, pa)
    mt = sum(temp) / len(temp)
    mp = sum(pa) / len(pa)
    return mp - s * (mt - tref)


if temp581 and p581_cor and temp390 and p390_c and not NO_CORR:
    off = value_at(temp390, p390_c) - value_at(temp581, p581_cor)
    print("\nAbsolute offset at 25 C:  BMP390 - corrected BMP581 = %+.0f Pa" % off)
    print("   (~0 means the CORR_OFFSET in bmp581_plot.py is aligned; track this across reboots)")

if not HAVE_MPL:
    print("\n(matplotlib unavailable - stats only.)")
    sys.exit(0)


# --- Plot ------------------------------------------------------------------
# Five stacked panels, each on its OWN independent y-scale, sharing the time axis:
#   1) BMP581 temperature
#   2) BMP581 original pressure (raw comp=3)
#   3) BMP581 corrected pressure
#   4) BMP390 temperature
#   5) BMP390 pressure (host-compensated)
# For a pressure panel, its pressure-vs-temperature slope is shown at the right.
COL_T = "tab:red"
has_corr = bool(col(d581, "pressure_corr_kPa"))


def slope_note(temp, press_kpa):
    if not temp or not press_kpa:
        return ""
    s, r2 = linfit(temp, [p * 1000.0 for p in press_kpa])
    return "%+.1f Pa/degC  R2=%.3f" % (s, r2)


# (title, ylabel, x, y, colour, temp_for_slope_or_None)
panels = []
if t581 and temp581:
    panels.append(("BMP581 temperature", "Temp (degC)", t581, temp581, COL_T, None))
if t581 and p581_raw:
    panels.append(("BMP581 original pressure (raw comp=3)", "Pressure (kPa)",
                   t581, p581_raw, COL_RAW, temp581))
if t581 and has_corr and p581_cor and not NO_CORR:
    panels.append(("BMP581 corrected pressure", "Pressure (kPa)",
                   t581, p581_cor, COL_COR, temp581))
if t390 and temp390:
    panels.append(("BMP390 temperature", "Temp (degC)", t390, temp390, COL_T, None))
if t390 and p390_c:
    panels.append(("BMP390 pressure (compensated)", "Pressure (kPa)",
                   t390, p390_c, COL_390, temp390))

if not panels:
    print("Nothing to plot - CSVs had no usable columns.")
    sys.exit(1)

n = len(panels)
fig, axes = plt.subplots(n, 1, figsize=(11, 2.2 * n), sharex=True)
if n == 1:
    axes = [axes]

for ax, (title, ylabel, x, y, colour, temp_ref) in zip(axes, panels):
    ax.plot(x, y, color=colour, lw=0.7)
    ax.set_ylabel(ylabel)
    note = slope_note(temp_ref, y) if temp_ref is not None else ""
    ax.set_title(title + (("   [%s]" % note) if note else ""), fontsize=9, loc="left")
    ax.grid(True, alpha=0.3)

axes[-1].set_xlabel("elapsed (s)  [BMP581 and BMP390 line up only if both scripts started together]")
fig.suptitle("BMP581 vs BMP390 - combined (independent scales)", fontsize=11)
fig.tight_layout(rect=(0, 0, 1, 0.99))
out = os.path.abspath("bmp_compare.png")
fig.savefig(out, dpi=110)
print("\ncombined plot (%d panels) saved -> %s" % (n, out))
if not NO_SHOW:
    plt.show()
