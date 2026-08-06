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
# Requires matplotlib (auto-installed on first run if missing).

import csv
import os
import subprocess
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
        print("matplotlib not found - attempting a one-time install via pip ...")
        try:
            subprocess.check_call([sys.executable, "-m", "pip", "install", "matplotlib"])
            import matplotlib  # noqa: F401
            return True
        except Exception as e:
            print("  auto-install failed (%s). Run manually:  pip install matplotlib" % e)
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


# --- Load ------------------------------------------------------------------
NO_SHOW = ("--no-show" in sys.argv) or bool(os.environ.get("BMP_NOSHOW"))
NO_CORR = ("--no-corr" in sys.argv)   # drop the BMP581 corrected-pressure panel + its temp cal
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
