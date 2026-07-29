#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# BMP581 live PLOT + CSV logger, with an optional host-side TEMPERATURE
# CORRECTION so you can watch raw vs corrected pressure in real time.
#
#   * streams pressure (raw/64) + temperature (signed/65536) over BT,
#   * applies a quadratic temp correction  P_corr = P - (a*(T-Tref)^2 + b*(T-Tref)),
#   * live-plots THREE panels: temperature, raw pressure, corrected pressure,
#   * logs elapsed, raw kPa, corrected kPa, temp, raw counts to bmp581_stream.csv,
#   * on exit prints the raw AND corrected pressure-vs-temperature slope, saves PNG.
#
# ROBUSTNESS: survives Bluetooth dropouts. If the link stalls or errors it closes
# the port, reconnects, and re-issues the streaming commands automatically (up to
# MAX_RECONNECTS times), so a long fridge sweep is not lost to a transient glitch.
# Every sample is flushed to CSV immediately, so data already captured is safe
# even on a hard crash.
#
# The correction coefficients below are PER-UNIT (fit from a fridge sweep). Refit
# for each sensor; set CORRECTION_ENABLE=False to see only the raw comp=3 output.
#
# Usage:  bmp581_plot.py COM56   |   <MAC>   |   --list
# Requires pyserial; matplotlib is auto-installed on first run if missing.

import bisect
import os
import re
import struct
import subprocess
import sys
import time

import serial
import serial.tools.list_ports


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
else:
    print("Continuing without plots - data still logged to bmp581_stream.csv")

# --- User settings ---------------------------------------------------------
DEFAULT_MAC = ""
DESIRED_SAMPLING_FREQ_HZ = 51.2
PLOT_WINDOW_S = 120.0

# --- Robustness settings ---------------------------------------------------
SERIAL_TIMEOUT_S = 1.0     # per-read timeout (keeps the GUI responsive)
STALL_TIMEOUT_S = 12.0     # no bytes for this long -> treat link as stalled
RECONNECT_WAIT_S = 3.0     # pause between reconnect attempts
MAX_RECONNECTS = 20        # give up after this many consecutive failures
ACK_TIMEOUT_S = 4.0        # how long to wait for a command ACK

# --- Host-side pressure temperature-correction (PER-UNIT; fit from a fridge sweep)
# P_corr = P_meas - (CORR_C*dt^3 + CORR_A*dt^2 + CORR_B*dt),  dt = T_raw - CORR_TREF  [Pa]
# Fit to the 2026-07-29 15:22 run (BMP581 UID 1600 2988 86DF C7B2) vs the RAW
# on-chip temperature. Cubic gave negligible gain here (the residual is
# noise-limited until the firmware IIR/OSR fix lands) so the default is quadratic;
# for cubic use CORR_C=0.019269, CORR_A=2.65922, CORR_B=450.4511.
# Set CORRECTION_ENABLE=False to plot only the raw comp=3 output.
CORRECTION_ENABLE = True
CORR_C = 0.0          # Pa/degC^3
CORR_A = 1.89211      # Pa/degC^2
CORR_B = 444.3297     # Pa/degC
CORR_TREF = 25.0      # degC
# Constant absolute-alignment offset ADDED to the corrected pressure so it matches
# the co-located BMP390 (the working reference). +1182 Pa (at 25 C) from the
# 2026-07-29 16:52 run; the sensor-to-sensor offset repeated to ~4 Pa at the
# reference temp across 2 runs, but the raw slope drifts ~10% boot-to-boot so this
# is PROVISIONAL pending the power-cycle repeatability test. Set 0.0 to disable.
CORR_OFFSET = 1182.0  # Pa

# --- BMP581 temperature-channel calibration (2-point) -------------------------
# The BMP581 temperature is uncompensated too (same broken on-chip engine as
# pressure): it OVER-reads the swing by ~28%. Interim mapping to true temperature
# derived by aligning this unit against a co-located BMP390
# (T_true ~= 0.782*T_raw + 6.70, RMS 0.76 C). REPLACE with a real 2-point
# measurement: read T_raw in an ice bath (0 C) and at a known warm reference
# against a thermometer, then
#     TEMP_CAL_GAIN   = (T_hi - T_lo) / (Traw_hi - Traw_lo)
#     TEMP_CAL_OFFSET = T_lo - TEMP_CAL_GAIN * Traw_lo
# This ONLY fixes the reported temperature (logged as temperature_cal_C and shown
# on the temp panel); the pressure correction above uses the raw temp and is
# unaffected, so you can re-tune this without refitting the pressure coeffs.
TEMP_CAL_ENABLE = True
TEMP_CAL_GAIN = 0.7822
TEMP_CAL_OFFSET = 6.695

# --- Shimmer BT bytes ------------------------------------------------------
ACK = 0xFF
NACK = 0xFE
GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND = 0xA7
PRESSURE_CALIBRATION_COEFFICIENTS_RESPONSE = 0xA6
SET_SENSORS_COMMAND = 0x08
SET_SAMPLING_RATE_COMMAND = 0x05
START_STREAMING_COMMAND = 0x07
STOP_STREAMING_COMMAND = 0x20
SENSORS0 = 0x00
SENSORS1 = 0x00
SENSORS2 = 0x04  # pressure & temperature


# --- Port / MAC helpers ----------------------------------------------------
def normalize_mac(s):
    return re.sub(r'[^0-9A-Fa-f]', '', s).upper()


def extract_mac_from_hwid(hwid):
    h = re.sub(r'\{[^}]*\}', '', hwid or "").replace(':', '')
    BASE = '00805F9B34FB'
    cands = [c.upper() for c in re.findall(r'([0-9A-Fa-f]{12})', h)
             if set(c.upper()) != {'0'} and c.upper() != BASE]
    return cands[-1] if cands else None


def serial_ports_bluetooth():
    return [p for p in serial.tools.list_ports.comports()
            if "Bluetooth" in (p.description or "")]


def list_bt_ports():
    print("Bluetooth serial ports (MAC | port | description):")
    found = False
    for p in serial.tools.list_ports.comports():
        mac = extract_mac_from_hwid(p.hwid)
        if mac:
            found = True
            print("  %-12s  %-6s  %s" % (mac, p.device, p.description))
    if not found:
        print("  (none found - is the device paired?)")


def pick_com_port_interactive():
    options = serial_ports_bluetooth()
    if not options:
        print("No Bluetooth serial ports found - is the Shimmer paired?")
        return None
    print("Pick an option:")
    for index, item in enumerate(options):
        mac = extract_mac_from_hwid(item.hwid) or "?"
        print("  %d) %s  (MAC %s)  %s" % (index + 1, item.device, mac, item.description))
    ui = ""
    while (not ui.isdigit()) or int(ui) < 1 or int(ui) > len(options):
        ui = input("Your choice: ").strip()
    port = options[int(ui) - 1].device
    print("You picked: %s\n" % port)
    return port


def find_port_by_mac(mac):
    target = normalize_mac(mac)
    outgoing = fallback = None
    for p in serial.tools.list_ports.comports():
        hwid = (p.hwid or "").upper().replace(':', '').replace('-', '')
        if target and target in hwid:
            if '_C00000000' in hwid:
                outgoing = p.device
            elif fallback is None:
                fallback = p.device
    return outgoing or fallback


def resolve_port(arg):
    if not arg:
        arg = DEFAULT_MAC
    if not arg:
        return None
    a = arg.strip()
    if a.upper().startswith('COM') or a.startswith('/dev/'):
        return a
    n = normalize_mac(a)
    if len(n) == 12:
        port = find_port_by_mac(n)
        print(("MAC %s -> %s" % (a, port)) if port else ("No paired port for MAC %s." % a))
        return port
    return a


def u24_le(b0, b1, b2):
    return b0 | (b1 << 8) | (b2 << 16)


def s24_le(b0, b1, b2):
    v = u24_le(b0, b1, b2)
    return v - 0x1000000 if v >= 0x800000 else v


def wait_for_ack(ser, timeout=ACK_TIMEOUT_S):
    """Wait up to `timeout` s for a 0xFF ACK. Returns True/False (never hangs)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        d = ser.read(1)
        if d and d[0] == ACK:
            return True
    return False


def check_calibration_nack(ser):
    try:
        ser.write(struct.pack('B', GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND))
        resp = ser.read(1)
    except Exception as e:
        print("WARNING: calibration probe failed (%s)." % e)
        return
    if len(resp) == 0:
        print("WARNING: no reply to calibration command (timeout).")
        return
    b = resp[0]
    if b == NACK:
        print("calibration command NACK'd (0xFE) - BMP581 pre-compensated. Good.")
    elif b == ACK:
        hdr = ser.read(1)
        if len(hdr) and hdr[0] == PRESSURE_CALIBRATION_COEFFICIENTS_RESPONSE:
            lenb = ser.read(1)
            if len(lenb):
                ser.read(lenb[0])
        print("WARNING: device ACK'd coefficients - streaming raw anyway.")
    else:
        print("WARNING: unexpected reply 0x%02x." % b)


def open_serial(port):
    ser = serial.Serial(port, 115200, timeout=SERIAL_TIMEOUT_S)
    ser.flushInput()
    return ser


def start_streaming(ser):
    """Run the full handshake. Returns True if streaming was (re)started."""
    check_calibration_nack(ser)
    ser.write(struct.pack('BBBB', SET_SENSORS_COMMAND, SENSORS0, SENSORS1, SENSORS2))
    if not wait_for_ack(ser):
        print("WARNING: no ACK for SET_SENSORS.")
    clock_wait = int(32768 / DESIRED_SAMPLING_FREQ_HZ)
    ser.write(struct.pack('<BH', SET_SAMPLING_RATE_COMMAND, clock_wait))
    if not wait_for_ack(ser):
        print("WARNING: no ACK for SET_SAMPLING_RATE.")
    ser.write(struct.pack('B', START_STREAMING_COMMAND))
    if not wait_for_ack(ser):
        print("WARNING: no ACK for START_STREAMING.")
        return False
    return True


def reconnect(port):
    """Close/reopen the port and restart streaming. Returns a serial obj or None."""
    for attempt in range(1, MAX_RECONNECTS + 1):
        print("  reconnect attempt %d/%d ..." % (attempt, MAX_RECONNECTS))
        try:
            time.sleep(RECONNECT_WAIT_S)
            ser = open_serial(port)
            if start_streaming(ser):
                print("  reconnected and streaming again.")
                return ser
            try:
                ser.close()
            except Exception:
                pass
        except Exception as e:
            print("    failed: %s" % e)
    return None


def linfit(x, y):
    n = len(x)
    if n < 2:
        return 0.0, 0.0, 0.0
    mx = sum(x) / n
    my = sum(y) / n
    sxx = sum((xi - mx) ** 2 for xi in x)
    sxy = sum((xi - mx) * (yi - my) for xi, yi in zip(x, y))
    syy = sum((yi - my) ** 2 for yi in y)
    if sxx == 0 or syy == 0:
        return 0.0, my, 0.0
    slope = sxy / sxx
    return slope, my - slope * mx, (sxy * sxy) / (sxx * syy)


def set_ylim_padded(ax, vals, minpad):
    if vals:
        pad = max(minpad, (max(vals) - min(vals)) * 0.15)
        ax.set_ylim(min(vals) - pad, max(vals) + pad)


def correct_pa(pressure_kpa, temperature_c):
    """Host-side temperature correction (cubic-capable). Uses the RAW temp. -> kPa."""
    if not CORRECTION_ENABLE:
        return pressure_kpa
    dt = temperature_c - CORR_TREF
    corr_pa = CORR_C * dt * dt * dt + CORR_A * dt * dt + CORR_B * dt
    return pressure_kpa - corr_pa / 1000.0 + CORR_OFFSET / 1000.0


def calibrate_temp(temperature_c):
    """Map the raw on-chip temperature to true temperature (2-point). -> degC."""
    if not TEMP_CAL_ENABLE:
        return temperature_c
    return TEMP_CAL_GAIN * temperature_c + TEMP_CAL_OFFSET


def save_summary_png(t_all, p_all, pc_all, temp_all, sr, sc, port):
    if not HAVE_MPL or len(t_all) < 2:
        return None
    fig, ax = plt.subplots(3, 1, figsize=(10, 11), sharex=True)
    ax[0].plot(t_all, temp_all, color="tab:red", lw=0.5)
    ax[0].set_ylabel("Temperature (degC)"); ax[0].grid(alpha=0.3)
    ax[0].set_title("BMP581 %s  (N=%d, %.0fs)" % (port, len(t_all), t_all[-1] - t_all[0]))
    ax[1].plot(t_all, p_all, color="tab:blue", lw=0.5)
    ax[1].set_ylabel("Raw pressure (kPa)"); ax[1].grid(alpha=0.3)
    ax[1].set_title("raw comp=3: %+.0f Pa/degC" % sr, fontsize=9, loc="right")
    ax[2].plot(t_all, pc_all, color="tab:green", lw=0.5)
    ax[2].set_ylabel("Corrected pressure (kPa)"); ax[2].set_xlabel("elapsed s"); ax[2].grid(alpha=0.3)
    ax[2].set_title("corrected: %+.1f Pa/degC" % sc, fontsize=9, loc="right")
    fig.tight_layout()
    png = os.path.abspath("bmp581_plot.png")
    fig.savefig(png, dpi=110)
    return png


# --- Main ------------------------------------------------------------------
raw_args = sys.argv[1:]
if any(a in ('--list', '-l', 'list') for a in raw_args):
    list_bt_ports()
    sys.exit(0)

arg = "".join(raw_args).strip() if raw_args else None
port = resolve_port(arg) or pick_com_port_interactive()
if not port:
    print("No device selected. Pass a COM port or MAC (see --list).")
    sys.exit(1)

ser = open_serial(port)
print("port %s opened." % port)
if CORRECTION_ENABLE:
    print("temp correction ON: P_corr = P - (%.4f*(T-%.1f)^2 + %.3f*(T-%.1f))"
          % (CORR_A, CORR_TREF, CORR_B, CORR_TREF))
else:
    print("temp correction OFF (raw comp=3 only).")

if not start_streaming(ser):
    print("streaming did not start cleanly - continuing to read anyway.")
print("streaming at %.1f Hz. %s\n"
      % (DESIRED_SAMPLING_FREQ_HZ,
         ("Close the plot window or press Ctrl+C to stop." if HAVE_MPL
          else "Press Ctrl+C to stop; data -> bmp581_stream.csv.")))

csv_path = os.path.abspath("bmp581_stream.csv")
csv_file = open(csv_path, "w")
csv_file.write("elapsed_s,pressure_kPa,pressure_corr_kPa,temperature_C,temperature_cal_C,raw_press,raw_temp\n")

t_all, p_all, pc_all, temp_all = [], [], [], []

live = False
if HAVE_MPL:
    try:
        plt.ion()
        fig, (axt, axp, axpc) = plt.subplots(3, 1, figsize=(9, 10), sharex=True)
        (ln_t,) = axt.plot([], [], color="tab:red", lw=0.8)
        (ln_p,) = axp.plot([], [], color="tab:blue", lw=0.8)
        (ln_pc,) = axpc.plot([], [], color="tab:green", lw=0.8)
        axt.set_ylabel("Temperature (degC, cal)" if TEMP_CAL_ENABLE else "Temperature (degC)")
        axp.set_ylabel("Raw pressure (kPa)")
        axpc.set_ylabel("Corrected pressure (kPa)")
        axpc.set_xlabel("elapsed (s)")
        axt.set_title("Shimmer BMP581 live - temperature / raw / corrected  (%s)" % port)
        for a in (axt, axp, axpc):
            a.grid(True, alpha=0.3)
        fig.tight_layout()
        plt.show(block=False)
        live = True
    except Exception as e:
        print("live plot unavailable (%s) - will still save the PNG on exit." % e)

ddata = bytes()
framesize = 10
t0 = time.monotonic()
i = 0
last_rx = time.monotonic()
stopped_clean = False
try:
    while True:
        # --- resilient read ------------------------------------------------
        try:
            navail = ser.in_waiting
        except Exception:
            navail = 0
        try:
            chunk = ser.read(navail if navail > framesize else framesize)
        except (serial.SerialException, OSError) as e:
            print("\nserial error: %s" % e)
            try:
                ser.close()
            except Exception:
                pass
            ser = reconnect(port)
            if ser is None:
                print("could not reconnect - stopping and saving.")
                break
            ddata = bytes()
            last_rx = time.monotonic()
            continue

        now = time.monotonic()
        if chunk:
            ddata += chunk
            last_rx = now
        elif now - last_rx > STALL_TIMEOUT_S:
            print("\nno data for %.0fs - link stalled, reconnecting ..." % (now - last_rx))
            try:
                ser.close()
            except Exception:
                pass
            ser = reconnect(port)
            if ser is None:
                print("could not reconnect - stopping and saving.")
                break
            ddata = bytes()
            last_rx = time.monotonic()
            continue

        # --- parse whatever complete frames we have ------------------------
        while len(ddata) >= framesize:
            frame = ddata[0:framesize]
            ddata = ddata[framesize:]
            try:
                (p0, p1, p2, tt0, tt1, tt2) = struct.unpack('BBBBBB', frame[4:framesize])
                pressure_kpa = u24_le(p0, p1, p2) / 64000.0
                temperature_c = s24_le(tt0, tt1, tt2) / 65536.0
                temperature_cal = calibrate_temp(temperature_c)
                pressure_corr_kpa = correct_pa(pressure_kpa, temperature_c)
                t = time.monotonic() - t0
                t_all.append(t)
                p_all.append(pressure_kpa)
                pc_all.append(pressure_corr_kpa)
                temp_all.append(temperature_cal)   # plot/report the calibrated temp
                csv_file.write("%.4f,%.5f,%.5f,%.5f,%.5f,%d,%d\n"
                               % (t, pressure_kpa, pressure_corr_kpa, temperature_c,
                                  temperature_cal, u24_le(p0, p1, p2), s24_le(tt0, tt1, tt2)))
                csv_file.flush()
                i += 1
            except Exception:
                continue  # skip a malformed frame rather than crash

            if live and (i % 5 == 0):
                lo = t - PLOT_WINDOW_S
                start = bisect.bisect_left(t_all, lo)   # t_all is monotonic
                xs = t_all[start:]
                ps = p_all[start:]
                pcs = pc_all[start:]
                ts = temp_all[start:]
                ln_t.set_data(xs, ts)
                ln_p.set_data(xs, ps)
                ln_pc.set_data(xs, pcs)
                axpc.set_xlim(max(0, lo), t + 0.5)
                set_ylim_padded(axt, ts, 0.01)
                set_ylim_padded(axp, ps, 0.002)
                set_ylim_padded(axpc, pcs, 0.002)
                try:
                    plt.pause(0.001)
                    if not plt.fignum_exists(fig.number):
                        stopped_clean = True
                        break
                except Exception:
                    live = False
        if stopped_clean:
            break

except KeyboardInterrupt:
    pass
finally:
    try:
        ser.write(struct.pack('B', STOP_STREAMING_COMMAND))
        wait_for_ack(ser, timeout=1.0)
    except Exception:
        pass
    try:
        ser.close()
    except Exception:
        pass
    csv_file.close()
    print("\nstopped. %d samples -> %s" % (len(t_all), csv_path))
    sr = sc = 0.0
    if len(temp_all) > 2:
        sr, _, r2r = linfit(temp_all, [p * 1000.0 for p in p_all])
        sc, _, r2c = linfit(temp_all, [p * 1000.0 for p in pc_all])
        print("pressure vs temperature slope:")
        print("   RAW comp=3   : %+.1f Pa/degC  (R2=%.3f)" % (sr, r2r))
        if CORRECTION_ENABLE:
            print("   CORRECTED    : %+.2f Pa/degC  (R2=%.3f)  <- should be near 0" % (sc, r2c))
        png = save_summary_png(t_all, p_all, pc_all, temp_all, sr, sc, port)
        if png:
            print("   plot saved -> %s" % png)
            try:
                plt.ioff(); plt.show()
            except Exception:
                pass
