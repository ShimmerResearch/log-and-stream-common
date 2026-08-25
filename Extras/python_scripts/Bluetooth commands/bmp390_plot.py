#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# BMP390 live PLOT + CSV logger - the BMP390 counterpart of bmp581_plot.py, for
# running a fridge test in parallel and seeing the real difference.
#
# UNLIKE the BMP581 (which self-compensates on-chip and only needs raw/64), the
# BMP390 streams RAW UNCOMPENSATED pressure/temperature plus 21 NVM calibration
# coefficients that the HOST must apply (Bosch BMP3 floating-point compensation,
# BST-BMP390-DS002 sec 8). This script reads those coefficients over BT
# (GET_PRESSURE_CALIBRATION_COEFFICIENTS, 0xA7 -> 0xA6 response) and applies the
# full polynomial, so the plotted pressure/temperature are properly compensated.
#
# ROBUSTNESS: survives Bluetooth dropouts. If the link stalls or errors it closes
# the port, reconnects, and re-issues the streaming commands automatically (up to
# MAX_RECONNECTS times). Every sample is flushed to CSV immediately, so captured
# data is safe even on a hard crash. Calibration is read once and reused across
# reconnects.
#
# Usage (same as bmp581_plot.py):
#   bmp390_plot.py COM<n>          bmp390_plot.py <MAC>          bmp390_plot.py --list
#
# Requires pyserial; requires matplotlib (pip install matplotlib).

import bisect
import os
import re
import struct
import sys
import time

import serial
import serial.tools.list_ports


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
else:
    print("Continuing without plots - data still logged to bmp390_stream.csv")

# --- User settings ---------------------------------------------------------
DEFAULT_MAC = ""
DESIRED_SAMPLING_FREQ_HZ = 51.2
PLOT_WINDOW_S = 120.0   # rolling x-axis window (fridge runs are minutes long)

# --- Robustness settings ---------------------------------------------------
SERIAL_TIMEOUT_S = 1.0     # per-read timeout (keeps the GUI responsive)
STALL_TIMEOUT_S = 12.0     # no bytes for this long -> treat link as stalled
RECONNECT_WAIT_S = 3.0     # pause between reconnect attempts
MAX_RECONNECTS = 20        # give up after this many consecutive failures
ACK_TIMEOUT_S = 4.0        # how long to wait for a command ACK

# --- Shimmer BT bytes ------------------------------------------------------
ACK = 0xFF
NACK = 0xFE
GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND = 0xA7
PRESSURE_CALIBRATION_COEFFICIENTS_RESPONSE = 0xA6
PRESSURE_SENSOR_BMP581 = 3  # sensor-id byte in the 0xA6 response
SET_SENSORS_COMMAND = 0x08
SET_SAMPLING_RATE_COMMAND = 0x05
START_STREAMING_COMMAND = 0x07
STOP_STREAMING_COMMAND = 0x20
SENSORS0 = 0x00
SENSORS1 = 0x00
SENSORS2 = 0x04  # pressure & temperature


# --- Port / MAC helpers (same as bmp581_plot.py) ---------------------------
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


# --- little-endian integer helpers -----------------------------------------
def u24_le(b0, b1, b2):
    return b0 | (b1 << 8) | (b2 << 16)


def u16_le(b):
    return b[0] | (b[1] << 8)


def s16_le(b):
    v = u16_le(b)
    return v - 0x10000 if v >= 0x8000 else v


def s8(b):
    return b - 0x100 if b >= 0x80 else b


def wait_for_ack(ser, timeout=ACK_TIMEOUT_S):
    """Wait up to `timeout` s for a 0xFF ACK. Returns True/False (never hangs)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        d = ser.read(1)
        if d and d[0] == ACK:
            return True
    return False


# --- BMP390 calibration: read 21 NVM coefficients and convert to floats -----
def read_bmp390_calib(ser):
    """Send 0xA7, expect 0xFF 0xA6 <len> <id> <21 bytes>. Returns a dict of par_*.
    A BMP581 unit answers with sensor id 3 and NO coefficient bytes (older
    interim DEV-818 firmware NACK'd instead) - both raise with a pointer to
    bmp581_plot.py."""
    ser.write(struct.pack('B', GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND))
    deadline = time.monotonic() + 3.0
    resp_seen = False
    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b[0] == NACK:
            raise RuntimeError("device NACK'd (0xFE) the calibration command - this is a "
                               "BMP581 (self-compensating, interim DEV-818 firmware). "
                               "Use bmp581_plot.py instead.")
        if b[0] == PRESSURE_CALIBRATION_COEFFICIENTS_RESPONSE:  # 0xA6
            resp_seen = True
            break
        # else: ACK (0xFF) or stray byte - keep scanning
    if not resp_seen:
        raise RuntimeError("no 0xA6 calibration response received (timeout).")
    length = ser.read(1)[0]              # 1 (sensor id) + 21 (calib) = 22
    payload = ser.read(length)
    sensor_id = payload[0]
    if sensor_id == PRESSURE_SENSOR_BMP581:
        raise RuntimeError("device reports sensor id 3 (BMP581, self-compensating, "
                           "no coefficients). Use bmp581_plot.py instead.")
    c = payload[1:1 + 21]                # 21 raw NVM bytes, register order 0x31..0x45
    if len(c) < 21:
        raise RuntimeError("short calibration payload (%d bytes)." % len(c))

    nvm_t1 = u16_le(c[0:2])              # unsigned
    nvm_t2 = u16_le(c[2:4])              # unsigned
    nvm_t3 = s8(c[4])
    nvm_p1 = s16_le(c[5:7])
    nvm_p2 = s16_le(c[7:9])
    nvm_p3 = s8(c[9])
    nvm_p4 = s8(c[10])
    nvm_p5 = u16_le(c[11:13])           # unsigned
    nvm_p6 = u16_le(c[13:15])           # unsigned
    nvm_p7 = s8(c[15])
    nvm_p8 = s8(c[16])
    nvm_p9 = s16_le(c[17:19])
    nvm_p10 = s8(c[19])
    nvm_p11 = s8(c[20])

    par = {
        't1': nvm_t1 / 2.0 ** -8,   't2': nvm_t2 / 2.0 ** 30,   't3': nvm_t3 / 2.0 ** 48,
        'p1': (nvm_p1 - 2.0 ** 14) / 2.0 ** 20,
        'p2': (nvm_p2 - 2.0 ** 14) / 2.0 ** 29,
        'p3': nvm_p3 / 2.0 ** 32,    'p4': nvm_p4 / 2.0 ** 37,
        'p5': nvm_p5 / 2.0 ** -3,    'p6': nvm_p6 / 2.0 ** 6,
        'p7': nvm_p7 / 2.0 ** 8,     'p8': nvm_p8 / 2.0 ** 15,
        'p9': nvm_p9 / 2.0 ** 48,    'p10': nvm_p10 / 2.0 ** 48,
        'p11': nvm_p11 / 2.0 ** 65,
    }
    print("calibration read (sensor id 0x%02x): 21 BMP3 coefficients OK." % sensor_id)
    return par


def compensate_temperature(uncomp_temp, par):
    d1 = uncomp_temp - par['t1']
    d2 = d1 * par['t2']
    return d2 + (d1 * d1) * par['t3']          # t_lin, degC


def compensate_pressure(uncomp_press, t_lin, par):
    d1 = par['p6'] * t_lin
    d2 = par['p7'] * (t_lin * t_lin)
    d3 = par['p8'] * (t_lin * t_lin * t_lin)
    out1 = par['p5'] + d1 + d2 + d3
    d1 = par['p2'] * t_lin
    d2 = par['p3'] * (t_lin * t_lin)
    d3 = par['p4'] * (t_lin * t_lin * t_lin)
    out2 = uncomp_press * (par['p1'] + d1 + d2 + d3)
    d1 = uncomp_press * uncomp_press
    d2 = par['p9'] + par['p10'] * t_lin
    d3 = d1 * d2
    d4 = d3 + (uncomp_press * uncomp_press * uncomp_press) * par['p11']
    return out1 + out2 + d4                     # Pa


def open_serial(port):
    ser = serial.Serial(port, 115200, timeout=SERIAL_TIMEOUT_S)
    ser.flushInput()
    return ser


def start_streaming(ser):
    """Handshake (SET_SENSORS / rate / START). Calibration is read separately."""
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
        pad = max(minpad, (max(vals) - min(vals)) * 0.2)
        ax.set_ylim(min(vals) - pad, max(vals) + pad)


def save_summary_png(t_all, p_all, temp_all, slope, r2, port):
    if not HAVE_MPL or len(t_all) < 2:
        return None
    fig, ax = plt.subplots(3, 1, figsize=(9, 10))
    ax[0].plot(t_all, p_all, lw=0.5, color="tab:green")
    ax[0].set_ylabel("Pressure (kPa)")
    ax[0].set_title("BMP390 host-compensated  %s  (N=%d, %.0fs, slope %+.1f Pa/degC, R2=%.2f)"
                    % (port, len(t_all), t_all[-1] - t_all[0], slope, r2))
    ax[1].plot(t_all, temp_all, lw=0.5, color="tab:red")
    ax[1].set_ylabel("Temp (degC)")
    ax[1].set_xlabel("elapsed s")
    ax[2].scatter(temp_all, p_all, s=2, color="tab:purple")
    ax[2].set_xlabel("Temp (degC)")
    ax[2].set_ylabel("Pressure (kPa)")
    ax[2].set_title("Pressure vs Temperature")
    fig.tight_layout()
    png = os.path.abspath("bmp390_plot.png")
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

par = read_bmp390_calib(ser)   # raises with a clear message if a BMP581 is on this port

if not start_streaming(ser):
    print("streaming did not start cleanly - continuing to read anyway.")
print("streaming started at %.1f Hz (BMP390, host-compensated). %s\n"
      % (DESIRED_SAMPLING_FREQ_HZ,
         ("Close the plot window or press Ctrl+C to stop." if HAVE_MPL
          else "Press Ctrl+C to stop; data -> bmp390_stream.csv.")))

csv_path = os.path.abspath("bmp390_stream.csv")
csv_file = open(csv_path, "w")
csv_file.write("elapsed_s,pressure_kPa,temperature_C,raw_press,raw_temp\n")

t_all, p_all, temp_all = [], [], []

live = False
if HAVE_MPL:
    try:
        plt.ion()
        fig, (axp, axt) = plt.subplots(2, 1, figsize=(9, 8), sharex=True)
        (ln_p,) = axp.plot([], [], color="tab:green", lw=0.8)
        (ln_t,) = axt.plot([], [], color="tab:red", lw=0.8)
        axp.set_ylabel("Pressure (kPa)")
        axt.set_ylabel("Temperature (degC)")
        axt.set_xlabel("elapsed (s)")
        axp.set_title("Shimmer BMP390 host-compensated (live) - %s" % port)
        axp.grid(True, alpha=0.3)
        axt.grid(True, alpha=0.3)
        fig.tight_layout()
        plt.show(block=False)
        live = True
    except Exception as e:
        print("live plot unavailable (%s) - will still save the PNG on exit." % e)

ddata = bytes()
framesize = 10  # 1 type + 3 ts + 3 pressure + 3 temperature (pressure first)
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
                uncomp_press = u24_le(p0, p1, p2)
                uncomp_temp = u24_le(tt0, tt1, tt2)
                t_lin = compensate_temperature(uncomp_temp, par)
                press_pa = compensate_pressure(uncomp_press, t_lin, par)
                pressure_kpa = press_pa / 1000.0
                temperature_c = t_lin
                t = time.monotonic() - t0
                t_all.append(t)
                p_all.append(pressure_kpa)
                temp_all.append(temperature_c)
                csv_file.write("%.4f,%.5f,%.5f,%d,%d\n"
                               % (t, pressure_kpa, temperature_c, uncomp_press, uncomp_temp))
                csv_file.flush()
                i += 1
            except Exception:
                continue  # skip a malformed frame rather than crash

            if live and (i % 5 == 0):
                lo = t - PLOT_WINDOW_S
                start = bisect.bisect_left(t_all, lo)   # t_all is monotonic
                xs = t_all[start:]
                ps = p_all[start:]
                ts = temp_all[start:]
                ln_p.set_data(xs, ps)
                ln_t.set_data(xs, ts)
                axp.set_xlim(max(0, lo), t + 0.5)
                set_ylim_padded(axp, ps, 0.002)
                set_ylim_padded(axt, ts, 0.01)
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
    if len(temp_all) > 2:
        slope, _, r2 = linfit(temp_all, [p * 1000.0 for p in p_all])
        total_pa = (max(p_all) - min(p_all)) * 1000.0
        print("pressure vs temperature: slope = %+.1f Pa/degC   R^2 = %.3f   excursion = %.0f Pa"
              % (slope, r2, total_pa))
        if abs(slope) < 5:
            print("   => BMP390 compensated correctly (near the +/-0.6 Pa/degC datasheet spec).")
        else:
            print("   => residual %.0f Pa/degC (check coefficients / thermal transients)." % slope)
        png = save_summary_png(t_all, p_all, temp_all, slope, r2, port)
        if png:
            print("   plot saved -> %s" % png)
            try:
                plt.ioff(); plt.show()
            except Exception:
                pass
