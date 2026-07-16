#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# BMP581 Bluetooth streaming test (Shimmer3R).
#
# You can pass EITHER a serial port OR the Shimmer's Bluetooth MAC address - the
# script resolves a MAC to its outgoing COM port automatically:
#   bmp581.py COM28                     # explicit port
#   bmp581.py 00:06:66:B1:4B:B7         # MAC (colons optional)
#   bmp581.py 000666B14BB7              # MAC, no separators
#   bmp581.py --list                    # list all BT serial ports + their MACs
# Or set DEFAULT_MAC below and run with no argument.
#
# Flow:
#   1. Send GET_PRESSURE_CALIBRATION_COEFFICIENTS (0xA7). A BMP581 self-compensates
#      and has NO calibration coefficients, so the firmware replies with a NACK
#      (0xFE). This script expects that NACK and proceeds - no coefficients needed.
#   2. Enable pressure + temperature, set the sampling rate, start streaming.
#   3. Convert each sample straight from the RAW 24-bit values (no host trimming):
#        pressure    (uint24 little-endian)  Pa   = raw / 64
#        temperature (int24  little-endian)  degC = raw / 65536
#
# Packet layout while only pressure+temperature are enabled (framesize 10):
#   [1] packet type | [3] timestamp | [3] pressure | [3] temperature  (all little-endian)

import re
import serial
import serial.tools.list_ports
import struct
import sys

# --- User setting: optionally hard-code your Shimmer's MAC here -------------
DEFAULT_MAC = ""  # e.g. "00:06:66:B1:4B:B7" (leave "" to require a CLI argument)

# --- Shimmer BT bytes ------------------------------------------------------
ACK = 0xFF
NACK = 0xFE
GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND = 0xA7
PRESSURE_CALIBRATION_COEFFICIENTS_RESPONSE = 0xA6
SET_SENSORS_COMMAND = 0x08
SET_SAMPLING_RATE_COMMAND = 0x05
START_STREAMING_COMMAND = 0x07
STOP_STREAMING_COMMAND = 0x20

# chEnPressureAndTemperature is bit 2 of the "Sensors 2" byte (3rd sensor byte)
SENSORS0 = 0x00
SENSORS1 = 0x00
SENSORS2 = 0x04  # pressure & temperature

DESIRED_SAMPLING_FREQ_HZ = 10.0

# BMP581 sanity ranges (quick pass/fail hint)
PRESS_MIN_PA = 30000.0
PRESS_MAX_PA = 125000.0
TEMP_MIN_C = -40.0
TEMP_MAX_C = 85.0


# --- Port / MAC helpers ----------------------------------------------------
def normalize_mac(s):
    """Strip separators, upper-case -> 12 hex chars (or whatever hex it holds)."""
    return re.sub(r'[^0-9A-Fa-f]', '', s).upper()


def extract_mac_from_hwid(hwid):
    """Pull the device MAC (a contiguous 12-hex run) out of a Windows BT hwid.
    Strips the {..} service-UUID first so the Bluetooth base UUID fragment
    (00805F9B34FB) on non-connectable local ports isn't mistaken for a MAC."""
    h = re.sub(r'\{[^}]*\}', '', hwid or "").replace(':', '')
    BASE = '00805F9B34FB'  # Bluetooth base UUID tail - not a device MAC
    candidates = [c.upper() for c in re.findall(r'([0-9A-Fa-f]{12})', h)
                  if set(c.upper()) != {'0'} and c.upper() != BASE]
    return candidates[-1] if candidates else None


def serial_ports_bluetooth():
    """Ports whose description mentions Bluetooth (same filter as
    shimmer_device.serial_ports_bluetooth())."""
    return [p for p in serial.tools.list_ports.comports()
            if "Bluetooth" in (p.description or "")]


def list_bt_ports():
    print("Bluetooth serial ports (MAC | port | description):")
    found = False
    for p in serial.tools.list_ports.comports():
        mac = extract_mac_from_hwid(p.hwid)
        if mac:  # only ports carrying a real device MAC are connectable
            found = True
            print("  %-12s  %-6s  %s" % (mac, p.device, p.description))
    if not found:
        print("  (none found - is the device paired?)")


def pick_com_port_interactive():
    """Mirror of shimmer_app_common.get_selected_com_port(dock_ports=False):
    list the Bluetooth serial ports and let the user pick one by number."""
    options = serial_ports_bluetooth()
    if not options:
        print("No Bluetooth serial ports found - is the Shimmer paired?")
        return None
    print("Pick an option:")
    for index, item in enumerate(options):
        mac = extract_mac_from_hwid(item.hwid) or "?"
        print("  %d) %s  (MAC %s)  %s" % (index + 1, item.device, mac, item.description))
    user_input = ""
    while (not user_input.isdigit()) or int(user_input) < 1 or int(user_input) > len(options):
        user_input = input("Your choice: ").strip()
    port = options[int(user_input) - 1].device
    print("You picked: %s\n" % port)
    return port


def find_port_by_mac(mac):
    """Return the COM port whose hwid contains this MAC, preferring the
    outgoing port (hwid ...<MAC>_C00000000)."""
    target = normalize_mac(mac)
    outgoing = None
    fallback = None
    for p in serial.tools.list_ports.comports():
        hwid = (p.hwid or "").upper().replace(':', '').replace('-', '')
        if target and target in hwid:
            if '_C00000000' in hwid:
                outgoing = p.device
            elif fallback is None:
                fallback = p.device
    return outgoing or fallback


def resolve_port(arg):
    """arg may be a COM/dev path or a MAC address. Returns a port string or None."""
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
        if port:
            print("MAC %s -> %s" % (a, port))
        else:
            print("No paired serial port found for MAC %s." % a)
            list_bt_ports()
        return port
    # not COM, not a 12-hex MAC: use verbatim and let serial.Serial complain
    return a


def wait_for_ack():
    ack = struct.pack('B', ACK)
    ddata = bytes()
    while ddata != ack:
        ddata = ser.read(1)
    return


def u24_le(b0, b1, b2):
    """Unsigned 24-bit, little-endian (b0 = XLSB)."""
    return b0 | (b1 << 8) | (b2 << 16)


def s24_le(b0, b1, b2):
    """Signed 24-bit, little-endian (b0 = XLSB)."""
    v = u24_le(b0, b1, b2)
    if v >= 0x800000:
        v -= 0x1000000
    return v


def check_calibration_nack():
    """Send GET_PRESSURE_CALIBRATION_COEFFICIENTS and expect a NACK for a BMP581."""
    ser.write(struct.pack('B', GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND))
    resp = ser.read(1)
    if len(resp) == 0:
        print("WARNING: no reply to calibration command (timeout).")
        return False
    b = resp[0]
    if b == NACK:
        print("calibration command NACK'd (0xFE) - BMP581 is pre-compensated, "
              "no coefficients needed. Good.")
        return True
    if b == ACK:
        hdr = ser.read(1)
        if len(hdr) and hdr[0] == PRESSURE_CALIBRATION_COEFFICIENTS_RESPONSE:
            lenb = ser.read(1)
            if len(lenb) == 0:
                print("WARNING: timed out reading calibration length byte.")
                return False
            length = lenb[0]
            payload = ser.read(length)
            sensor_id = payload[0] if len(payload) else -1
            print("WARNING: device ACK'd and returned coefficients (sensor id "
                  "0x%02x). NACK not active for this unit - discarding "
                  "coefficients and streaming raw anyway." % sensor_id)
        else:
            print("WARNING: unexpected response header after ACK: %r" % hdr)
        return False
    print("WARNING: unexpected reply 0x%02x to calibration command." % b)
    return False


# --- Argument handling -----------------------------------------------------
raw_args = sys.argv[1:]

if any(a in ('--list', '-l', 'list') for a in raw_args):
    list_bt_ports()
    sys.exit(0)

# Join args and drop stray spaces so "COM 38" -> "COM38" and a spaced MAC works too
arg = "".join(raw_args).strip() if raw_args else None

port = resolve_port(arg)
if not port:
    # No argument (or MAC not found): fall back to the interactive numbered
    # picker, same behaviour as test_bt_cmds.py / get_selected_com_port().
    port = pick_com_port_interactive()
if not port:
    print("No device selected. You can also pass a COM port or MAC directly:")
    print("   bmp581.py COM28")
    print("   bmp581.py 00:06:66:B1:4B:B7")
    print("   bmp581.py --list        (show paired BT ports + MACs)")
    sys.exit(1)

ser = serial.Serial(port, 115200, timeout=2)
ser.flushInput()
print("port %s opened." % port)

# 1. calibration handshake - expect NACK for a BMP581
check_calibration_nack()

# 2. enable pressure + temperature
ser.write(struct.pack('BBBB', SET_SENSORS_COMMAND, SENSORS0, SENSORS1, SENSORS2))
wait_for_ack()
print("sensors set (pressure + temperature).")

# 3. set sampling rate: clock_wait = 32768 / freq, little-endian uint16
clock_wait = int(32768 / DESIRED_SAMPLING_FREQ_HZ)
ser.write(struct.pack('<BH', SET_SAMPLING_RATE_COMMAND, clock_wait))
wait_for_ack()
print("sampling rate set (%.2f Hz, clock_wait=%d)." % (DESIRED_SAMPLING_FREQ_HZ, clock_wait))

# 4. start streaming
ser.write(struct.pack('B', START_STREAMING_COMMAND))
wait_for_ack()
print("streaming started. Press Ctrl+C to stop.\n")

ddata = bytes()
numbytes = 0
framesize = 10  # 1 type + 3 timestamp + 3 pressure + 3 temperature

sample_count = 0
ok_count = 0

print("PktType  Timestamp   RawPress  Pressure(Pa)  Pressure(bar)  RawTemp   Temp(degC)  OK")
try:
    while True:
        while numbytes < framesize:
            ddata += ser.read(framesize)
            numbytes = len(ddata)

        data = ddata[0:framesize]
        ddata = ddata[framesize:]
        numbytes = len(ddata)

        (packettype,) = struct.unpack('B', data[0:1])
        (ts0, ts1, ts2) = struct.unpack('BBB', data[1:4])
        timestamp = ts0 | (ts1 << 8) | (ts2 << 16)

        (p0, p1, p2, t0, t1, t2) = struct.unpack('BBBBBB', data[4:framesize])

        raw_press = u24_le(p0, p1, p2)   # unsigned
        raw_temp = s24_le(t0, t1, t2)    # signed

        pressure_pa = raw_press / 64.0
        temperature_c = raw_temp / 65536.0

        in_range = (PRESS_MIN_PA <= pressure_pa <= PRESS_MAX_PA
                    and TEMP_MIN_C <= temperature_c <= TEMP_MAX_C)
        sample_count += 1
        if in_range:
            ok_count += 1

        print("0x%02x   %8d   %8d   %10.2f   %11.5f   %8d   %9.2f   %s"
              % (packettype, timestamp, raw_press, pressure_pa,
                 pressure_pa / 100000.0, raw_temp, temperature_c,
                 "yes" if in_range else "OUT-OF-RANGE"))

except KeyboardInterrupt:
    ser.write(struct.pack('B', STOP_STREAMING_COMMAND))
    wait_for_ack()
    ser.close()
    print("")
    print("stop command sent, port closed.")
    if sample_count:
        print("Received %d samples, %d in valid BMP581 range (%.0f%%)."
              % (sample_count, ok_count, 100.0 * ok_count / sample_count))
        print("RESULT: %s" % ("PASS - BMP581 streaming over BT verified."
                               if ok_count == sample_count and sample_count > 0
                               else "CHECK - some/all samples out of range."))
    else:
        print("RESULT: FAIL - no samples received.")
