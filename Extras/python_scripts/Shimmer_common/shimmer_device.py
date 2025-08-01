import serial.tools.list_ports
from enum import Enum

from Shimmer_common import shimmer_comms_docked, shimmer_comms_bluetooth, util_shimmer


class SrHwVer(Enum):
    SHIMMER3 = 3
    SHIMMER3R = 10

class SrBoardCodes(Enum):
    EXP_BRD_BR_AMP = 8
    EXP_BRD_GSR = 14
    SHIMMER3_IMU = 31
    EXP_BRD_PROTO3_MINI = 36
    EXP_BRD_EXG = 37
    EXP_BRD_PROTO3_DELUXE = 38
    EXP_BRD_ADXL377_ACCEL_200G = 44
    EXP_BRD_EXG_UNIFIED = 47
    EXP_BRD_GSR_UNIFIED = 48
    EXP_BRD_BR_AMP_UNIFIED = 49
    EXP_BRD_H3LIS331DL_ACCEL_HIGH_G = 55
    SHIMMER_GQ_LR = 56
    SHIMMER_GQ_SR = 57
    SHIMMER_ECG_MD = 59


def serial_ports_shimmer_dock():
    serial_port_list = serial_ports()
    serial_port_list_filtered = []
    for item in serial_port_list:
        if "USB Serial Port" in item.description \
                and (("PID=0403:6010" in item.hwid and item.hwid.endswith("B"))
                     or ("PID=0403:6011" in item.hwid and item.hwid.endswith("D"))):
            serial_port_list_filtered += [item]
    return serial_port_list_filtered


def serial_ports_bluetooth():
    serial_port_list = serial_ports()
    serial_port_list_filtered = []
    for item in serial_port_list:
        if "Bluetooth" in item.description:
            serial_port_list_filtered += [item]
    return serial_port_list_filtered


def serial_ports():
    ports = serial.tools.list_ports.comports()
    # for port, desc, hwid in sorted(ports):
    #     print("{}: {} [{}]".format(port, desc, hwid))
    return ports


class Shimmer3:
    mac_id = None

    hw_ver = None
    fw_id = None
    fw_ver_major = None
    fw_ver_minor = None
    fw_ver_internal = None

    daughter_card_id = None
    daughter_card_rev_major = None
    daughter_card_rev_minor = None

    bluetooth_ver_str = None

    batt_adc_value = None
    charging_status = None

    status_toggle_led_red = None
    status_sd_error = None
    status_sd_in_slot = None
    status_is_streaming = None
    status_is_logging = None
    status_is_rwc_time_set = None
    status_is_sensing = None
    status_is_docked = None
    status_is_usb_plugged_in = None

    bt_crc_byte_count = 0

    def __init__(self):
        self.dock_port = shimmer_comms_docked.ShimmerUart(self)
        self.bluetooth_port = shimmer_comms_bluetooth.ShimmerBluetooth(self)

    def setup_dock_com_port(self, com_port, debug_txrx_packets=False):
        return self.dock_port.setup_serial_port(com_port, 115200, debug_txrx_packets)

    def setup_bluetooth_com_port(self, com_port, debug_txrx_packets=False):
        return self.bluetooth_port.setup_serial_port(com_port, 1000000, debug_txrx_packets)

    def parse_hw_fw_ver_bytes(self, byte_buf):
        index = 0
        if len(byte_buf) == 7:
            self.hw_ver = byte_buf[index]
            index += 1
        elif len(byte_buf) == 8:
            self.hw_ver = (byte_buf[index] | (byte_buf[index + 1] << 8))
            index += 2

        self.parse_fw_ver_bytes(byte_buf, index)

    def parse_fw_ver_bytes(self, byte_buf, index=0):
        self.fw_id = ((byte_buf[index]) | (byte_buf[index + 1] << 8))
        index += 2
        self.fw_ver_major = ((byte_buf[index]) | (byte_buf[index + 1] << 8))
        index += 2
        self.fw_ver_minor = byte_buf[index]
        index += 1
        self.fw_ver_internal = byte_buf[index]

    def parse_daughter_card_id(self, byte_buf):
        self.daughter_card_id = byte_buf[0]
        self.daughter_card_rev_major = byte_buf[1]
        self.daughter_card_rev_minor = byte_buf[2]

    def parse_infomem(self, byte_buf):
        # TODO
        return

    def set_bluetooth_ver_str(self, byte_buf):
        self.bluetooth_ver_str = bytearray(byte_buf).decode("ASCII")

    def print_hw_fw_revision(self):
        print("HW Revision: " + str(self.hw_ver))
        print("FW Revision: ID=" + str(self.fw_id)
              + ", v" + str(self.fw_ver_major)
              + "." + str(self.fw_ver_minor)
              + "." + str(self.fw_ver_internal))

    def print_daughter_card_id(self):
        res = "SR" + str(self.daughter_card_id) + "." + str(self.daughter_card_rev_major) + "." + str(
            self.daughter_card_rev_minor)
        if self.daughter_card_id == 255 and self.daughter_card_rev_major == 255 and self.daughter_card_rev_minor == 255:
            res += " (No expansion board detected)"
        print(res)

    def print_batt_status(self):
        print("ADC Value=" + str(self.batt_adc_value) + ", Charging status=" + str(self.charging_status))

    def parse_status(self, status):

        self.status_toggle_led_red = (status[0] >> 7) & 0x01
        self.status_sd_error = (status[0] >> 6) & 0x01
        self.status_sd_in_slot = (status[0] >> 5) & 0x01
        self.status_is_streaming = (status[0] >> 4) & 0x01
        self.status_is_logging = (status[0] >> 3) & 0x01
        self.status_is_rwc_time_set = (status[0] >> 2) & 0x01
        self.status_is_sensing = (status[0] >> 1) & 0x01
        self.status_is_docked = (status[0] >> 0) & 0x01
        print("Status#1 0x%02x, Red LED state: %d, sdError: %d, sdInserted: %d, btStreaming: %d, sdLogging: %d, "
              "isRwcTimeSet: %d, isSensing: %d, docked: %d" % (status[0], self.status_toggle_led_red,
                                                                 self.status_sd_error,
                                                                 self.status_sd_in_slot,
                                                                 self.status_is_streaming,
                                                                 self.status_is_logging,
                                                                 self.status_is_rwc_time_set,
                                                                 self.status_is_sensing,
                                                                 self.status_is_docked))

        if len(status) > 1:
            self.status_is_usb_plugged_in = (status[1] >> 0) & 0x01
            print("Status#2 0x%02x, usbPluggedIn: %d" % (status[1], self.status_is_usb_plugged_in))
        else:
            self.status_is_usb_plugged_in = 0

    def is_expansion_board_set(self):
        return (self.daughter_card_id is not None
                and self.daughter_card_rev_major is not None
                and self.daughter_card_rev_minor is not None)

    def is_hardware_version_set(self):
        return self.hw_ver is not None

    def is_firmware_version_set(self):
        return (self.fw_ver_major is not None
                and self.fw_ver_minor is not None
                and self.fw_ver_internal is not None)

    def is_icm20948_present(self):
        return self.hw_ver == SrHwVer.SHIMMER3.value and ((self.daughter_card_id == SrBoardCodes.SHIMMER3_IMU.value and (self.daughter_card_rev_major == 9
                                                                                   or self.daughter_card_rev_major == 10))
                or (self.daughter_card_id == SrBoardCodes.EXP_BRD_EXG_UNIFIED.value and (self.daughter_card_rev_major == 5
                                                                                   or self.daughter_card_rev_major == 6))
                or (self.daughter_card_id == SrBoardCodes.EXP_BRD_GSR_UNIFIED.value and (self.daughter_card_rev_major == 4
                                                                                   or self.daughter_card_rev_major == 5))
                or (self.daughter_card_id == SrBoardCodes.EXP_BRD_BR_AMP_UNIFIED.value and self.daughter_card_rev_major == 3)
                or (self.daughter_card_id == SrBoardCodes.EXP_BRD_PROTO3_DELUXE.value and (self.daughter_card_rev_major == 3
                                                                                     or self.daughter_card_rev_major == 4))
                or (self.daughter_card_id == SrBoardCodes.EXP_BRD_PROTO3_MINI.value and (self.daughter_card_rev_major == 3
                                                                                   or self.daughter_card_rev_major == 4))
                or (self.daughter_card_id == SrBoardCodes.EXP_BRD_ADXL377_ACCEL_200G.value and (
                        self.daughter_card_rev_major == 2 or self.daughter_card_rev_major == 3)))

    def is_bmp180_present(self):
        return self.hw_ver == SrHwVer.SHIMMER3.value and ((self.daughter_card_id == SrBoardCodes.SHIMMER3_IMU.value and self.daughter_card_rev_major <= 5)
                or self.daughter_card_id == SrBoardCodes.EXP_BRD_EXG.value
                or (self.daughter_card_id == SrBoardCodes.EXP_BRD_EXG_UNIFIED.value and self.daughter_card_rev_major <= 2)
                or self.daughter_card_id == SrBoardCodes.EXP_BRD_GSR.value
                or (self.daughter_card_id == SrBoardCodes.EXP_BRD_GSR_UNIFIED.value and self.daughter_card_rev_major <= 2)
                or self.daughter_card_id == SrBoardCodes.EXP_BRD_BR_AMP.value
                or (self.daughter_card_id == SrBoardCodes.EXP_BRD_BR_AMP_UNIFIED.value and self.daughter_card_rev_major == 1)
                or (self.daughter_card_id == SrBoardCodes.EXP_BRD_PROTO3_DELUXE.value and self.daughter_card_rev_major <= 2)
                or (self.daughter_card_id == SrBoardCodes.EXP_BRD_PROTO3_MINI.value and self.daughter_card_rev_major <= 2)
                or (self.daughter_card_id == SrBoardCodes.EXP_BRD_ADXL377_ACCEL_200G.value and self.daughter_card_rev_major == 1))

    def is_bmp280_present(self):
        return self.hw_ver == SrHwVer.SHIMMER3.value and ((not self.is_bmp180_present()) and
                (self.daughter_card_rev_major == 171
                 or (self.daughter_card_id == SrBoardCodes.SHIMMER3_IMU.value and self.daughter_card_rev_major <= 10)
                 or (self.daughter_card_id == SrBoardCodes.EXP_BRD_EXG_UNIFIED.value and self.daughter_card_rev_major <= 6)
                 or (self.daughter_card_id == SrBoardCodes.EXP_BRD_GSR_UNIFIED.value and self.daughter_card_rev_major <= 5)
                 or (self.daughter_card_id == SrBoardCodes.EXP_BRD_BR_AMP_UNIFIED.value and self.daughter_card_rev_major <= 3)
                 or (self.daughter_card_id == SrBoardCodes.EXP_BRD_PROTO3_DELUXE.value and self.daughter_card_rev_major <= 4)
                 or (self.daughter_card_id == SrBoardCodes.EXP_BRD_PROTO3_MINI.value and self.daughter_card_rev_major <= 4)
                 or (self.daughter_card_id == SrBoardCodes.EXP_BRD_ADXL377_ACCEL_200G.value and self.daughter_card_rev_major <= 3)))

    def is_hardware_shimmer3(self):
        return self.hw_ver == SrHwVer.SHIMMER3.value

    def is_hardware_shimmer3r(self):
        return self.hw_ver == SrHwVer.SHIMMER3R.value

    def is_bt_cmd_common_pressure_calibration_supported(self):
        return ((self.hw_ver == SrHwVer.SHIMMER3.value and self.compare_versions(0, 16, 6))
                or (self.hw_ver == SrHwVer.SHIMMER3R.value and self.compare_versions(0, 1, 0)))

    def compare_versions(self, comp_major, comp_minor, comp_internal):
        return util_shimmer.compare_versions(self.fw_ver_major, self.fw_ver_minor, self.fw_ver_internal, comp_major,
                                             comp_minor, comp_internal)
