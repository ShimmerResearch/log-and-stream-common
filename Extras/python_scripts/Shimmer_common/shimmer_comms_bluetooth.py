import binascii
from turtle import delay

import serial
import time
import struct

import serial.win32
from serial import SerialException, Serial
from Shimmer_common import shimmer_crc, util_shimmer


class BtCmds:
    DATA_PACKET = 0x00
    INQUIRY_COMMAND = 0x01
    INQUIRY_RESPONSE = 0x02
    GET_SAMPLING_RATE_COMMAND = 0x03
    SAMPLING_RATE_RESPONSE = 0x04
    SET_SAMPLING_RATE_COMMAND = 0x05
    TOGGLE_LED_COMMAND = 0x06
    START_STREAMING_COMMAND = 0x07  # maintain compatibility with Shimmer2/2r BtStream
    SET_SENSORS_COMMAND = 0x08
    SET_ACCEL_RANGE_COMMAND = 0x09
    ACCEL_RANGE_RESPONSE = 0x0A
    GET_ACCEL_RANGE_COMMAND = 0x0B
    SET_CONFIG_SETUP_BYTES_COMMAND = 0x0E
    CONFIG_SETUP_BYTES_RESPONSE = 0x0F
    GET_CONFIG_SETUP_BYTES_COMMAND = 0x10
    SET_A_ACCEL_CALIBRATION_COMMAND = 0x11
    A_ACCEL_CALIBRATION_RESPONSE = 0x12
    GET_A_ACCEL_CALIBRATION_COMMAND = 0x13
    SET_GYRO_CALIBRATION_COMMAND = 0x14
    GYRO_CALIBRATION_RESPONSE = 0x15
    GET_GYRO_CALIBRATION_COMMAND = 0x16
    SET_MAG_CALIBRATION_COMMAND = 0x17
    MAG_CALIBRATION_RESPONSE = 0x18
    GET_MAG_CALIBRATION_COMMAND = 0x19
    SET_ACCEL_CALIBRATION_COMMAND = 0x1A
    ACCEL_CALIBRATION_RESPONSE = 0x1B
    GET_ACCEL_CALIBRATION_COMMAND = 0x1C
    STOP_STREAMING_COMMAND = 0x20  # maintain compatibility with Shimmer2/2r BtStream
    SET_GSR_RANGE_COMMAND = 0x21
    GSR_RANGE_RESPONSE = 0x22
    GET_GSR_RANGE_COMMAND = 0x23
    # DEPRECATED_GET_DEVICE_VERSION_COMMAND         = 0x24
    DEVICE_VERSION_RESPONSE = 0x25  # maintain compatibility with Shimmer2/2r BtStream
    SET_EMG_CALIBRATION_COMMAND = 0x26
    EMG_CALIBRATION_RESPONSE = 0x27
    GET_EMG_CALIBRATION_COMMAND = 0x28
    SET_ECG_CALIBRATION_COMMAND = 0x29
    ECG_CALIBRATION_RESPONSE = 0x2A
    GET_ECG_CALIBRATION_COMMAND = 0x2B
    GET_ALL_CALIBRATION_COMMAND = 0x2C
    ALL_CALIBRATION_RESPONSE = 0x2D
    GET_FW_VERSION_COMMAND = 0x2E  # maintain compatibility with Shimmer2/2r BtStream
    FW_VERSION_RESPONSE = 0x2F  # maintain compatibility with Shimmer2/2r BtStream
    SET_CHARGE_STATUS_LED_COMMAND = 0x30
    CHARGE_STATUS_LED_RESPONSE = 0x31
    GET_CHARGE_STATUS_LED_COMMAND = 0x32
    BUFFER_SIZE_RESPONSE = 0x35
    GET_BUFFER_SIZE_COMMAND = 0x36
    SET_MAG_GAIN_COMMAND = 0x37
    MAG_GAIN_RESPONSE = 0x38
    GET_MAG_GAIN_COMMAND = 0x39
    SET_MAG_SAMPLING_RATE_COMMAND = 0x3A
    MAG_SAMPLING_RATE_RESPONSE = 0x3B
    GET_MAG_SAMPLING_RATE_COMMAND = 0x3C
    UNIQUE_SERIAL_RESPONSE = 0x3D
    GET_UNIQUE_SERIAL_COMMAND = 0x3E
    GET_DEVICE_VERSION_COMMAND = 0x3F
    SET_ACCEL_SAMPLING_RATE_COMMAND = 0x40
    ACCEL_SAMPLING_RATE_RESPONSE = 0x41
    GET_ACCEL_SAMPLING_RATE_COMMAND = 0x42
    SET_ACCEL_LPMODE_COMMAND = 0x43
    ACCEL_LPMODE_RESPONSE = 0x44
    GET_ACCEL_LPMODE_COMMAND = 0x45
    SET_ACCEL_HRMODE_COMMAND = 0x46
    ACCEL_HRMODE_RESPONSE = 0x47
    GET_ACCEL_HRMODE_COMMAND = 0x48
    SET_GYRO_RANGE_COMMAND = 0x49
    GYRO_RANGE_RESPONSE = 0x4A
    GET_GYRO_RANGE_COMMAND = 0x4B
    SET_GYRO_SAMPLING_RATE_COMMAND = 0x4C
    GYRO_SAMPLING_RATE_RESPONSE = 0x4D
    GET_GYRO_SAMPLING_RATE_COMMAND = 0x4E
    SET_ALT_ACCEL_RANGE_COMMAND = 0x4F
    ALT_ACCEL_RANGE_RESPONSE = 0x50
    GET_ALT_ACCEL_RANGE_COMMAND = 0x51
    SET_PRES_OVERSAMPLING_RATIO_COMMAND = 0x52
    PRES_OVERSAMPLING_RATIO_RESPONSE = 0x53
    GET_PRES_OVERSAMPLING_RATIO_COMMAND = 0x54
    BMP180_CALIBRATION_COEFFICIENTS_RESPONSE = 0x58
    GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND = 0x59
    RESET_TO_DEFAULT_CONFIGURATION_COMMAND = 0x5A
    RESET_CALIBRATION_VALUE_COMMAND = 0x5B
    ALT_MAG_SENS_ADJ_VALS_RESPONSE = 0x5C
    GET_ALT_MAG_SENS_ADJ_VALS_COMMAND = 0x5D
    SET_INTERNAL_EXP_POWER_ENABLE_COMMAND = 0x5E
    INTERNAL_EXP_POWER_ENABLE_RESPONSE = 0x5F
    GET_INTERNAL_EXP_POWER_ENABLE_COMMAND = 0x60
    SET_EXG_REGS_COMMAND = 0x61
    EXG_REGS_RESPONSE = 0x62
    GET_EXG_REGS_COMMAND = 0x63
    SET_DAUGHTER_CARD_ID_COMMAND = 0x64
    DAUGHTER_CARD_ID_RESPONSE = 0x65
    GET_DAUGHTER_CARD_ID_COMMAND = 0x66
    SET_DAUGHTER_CARD_MEM_COMMAND = 0x67
    DAUGHTER_CARD_MEM_RESPONSE = 0x68
    GET_DAUGHTER_CARD_MEM_COMMAND = 0x69
    SET_BT_COMMS_BAUD_RATE = 0x6A
    BT_COMMS_BAUD_RATE_RESPONSE = 0x6B
    GET_BT_COMMS_BAUD_RATE = 0x6C
    SET_DERIVED_CHANNEL_BYTES = 0x6D
    DERIVED_CHANNEL_BYTES_RESPONSE = 0x6E
    GET_DERIVED_CHANNEL_BYTES = 0x6F
    START_SDBT_COMMAND = 0x70
    STATUS_RESPONSE = 0x71
    GET_STATUS_COMMAND = 0x72
    SET_TRIAL_CONFIG_COMMAND = 0x73
    TRIAL_CONFIG_RESPONSE = 0x74
    GET_TRIAL_CONFIG_COMMAND = 0x75
    SET_CENTER_COMMAND = 0x76
    CENTER_RESPONSE = 0x77
    GET_CENTER_COMMAND = 0x78
    SET_SHIMMERNAME_COMMAND = 0x79
    SHIMMERNAME_RESPONSE = 0x7a
    GET_SHIMMERNAME_COMMAND = 0x7b
    SET_EXPID_COMMAND = 0x7c
    EXPID_RESPONSE = 0x7d
    GET_EXPID_COMMAND = 0x7e
    SET_MYID_COMMAND = 0x7F
    MYID_RESPONSE = 0x80
    GET_MYID_COMMAND = 0x81
    SET_NSHIMMER_COMMAND = 0x82
    NSHIMMER_RESPONSE = 0x83
    GET_NSHIMMER_COMMAND = 0x84
    SET_CONFIGTIME_COMMAND = 0x85
    CONFIGTIME_RESPONSE = 0x86
    GET_CONFIGTIME_COMMAND = 0x87
    DIR_RESPONSE = 0x88
    GET_DIR_COMMAND = 0x89
    INSTREAM_CMD_RESPONSE = 0x8A
    SET_CRC_COMMAND = 0x8B
    SET_INFOMEM_COMMAND = 0x8C
    INFOMEM_RESPONSE = 0x8D
    GET_INFOMEM_COMMAND = 0x8E
    SET_RWC_COMMAND = 0x8F
    RWC_RESPONSE = 0x90
    GET_RWC_COMMAND = 0x91
    START_LOGGING_COMMAND = 0x92
    STOP_LOGGING_COMMAND = 0x93
    VBATT_RESPONSE = 0x94
    GET_VBATT_COMMAND = 0x95
    TEST_CONNECTION_COMMAND = 0x96
    STOP_SDBT_COMMAND = 0x97
    SET_CALIB_DUMP_COMMAND = 0x98
    RSP_CALIB_DUMP_COMMAND = 0x99
    GET_CALIB_DUMP_COMMAND = 0x9A
    UPD_CALIB_DUMP_COMMAND = 0x9B
    UPD_SDLOG_CFG_COMMAND = 0x9C
    BMP280_CALIBRATION_COEFFICIENTS_RESPONSE = 0x9F
    GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND = 0xA0
    GET_BT_VERSION_STR_COMMAND = 0xA1
    BT_VERSION_STR_RESPONSE = 0xA2
    SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE = 0xA3
    SET_DATA_RATE_TEST_MODE = 0xA4
    DATA_RATE_TEST_RESPONSE = 0xA5
    PRESSURE_CALIBRATION_COEFFICIENTS_RESPONSE = 0xA6
    GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND = 0xA7
    SET_SD_SYNC_COMMAND = 0xE0
    SD_SYNC_RESPONSE = 0xE1
    ACK_COMMAND_PROCESSED = 0xFF
    SET_FACTORY_TEST = 0xA8
    SET_ALT_ACCEL_CALIBRATION_COMMAND = 0xA9
    ALT_ACCEL_CALIBRATION_RESPONSE = 0xAA
    GET_ALT_ACCEL_CALIBRATION_COMMAND = 0xAB
    SET_ALT_ACCEL_SAMPLING_RATE_COMMAND = 0XAC
    ALT_ACCEL_SAMPLING_RATE_RESPONSE = 0xAD
    GET_ALT_ACCEL_SAMPLING_RATE_COMMAND = 0xAE
    SET_ALT_MAG_CALIBRATION_COMMAND = 0XAF
    ALT_MAG_CALIBRATION_RESPONSE = 0XB0
    GET_ALT_MAG_CALIBRATION_COMMAND = 0xB1
    SET_ALT_MAG_SAMPLING_RATE_COMMAND = 0xB2
    ALT_MAG_SAMPLING_RATE_RESPONSE = 0XB3
    GET_ALT_MAG_SAMPLING_RATE_COMMAND = 0xB4
    DUMMY_COMMAND = 0xB5
    RESET_BT_ERROR_COUNTS = 0xB6
    SET_FEATURE = 0xB7
    # SD-card file transfer (LogAndStream_Shimmer3R >= v1.01.008).
    # Command opcodes avoid the EZ-Serial SOF bytes 0x80/0xC0/0xD0 (CYW20820
    # UART RX demux would swallow them), hence LIST sits at 0xCC.
    SD_LIST_DIR_COMMAND = 0xCC
    SD_LIST_DIR_RESPONSE = 0xC1
    SD_FILE_STAT_COMMAND = 0xC2
    SD_FILE_STAT_RESPONSE = 0xC3
    SD_FILE_READ_COMMAND = 0xC4
    SD_FILE_DATA_RESPONSE = 0xC5
    SD_FILE_STATUS_RESPONSE = 0xC6
    SD_TRANSFER_ABORT_COMMAND = 0xC7
    SD_FREE_SPACE_COMMAND = 0xC8
    SD_FREE_SPACE_RESPONSE = 0xC9
    SD_DELETE_COMMAND = 0xCA
    SD_DELETE_RESPONSE = 0xCB
    INSTREAM_CMD_RESPONSE = 0x8A


class ShimmerBluetooth:
    serial_port_timeout_ms = 500
    debug_tx_rx_packets = False

    ser = None
    shimmer_device = None

    def __init__(self, shimmer):
        self.shimmer_device = shimmer

    def setup_serial_port(self, com_port, baud_rate, debug_txrx_packets=False):
        ShimmerBluetooth.debug_tx_rx_packets = debug_txrx_packets

        try:
            self.ser = serial.Serial(com_port, baud_rate, timeout=self.serial_port_timeout_ms / 1000)
            if self.ser.is_open:
                self.ser.flushInput()
                print("port opening, done.")
                return True
            else:
                print("can't open port.")
                return False
        except SerialException:
            print("Serial port exception.")
            return False

    def clear_serial_buffer(self):
        self.ser.flushInput()
        self.ser.flushOutput()

    def close_port(self):
        self.ser.close()

    def write_calibration(self, calib_bytes=None, timeout_ms=2000):

        if calib_bytes is None:
            calib_bytes = []
        len_calib_bytes = len(calib_bytes)

        # calib_bytes = [(len_calib_bytes & 0xFF), ((len_calib_bytes >> 8) & 0xFF)] + calib_bytes
        for i in range(0, len_calib_bytes, 128):
            bytes_remaining = len_calib_bytes - i
            buf_len = 128 if bytes_remaining > 128 else bytes_remaining

            if not self.send_bluetooth(
                    [BtCmds.SET_CALIB_DUMP_COMMAND, buf_len, i & 0xFF, (i >> 8) & 0xFF] + calib_bytes[i:i + buf_len]):
                return False

            if not self.wait_for_ack(timeout_ms):
                return False

        return True

    def read_calibration(self, timeout_ms=500):

        calib_dump_concat = []
        overall_mem_length = 0
        length_read_so_far = 0
        first_read = True
        while length_read_so_far == 0 or length_read_so_far < overall_mem_length:
            length_left_to_read = 128 if first_read else overall_mem_length - length_read_so_far + 2
            length_to_read = 128 if (length_left_to_read >= 128) else length_left_to_read
            address = length_read_so_far

            if not self.send_bluetooth([BtCmds.GET_CALIB_DUMP_COMMAND, length_to_read, address & 0xFF,
                                        (address >> 8) & 0xFF]):
                return False

            if not self.wait_for_ack(timeout_ms):
                return False

            rsp_byte = self.wait_for_response(1)
            if rsp_byte[0] != BtCmds.RSP_CALIB_DUMP_COMMAND:
                return False

            # +3 for 1 length byte followed byte 2 bytes address
            rx_bytes = self.wait_for_response(length_to_read + 3, timeout_ms)
            if len(rx_bytes) == 0:
                return False

            calib_dump_concat += rx_bytes[3:len(rx_bytes)]

            if first_read:
                overall_mem_length = (calib_dump_concat[1] << 8) | calib_dump_concat[0]
                first_read = False

            length_read_so_far += length_to_read

        return calib_dump_concat

    def write_configuration(self, tx_bytes=None, timeout_ms=500):

        if tx_bytes is None:
            tx_bytes = []
        len_tx_bytes = len(tx_bytes)

        for i in range(0, len_tx_bytes, 128):
            bytes_remaining = len_tx_bytes - i
            buf_len = 128 if bytes_remaining > 128 else bytes_remaining

            if not self.send_bluetooth(
                    [BtCmds.SET_INFOMEM_COMMAND,
                     buf_len,
                     i & 0xFF, (i >> 8) & 0xFF] + tx_bytes[i:i + buf_len]):
                return False

            if not self.wait_for_ack(timeout_ms):
                return False

        return True

    def read_configuration(self, timeout_ms=500):

        len_config_bytes = 384
        config_bytes = []
        for i in range(0, len_config_bytes, 128):
            bytes_remaining = len_config_bytes - i
            buf_len = 128 if bytes_remaining > 128 else bytes_remaining

            if not self.send_bluetooth([BtCmds.GET_INFOMEM_COMMAND, buf_len, i & 0xFF, (i >> 8) & 0xFF]):
                return False

            if not self.wait_for_ack(timeout_ms):
                return False

            rx_bytes = self.wait_for_response(buf_len + 2, timeout_ms)
            if len(rx_bytes) == 0:
                return False

            config_bytes += rx_bytes[2:len(rx_bytes)]

        return config_bytes

    def wait_for_ack(self, timeout_ms=500):
        response = self.wait_for_response(1, timeout_ms)
        return True if len(response) == 1 and response[0] is BtCmds.ACK_COMMAND_PROCESSED else False

    def get_qty_waiting_in_port(self):
        return self.ser.inWaiting()

    def wait_for_response(self, expected_len, timeout_ms=500, console_print_timeout_msg=True):
        flag = True

        loop_count = 0
        wait_interval_ms = 100
        loop_count_total = timeout_ms / wait_interval_ms

        rx_buf = []

        while flag:
            time.sleep(wait_interval_ms / 1000)
            loop_count += 1
            if loop_count >= loop_count_total:
                if console_print_timeout_msg:
                    print("Timeout while waiting for response")
                break

            buf_len = self.ser.inWaiting()
            if buf_len >= expected_len:
                data_read = self.ser.read(expected_len)
                if isinstance(data_read, str):
                    rx_buf += bytearray.fromhex(binascii.hexlify(data_read))
                else:
                    rx_buf += data_read

                if self.debug_tx_rx_packets:
                    print("UART RX: " + util_shimmer.byte_array_to_hex_string(rx_buf))

                break

        return rx_buf

    def send_bluetooth(self, tx_buf):
        if self.debug_tx_rx_packets:
            print("UART TX: " + util_shimmer.byte_array_to_hex_string(tx_buf))
        self.ser.write(tx_buf)
        time.sleep(0.1)
        return True

    # ------------------------------------------------------------------
    # SD-card file transfer (LogAndStream_Shimmer3R >= v1.01.008)
    # ------------------------------------------------------------------

    @staticmethod
    def _sd_path_bytes(path):
        path_bytes = path.encode("ascii")
        if not 1 <= len(path_bytes) <= 96:
            raise ValueError("path must be 1..96 ASCII bytes")
        return list(path_bytes)

    def sd_list_dir_page(self, path, start_idx=0, max_entries=16, timeout_ms=2000):
        """Read one page of a directory listing.

        Returns (status, entries, has_more); status is None on comms failure,
        0 on success, otherwise the in-band status byte (FatFs FRESULT or
        0xF0=SD unavailable / 0xF1=busy / 0xF2=bad args)."""
        p = self._sd_path_bytes(path)
        if not self.send_bluetooth([BtCmds.SD_LIST_DIR_COMMAND, start_idx & 0xFF,
                                    (start_idx >> 8) & 0xFF, max_entries, len(p)] + p):
            return None, [], False
        if not self.wait_for_ack(timeout_ms):
            return None, [], False
        hdr = self.wait_for_response(8, timeout_ms)
        if len(hdr) < 8 or hdr[0] != BtCmds.SD_LIST_DIR_RESPONSE:
            return None, [], False
        status = hdr[1]
        entries_len = hdr[4] | (hdr[5] << 8)
        n_entries = hdr[6]
        has_more = (hdr[7] & 0x01) != 0
        body = self.wait_for_response(entries_len, timeout_ms) if entries_len else b""
        if len(body) < entries_len:
            return None, [], False
        entries = []
        off = 0
        for _ in range(n_entries):
            attr = body[off]
            size = body[off + 1] | body[off + 2] << 8 | body[off + 3] << 16 | body[off + 4] << 24
            fdate = body[off + 5] | (body[off + 6] << 8)
            ftime = body[off + 7] | (body[off + 8] << 8)
            name_len = body[off + 9]
            name = bytes(body[off + 10:off + 10 + name_len]).decode("ascii", "replace")
            entries.append({"name": name, "is_dir": bool(attr & 0x01),
                            "name_truncated": bool(attr & 0x02), "size": size,
                            "fdate": fdate, "ftime": ftime})
            off += 10 + name_len
        return status, entries, has_more

    def sd_list_dir(self, path, timeout_ms=2000):
        """Full directory listing, following startIdx paging. Returns (status, entries)."""
        all_entries = []
        start_idx = 0
        while True:
            status, entries, has_more = self.sd_list_dir_page(path, start_idx, 16, timeout_ms)
            if status is None or status != 0:
                return status, all_entries
            all_entries += entries
            if not has_more or len(entries) == 0:
                return 0, all_entries
            start_idx += len(entries)

    def sd_stat(self, path, timeout_ms=2000):
        """Stat one file/directory. Returns (status, {size, fdate, ftime, is_dir})."""
        p = self._sd_path_bytes(path)
        if not self.send_bluetooth([BtCmds.SD_FILE_STAT_COMMAND, len(p)] + p):
            return None, {}
        if not self.wait_for_ack(timeout_ms):
            return None, {}
        rsp = self.wait_for_response(11, timeout_ms)
        if len(rsp) < 11 or rsp[0] != BtCmds.SD_FILE_STAT_RESPONSE:
            return None, {}
        return rsp[1], {"size": rsp[2] | rsp[3] << 8 | rsp[4] << 16 | rsp[5] << 24,
                        "fdate": rsp[6] | (rsp[7] << 8), "ftime": rsp[8] | (rsp[9] << 8),
                        "is_dir": bool(rsp[10] & 0x01)}

    def sd_free_space(self, timeout_ms=20000):
        """Returns (status, free_kb, total_kb). First call on a large FAT32
        card can take several seconds (full FAT scan)."""
        if not self.send_bluetooth([BtCmds.SD_FREE_SPACE_COMMAND]):
            return None, 0, 0
        if not self.wait_for_ack(timeout_ms):
            return None, 0, 0
        rsp = self.wait_for_response(10, timeout_ms)
        if len(rsp) < 10 or rsp[0] != BtCmds.SD_FREE_SPACE_RESPONSE:
            return None, 0, 0
        free_kb = rsp[2] | rsp[3] << 8 | rsp[4] << 16 | rsp[5] << 24
        total_kb = rsp[6] | rsp[7] << 8 | rsp[8] << 16 | rsp[9] << 24
        return rsp[1], free_kb, total_kb

    def sd_delete(self, path, timeout_ms=2000):
        """Delete one file or empty directory (firmware only allows paths
        strictly under data/). Returns the status byte or None."""
        p = self._sd_path_bytes(path)
        if not self.send_bluetooth([BtCmds.SD_DELETE_COMMAND, len(p)] + p):
            return None
        if not self.wait_for_ack(timeout_ms):
            return None
        rsp = self.wait_for_response(2, timeout_ms)
        if len(rsp) < 2 or rsp[0] != BtCmds.SD_DELETE_RESPONSE:
            return None
        return rsp[1]

    def sd_abort_transfer(self, timeout_ms=2000):
        if not self.send_bluetooth([BtCmds.SD_TRANSFER_ABORT_COMMAND]):
            return False
        return self.wait_for_ack(timeout_ms)

    def sd_read_window(self, path, offset, window_len, block_len=512, on_block=None,
                       timeout_ms=5000):
        """Request one read window and collect its streamed frames.

        Returns (xfer_status, next_offset, data). xfer_status is None on a
        comms/CRC/sequence failure (resume by re-requesting from
        offset + len(data)), else the SD_FILE_STATUS_RESPONSE code
        (0=window complete, 1=EOF, ...)."""
        p = self._sd_path_bytes(path)
        cmd = ([BtCmds.SD_FILE_READ_COMMAND,
                offset & 0xFF, (offset >> 8) & 0xFF, (offset >> 16) & 0xFF, (offset >> 24) & 0xFF,
                window_len & 0xFF, (window_len >> 8) & 0xFF, (window_len >> 16) & 0xFF,
                (window_len >> 24) & 0xFF,
                block_len & 0xFF, (block_len >> 8) & 0xFF, len(p)] + p)
        data = bytearray()
        if not self.send_bluetooth(cmd):
            return None, offset, bytes(data)
        if not self.wait_for_ack(timeout_ms):
            return None, offset, bytes(data)

        expected_seq = 0
        while True:
            hdr2 = self.wait_for_response(2, timeout_ms)
            if len(hdr2) < 2 or hdr2[0] != BtCmds.INSTREAM_CMD_RESPONSE:
                return None, offset + len(data), bytes(data)
            if hdr2[1] == BtCmds.SD_FILE_DATA_RESPONSE:
                hdr = self.wait_for_response(5, timeout_ms)  # session, seq u16, len u16
                if len(hdr) < 5:
                    return None, offset + len(data), bytes(data)
                seq = hdr[1] | (hdr[2] << 8)
                length = hdr[3] | (hdr[4] << 8)
                payload_crc = self.wait_for_response(length + 2, timeout_ms)
                if len(payload_crc) < length + 2:
                    return None, offset + len(data), bytes(data)
                frame = list(hdr2) + list(hdr) + list(payload_crc)
                if not shimmer_crc.crc_check(len(frame), frame):
                    print("SD data frame CRC failure at seq %d" % seq)
                    return None, offset + len(data), bytes(data)
                if seq != expected_seq:
                    print("SD block sequence gap (expected %d, got %d)" % (expected_seq, seq))
                    return None, offset + len(data), bytes(data)
                expected_seq += 1
                payload = bytes(payload_crc[:length])
                data += payload
                if on_block:
                    on_block(payload)
            elif hdr2[1] == BtCmds.SD_FILE_STATUS_RESPONSE:
                rest = self.wait_for_response(8, timeout_ms)  # session, status, nextOffset u32, crc
                if len(rest) < 8:
                    return None, offset + len(data), bytes(data)
                frame = list(hdr2) + list(rest)
                if not shimmer_crc.crc_check(len(frame), frame):
                    return None, offset + len(data), bytes(data)
                next_offset = rest[2] | rest[3] << 8 | rest[4] << 16 | rest[5] << 24
                return rest[1], next_offset, bytes(data)
            else:
                return None, offset + len(data), bytes(data)

    def sd_download_file(self, path, size=None, block_len=512, window_len=131072,
                         on_progress=None):
        """Download a whole file via windowed reads. Returns bytes or None."""
        if size is None:
            status, stat = self.sd_stat(path)
            if status != 0:
                print("sd_stat('%s') failed, status=%s" % (path, status))
                return None
            size = stat["size"]
        out = bytearray()
        offset = 0
        while offset < size:
            window = min(window_len, size - offset)
            status, next_offset, chunk = self.sd_read_window(path, offset, window, block_len)
            if status is None or status not in (0, 1):
                print("sd_read_window('%s', %d) failed, status=%s" % (path, offset, status))
                return None
            out += chunk
            if next_offset <= offset:
                return None
            offset = next_offset
            if on_progress:
                on_progress(offset, size)
            if status == 1:  # EOF
                break
        return bytes(out)
