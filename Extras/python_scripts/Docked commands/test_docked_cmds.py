#!/usr/bin/python
import unittest

from Shimmer_common import util_shimmer, util_shimmer_time, shimmer_device, shimmer_app_common, shimmer_comms_docked

from colorama import Fore

from Shimmer_common.shimmer_comms_docked import UartPacketCmd, UartComponent, FACTORY_TEST, UartProperty


class TestShimmerDockCommunication(unittest.TestCase):
    shimmer = None
    eeprom_bytes_backup = None
    infomem_bytes_backup = None
    calib_bytes_backup = None

    @classmethod
    def setUpClass(cls):
        com_port = shimmer_app_common.get_selected_com_port()
        if not com_port:
            print("Supported COM port not found, exiting")
            exit()

        cls.shimmer = shimmer_device.Shimmer3()

        if not cls.shimmer.setup_dock_com_port(com_port, debug_txrx_packets=True):
            exit()

    @classmethod
    def tearDownClass(cls):
        # cls._connection.destroy()
        print("All done")

    def test_01_read_mac_id(self):
        print("Read MAC ID:")
        if not self.shimmer.dock_port.read_mac_id():
            print("Error, exiting")
            self.assertTrue(False)
        print(self.shimmer.mac_id)
        print("")

    def test_02_read_hw_fw_ver(self):
        print("Read HW/FW version:")
        if not self.shimmer.dock_port.read_hw_fw_ver():
            print("Error, exiting")
            self.assertTrue(False)
        self.shimmer.print_hw_fw_revision()
        print("")

    def test_03_read_bluetooth_ver(self):
        print("Read Bluetooth version:")
        if not self.shimmer.dock_port.read_bluetooth_ver():
            # Not all firmware versions support this command
            print("Error")
            self.assertTrue(False)
        print(self.shimmer.bluetooth_ver_str)
        print("")

    def test_04_read_battery(self):
        print("Read battery:")
        if not self.shimmer.dock_port.read_batt():
            print("Error, exiting")
            self.assertTrue(False)
        self.shimmer.print_batt_status()
        print("")

    def test_05_write_real_world_clock(self):
        print("Write time from PC:")
        if not self.shimmer.dock_port.write_real_world_clock_from_pc_time():
            print("Error, exiting")
            self.assertTrue(False)
        print("Success")
        print("")

    def test_06_read_real_world_clock_config_time(self):
        print("Read real-time-clock config time:")
        ts_ms = self.shimmer.dock_port.read_real_world_clock_config_time()
        if isinstance(ts_ms, bool):
            print("Error, exiting")
            self.assertTrue(False)
        print(util_shimmer_time.seconds_to_time_str(ts_ms / 1000, True))
        print("")

    def test_07_read_current_time(self):
        print("Read current time:")
        ts_ms = self.shimmer.dock_port.read_current_time()
        if isinstance(ts_ms, bool):
            print("Error, exiting")
            self.assertTrue(False)
        print(util_shimmer_time.seconds_to_time_str(ts_ms / 1000, True))
        print("")

    def test_08_read_daughter_card_id(self):
        print("Read Daughter Card ID:")
        if self.shimmer.dock_port.read_daughter_card_id():
            self.shimmer.print_daughter_card_id()
        else:
            print("Error")
            self.assertTrue(False)
        print("")

    def test_09_write_daughter_card_id(self):
        print("Write Daughter Card ID:")
        if self.shimmer.daughter_card_id is None \
                or self.shimmer.daughter_card_rev_major is None \
                or self.shimmer.daughter_card_rev_minor is None:
            self.assertTrue(False)

        if self.shimmer.dock_port.write_daughter_card_id(self.shimmer.daughter_card_id,
                                                         self.shimmer.daughter_card_rev_major,
                                                         self.shimmer.daughter_card_rev_minor) \
                and self.shimmer.dock_port.read_daughter_card_id():
            self.shimmer.print_daughter_card_id()
        else:
            print("No expansion board detected")
        print("")

    def test_10_read_write_daughter_card_memory(self):
        print("Read Daughter Card Mem:")
        eeprom_bytes = self.shimmer.dock_port.read_daughter_card_mem()
        if isinstance(eeprom_bytes, bool):
            print("Error")
            self.assertTrue(False)
        else:
            print(util_shimmer.byte_array_to_hex_string(eeprom_bytes))
            self.eeprom_bytes_backup = eeprom_bytes
        print("")

        if self.eeprom_bytes_backup is not None:
            print("Write Daughter Card Mem:")
            # test_buf = [0xFF] * 128
            test_buf = [i for i in range(0, 128)]
            result = self.shimmer.dock_port.write_daughter_card_mem(test_buf)
            if result:
                eeprom_bytes = self.shimmer.dock_port.read_daughter_card_mem()
                print("Success" if test_buf == eeprom_bytes else "FAIL")
                print("Restoring Daughter Card Mem contents")
                result = self.shimmer.dock_port.write_daughter_card_mem(self.eeprom_bytes_backup)
                self.assertTrue(result)
            else:
                self.assertTrue(False)
                print("Error")
            print("")

    def test_11_read_write_infomem(self):
        print("Read Infomem:")
        infomem_bytes = self.shimmer.dock_port.read_infomem()
        if isinstance(infomem_bytes, bool):
            print("Error")
            self.assertTrue(False)
        else:
            print(util_shimmer.byte_array_to_hex_string(infomem_bytes))
            self.infomem_bytes_backup = infomem_bytes
        print("")

        if self.infomem_bytes_backup is not None:
            print("Write Infomem:")
            test_buf = [(i & 0xFF) for i in range(0, 384)]
            print("- Writing test buffer")
            result = self.shimmer.dock_port.write_infomem(test_buf)
            if result:
                print("- Reading back")
                infomem_bytes = self.shimmer.dock_port.read_infomem()
                if isinstance(infomem_bytes, bool):
                    print("Error")
                    self.assertTrue(False)
                else:
                    print(util_shimmer.byte_array_to_hex_string(infomem_bytes))

                print("-Checking each byte")
                result = True
                for i in range(0, 384):
                    # Shimmer FW protects these bytes 224-229 as the MAC address is stored here
                    if 224 <= i <= 229:
                        pass
                    # TODO unsure why FW is masking byte index 213, bit 5
                    elif i == 213 and (infomem_bytes[i] & 0xEF) == (test_buf[i] & 0xEF):
                        pass
                    elif infomem_bytes[i] != test_buf[i]:
                        print("Index=" + str(i) + ": "
                              + util_shimmer.byte_array_to_hex_string([infomem_bytes[i]])
                              + "!=" + util_shimmer.byte_array_to_hex_string([test_buf[i]]))
                        result = False
                        break
                print("Success" if result else "FAIL")
                print("- Restoring Infomem contents")
                resultRestore = self.shimmer.dock_port.write_infomem(self.infomem_bytes_backup)
                self.assertTrue(result)
                self.assertTrue(resultRestore)
            else:
                print("Error")
                self.assertTrue(False)
            print("")

    # def test_12_read_write_calibration(self):
    #     print("Read Calibration:")
    #     calib_bytes = self.shimmer.dock_port.read_calibration()
    #     if isinstance(calib_bytes, bool):
    #         print("Error")
    #         self.assertTrue(False)
    #     else:
    #         print(util_shimmer.byte_array_to_hex_string(calib_bytes))
    #         self.calib_bytes_backup = calib_bytes
    #     print("")
    #
    #     if self.calib_bytes_backup is not None:
    #         print("Write Calibration:")
    #         lenCalib = 1024 - 2 - 8  # 1024 = full memory size, minus 2 for length bytes, minus 8 for device version info
    #         test_buf = [lenCalib & 0xFF, (lenCalib >> 8) & 0xFF, 0, 0, 0, 0, 0, 0, 0, 0]
    #         test_buf += [(i & 0xFF) for i in range(0, lenCalib)]
    #         print("- Writing test buffer")
    #         result = self.shimmer.dock_port.write_calibration(test_buf)
    #         if result:
    #             print("- Reading back")
    #             calib_bytes = self.shimmer.dock_port.read_calibration()
    #             if isinstance(calib_bytes, bool):
    #                 print("Error")
    #                 self.assertTrue(False)
    #             else:
    #                 print(util_shimmer.byte_array_to_hex_string(calib_bytes))
    #
    #             print("-Checking each byte")
    #             result = True
    #             for i in range(0, 1024):
    #                 # Skip device version info
    #                 if 2 <= i <= 9:
    #                     pass
    #                 elif calib_bytes[i] != test_buf[i]:
    #                     print("Index=" + str(i) + ": "
    #                           + util_shimmer.byte_array_to_hex_string([calib_bytes[i]])
    #                           + "!=" + util_shimmer.byte_array_to_hex_string([test_buf[i]]))
    #                     result = False
    #                     self.assertTrue(False)
    #                     break
    #             print("Success" if result else "FAIL")
    #             print("- Restoring Infomem contents")
    #             result = self.shimmer.dock_port.write_calibration(self.calib_bytes_backup)
    #         else:
    #             print("Error")
    #             self.assertTrue(False)
    #         print("")

    def test_12_factory_test_dock(self):
        print(Fore.LIGHTMAGENTA_EX + "Factory Test Start")
        tx_bytes = shimmer_comms_docked.assemble_tx_packet(UartPacketCmd.WRITE, UartComponent.TEST, FACTORY_TEST.MAIN.value, None)
        if self.shimmer.dock_port.send_uart(tx_bytes):
            end = "//***************************** TEST END *************************************//\r\n"
            while True:
                # response = data.decode('utf-8')
                response = self.shimmer.dock_port.ser.readline().decode('utf-8')
                print(response, end='')
                if response == end:
                    print(Fore.LIGHTMAGENTA_EX + "Factory Test End")
                    break
        else:
            print("Error")
            self.assertTrue(False)

        print("")

    def test_13_enter_bootloader_mode(self):
        if not self.shimmer.is_hardware_version_set():
            if not self.shimmer.dock_port.read_hw_fw_ver():
                print("Error reading hardware version")
                self.assertTrue(False)

        if self.shimmer.is_hardware_shimmer3():
            print("Bootloader command not supported")
            return

        print(Fore.LIGHTMAGENTA_EX + "Request to enter bootloader mode via command")
        timeout_seconds = 5

        tx_bytes = shimmer_comms_docked.assemble_tx_packet(UartPacketCmd.WRITE, UartComponent.MAIN_PROCESSOR, UartProperty.MainProcessor.ENTER_BOOTLOADER, [timeout_seconds])
        response = self.shimmer.dock_port.send_uart(tx_bytes)

        if isinstance(response, bool):
            print("Error")
            self.assertTrue(False)
        elif isinstance(response, int) and response == UartPacketCmd.BAD_CMD_RESPONSE:
            print("Error: BAD_CMD_RESPONSE")
            self.assertTrue(False)
        elif isinstance(response, int) and response == UartPacketCmd.BAD_ARG_RESPONSE:
            print("Error: BAD_ARG_RESPONSE")
            self.assertTrue(False)
        elif isinstance(response, int) and response == UartPacketCmd.BAD_CRC_RESPONSE:
            print("Error: BAD_CRC_RESPONSE")
            self.assertTrue(False)
        else:
            print("Success")

        # Immediately check that device has become unresponsive
        print("Verifying device has become unresponsive...")

        import time
        poll_interval = 0.5
        start_time = time.time()
        still_responding = False

        while time.time() - start_time < timeout_seconds:
            try:
                # Try a simple harmless query to see if the device still responds
                tx_check = shimmer_comms_docked.assemble_tx_packet(
                    UartPacketCmd.READ,
                    UartComponent.MAIN_PROCESSOR,
                    UartProperty.MainProcessor.MAC,
                    None
                )
                rx_check = self.shimmer.dock_port.send_uart(tx_check)

                if isinstance(rx_check, (bytes, bytearray)) and len(rx_check) > 0:
                    print("Warning: device responded unexpectedly.")
                    still_responding = True
                    break
            except Exception as e:
                # If exception occurs, assume it's due to no response (expected)
                print(f"Expected exception (device likely rebooting): {e}")
                break

            time.sleep(poll_interval)

        self.assertFalse(still_responding, "Device responded after reboot command – it should have become unresponsive.")

        # self.shimmer.dock_port.reset_device(boot_mode=False)

        print("")


if __name__ == '__main__':
    unittest.main()
