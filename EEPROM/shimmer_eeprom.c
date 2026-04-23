/*
 * shimmer_eeprom.c
 *
 *  Created on: 31 Jul 2025
 *      Author: MarkNolan
 */

#include "EEPROM/shimmer_eeprom.h"

#include <stdint.h>

#include "log_and_stream_externs.h"
#include "log_and_stream_includes.h"

uint8_t eepromIsPresent = 0;
gEepromBtSettings eepromSensorSettingsPage;

void ShimEeprom_init(void)
{
  ShimEeprom_setIsPresent(0);
  memset((uint8_t *) &eepromSensorSettingsPage, 0xFF, sizeof(eepromSensorSettingsPage));
}

void ShimEeprom_setIsPresent(uint8_t eeprom_is_preset)
{
  eepromIsPresent = eeprom_is_preset;
}

uint8_t ShimEeprom_isPresent(void)
{
  return eepromIsPresent;
}

void ShimEeprom_readAll(void)
{
  /* Read Daughter card ID */
  ShimEeprom_readHwDetails();
  //Read Bluetooth configuration parameters from EEPROM
  ShimEeprom_readSensorSettingsPage();
}

void ShimEeprom_readHwDetails(void)
{
  eepromRead(EEPROM_ADDRESS_HW_DETAILS, CAT24C16_PAGE_SIZE, ShimBrd_getDaughtCardIdPtr());
}

void ShimEeprom_readSensorSettingsPage(void)
{
  eepromRead(EEPROM_ADDRESS_BLUETOOTH_DETAILS, sizeof(eepromSensorSettingsPage.rawBytes),
      &eepromSensorSettingsPage.rawBytes[0]);
}

void ShimEeprom_writeSensorSettingsPage(void)
{
  eepromWrite(EEPROM_ADDRESS_BLUETOOTH_DETAILS, sizeof(eepromSensorSettingsPage.rawBytes),
      &eepromSensorSettingsPage.rawBytes[0]);
}

void ShimEeprom_updateRadioDetails(void)
{
  eepromSensorSettingsPage.radioHwVer = (uint8_t) ShimEeprom_getRadioHwVersion();
#if defined(SHIMMER3) || defined(SHIMMER4_SDK)
  if (isBtDeviceRn41orRN42())
  {
    eepromSensorSettingsPage.baudRate = BAUD_115200;
    eepromSensorSettingsPage.bleEnabled = 0; //BLE not supported in RN42
  }
  else
  {
    eepromSensorSettingsPage.baudRate = ShimBt_getBtBaudRateToUse();
  }
#else
  /* Shimmer3R stores baud rate as the actual baud rate value, but Shimmer3 and
   * Shimmer4 store it as an index representing the baud rate to use. To
   * maintain compatibility with both formats, we need to convert the actual
   * baud rate value to the corresponding index when updating the radio details
   * for Shimmer3R.*/
  uint32_t baudRateToUse = ShimBt_getBtBaudRateToUse();
  uint8_t baudEnum;
  switch (baudRateToUse)
  {
    case 0: //Default to 115200 if baud rate not set
    case 115200L:
      baudEnum = BAUD_115200;
      break;
    case 1200L:
      baudEnum = BAUD_1200;
      break;
    case 2400L:
      baudEnum = BAUD_2400;
      break;
    case 4800L:
      baudEnum = BAUD_4800;
      break;
    case 9600L:
      baudEnum = BAUD_9600;
      break;
    case 19200L:
      baudEnum = BAUD_19200;
      break;
    case 38400L:
      baudEnum = BAUD_38400;
      break;
    case 57600L:
      baudEnum = BAUD_57600;
      break;
    case 230400L:
      baudEnum = BAUD_230400;
      break;
    case 460800L:
      baudEnum = BAUD_460800;
      break;
    case 921600L:
      baudEnum = BAUD_921600;
      break;
    case 1000000L:
      baudEnum = BAUD_1000000;
      break;
    case 2000000L:
      baudEnum = BAUD_2000000;
      break;
    default:
      baudEnum = 0xFF; //Invalid/unknown baud rate
      break;
  }

  eepromSensorSettingsPage.baudRate = baudEnum;
#endif
  //leave eepromBtSettings.bleDisabled as is
}

uint8_t ShimEeprom_areRadioDetailsIncorrect(void)
{
  return (eepromSensorSettingsPage.radioHwVer != ShimEeprom_getRadioHwVersion()
      || eepromSensorSettingsPage.baudRate == 0xFF
#if defined(SHIMMER3) || defined(SHIMMER4_SDK)
      || (isBtDeviceRn41orRN42() && eepromSensorSettingsPage.baudRate != BAUD_115200)
      || (isBtDeviceRn4678() && eepromSensorSettingsPage.baudRate != ShimBt_getBtBaudRateToUse())
      || (!ShimBrd_doesDeviceSupportBle() && eepromSensorSettingsPage.bleEnabled != 0)
#else
      || eepromSensorSettingsPage.baudRate != ShimBt_getBtBaudRateToUse()
#endif
  );
}

#if defined(SHIMMER3)
/**
 * Checks if any Bluetooth error count fields are set to the invalid value
 * (0xFFFF). If so, resets all error counts to zero.
 *
 * @return 1 if any error count was invalid and reset, 0 otherwise.
 * @sideeffect Calls ShimEeprom_resetBtErrorCounts() if any count is invalid.
 */
uint8_t ShimEeprom_checkBtErrorCounts(void)
{
  if (eepromSensorSettingsPage.btCntDisconnectWhileStreaming == 0xFFFF
      || eepromSensorSettingsPage.btCntUnsolicitedReboot == 0xFFFF
      || eepromSensorSettingsPage.btCntRtsLockup == 0xFFFF
      || eepromSensorSettingsPage.btCntDataRateTestBlockage == 0xFFFF)
  {
    return 1;
  }
  return 0;
}

/**
 * Resets all Bluetooth error counters in eepromBtSettings to zero.
 */
void ShimEeprom_resetBtErrorCounts(void)
{
  eepromSensorSettingsPage.btCntDisconnectWhileStreaming = 0;
  eepromSensorSettingsPage.btCntUnsolicitedReboot = 0;
  eepromSensorSettingsPage.btCntRtsLockup = 0;
  eepromSensorSettingsPage.btCntDataRateTestBlockage = 0;
}
#endif

gEepromBtSettings *ShimEeprom_getSensorSettingsPage(void)
{
  return &eepromSensorSettingsPage;
}

uint8_t ShimEeprom_isBleEnabled(void)
{
  return eepromSensorSettingsPage.bleEnabled;
}

uint8_t ShimEeprom_isBtClassicEnabled(void)
{
  return eepromSensorSettingsPage.btClassicEnabled;
}

enum RADIO_HARDWARE_VERSION ShimEeprom_getRadioHwVersion(void)
{
#if defined(SHIMMER4_SDK)
  return RN42;
#elif defined(SHIMMER3)
  if (isBtDeviceRn42())
  {
    return RN42;
  }
  else if (isBtDeviceRn4678())
  {
    return RN4678;
  }
  else if (isBtDeviceRn41())
  {
    return RN41;
  }
  return BT_HW_VER_UNKNOWN;
#else
  return CYW20820;
#endif
}

/* This function skips the first page of the EEPROM as this is reserved
 * for HW information and therefore an offset of 0 is actually the start
 * of the second page in the EEPROM. */
uint8_t ShimEeprom_writeDaughterCardMem(uint16_t memOffset, uint8_t memLength, uint8_t *buf)
{
  uint16_t writeStart = memOffset;
  uint16_t writeEnd = memOffset + memLength - 1;
  uint16_t targetAddr = EEPROM_ADDRESS_BLUETOOTH_DETAILS_MINUS_OFFSET + RADIO_SETTINGS_IDX;

  if ((memLength <= 128) && (writeEnd < EEPROM_AVAILABLE_SIZE))
  {
    eepromWrite(memOffset + CAT24C16_PAGE_SIZE, (uint16_t) memLength, buf);

    /* Handle if the BLE/BT state is being changed */
    if (writeStart <= targetAddr && writeEnd >= targetAddr)
    {
      ShimEeprom_readSensorSettingsPage();
    }

    return 1;
  }
  return 0;
}
