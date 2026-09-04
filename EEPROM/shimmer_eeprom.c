/*
 * shimmer_eeprom.c
 *
 *  Created on: 31 Jul 2025
 *      Author: MarkNolan
 */

#include "EEPROM/shimmer_eeprom.h"

#include <stdint.h>
#include <string.h>

#include "log_and_stream_externs.h"
#include "log_and_stream_includes.h"

uint8_t eepromIsPresent = 0;
gEepromSensorSettings eepromSensorSettingsPage;

static gEepromBrandDetails eepromBrandDetails;
static uint8_t eepromBrandIsValid = 0;

/* NUL-terminated working copies of the effective name prefixes. Initialised
 * to the compile-time defaults and only overwritten by a valid EEPROM brand
 * record, so callers always get a usable string. */
static char brandBtClassicStr[EEPROM_BRAND_BT_CLASSIC_MAX_CHARS + 1] = BRAND_DEFAULT_BT_CLASSIC;
static char brandBleStr[EEPROM_BRAND_BLE_MAX_CHARS + 1] = BRAND_DEFAULT_BLE;
static char brandUsbProductStr[EEPROM_BRAND_USB_PRODUCT_MAX_CHARS + 1] = BRAND_DEFAULT_USB_PRODUCT;
static char brandUsbManufacturerStr[EEPROM_BRAND_USB_MANUFACTURER_MAX_CHARS + 1]
    = BRAND_DEFAULT_USB_MANUFACTURER;

static uint8_t ShimEeprom_isBrandRecordValid(void);
static void ShimEeprom_seedBrandDefaults(void);
static void ShimEeprom_updateBrandStrings(void);
static uint8_t ShimEeprom_isBrandNameFieldValid(const char *field, uint8_t len, uint8_t maxLen);
static void
ShimEeprom_setBrandNameField(char *fieldPtr, uint8_t *lenPtr, const char *str, uint8_t maxLen);

void ShimEeprom_init(void)
{
  ShimEeprom_setIsPresent(0);
  memset((uint8_t *) &eepromSensorSettingsPage, 0xFF, sizeof(eepromSensorSettingsPage));
  memset((uint8_t *) &eepromBrandDetails, 0xFF, sizeof(eepromBrandDetails));
  eepromBrandIsValid = 0;
  strcpy(brandBtClassicStr, BRAND_DEFAULT_BT_CLASSIC);
  strcpy(brandBleStr, BRAND_DEFAULT_BLE);
  strcpy(brandUsbProductStr, BRAND_DEFAULT_USB_PRODUCT);
  strcpy(brandUsbManufacturerStr, BRAND_DEFAULT_USB_MANUFACTURER);
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
  /* Read brand (advertising name) record, seeding defaults if blank/invalid */
  ShimEeprom_readBrandDetails(1U);
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

/* Copies a NUL-terminated string into a length-prefixed brand record field. */
static void
ShimEeprom_setBrandNameField(char *fieldPtr, uint8_t *lenPtr, const char *str, uint8_t maxLen)
{
  uint8_t i;
  for (i = 0; i < maxLen && str[i] != '\0'; i++)
  {
    fieldPtr[i] = str[i];
  }
  *lenPtr = i;
}

/* A brand name field must be 1..maxLen printable ASCII characters. Commas
 * are rejected as they would corrupt the RN4X "S-,<name>" command. */
static uint8_t ShimEeprom_isBrandNameFieldValid(const char *field, uint8_t len, uint8_t maxLen)
{
  uint8_t i;
  if (len == 0 || len > maxLen)
  {
    return 0;
  }
  for (i = 0; i < len; i++)
  {
    if (field[i] < 0x20 || field[i] > 0x7E || field[i] == ',')
    {
      return 0;
    }
  }
  return 1;
}

static uint8_t ShimEeprom_isBrandRecordValid(void)
{
  return (eepromBrandDetails.magic == EEPROM_BRAND_MAGIC
      && eepromBrandDetails.layoutVer == EEPROM_BRAND_LAYOUT_VER
      && ShimEeprom_isBrandNameFieldValid(eepromBrandDetails.btClassic,
          eepromBrandDetails.btClassicLen, EEPROM_BRAND_BT_CLASSIC_MAX_CHARS)
      && ShimEeprom_isBrandNameFieldValid(eepromBrandDetails.ble,
          eepromBrandDetails.bleLen, EEPROM_BRAND_BLE_MAX_CHARS)
      && ShimEeprom_isBrandNameFieldValid(eepromBrandDetails.usbProduct,
          eepromBrandDetails.usbProductLen, EEPROM_BRAND_USB_PRODUCT_MAX_CHARS)
      && ShimEeprom_isBrandNameFieldValid(eepromBrandDetails.usbManufacturer,
          eepromBrandDetails.usbManufacturerLen, EEPROM_BRAND_USB_MANUFACTURER_MAX_CHARS)
      && checkCrc(CRC_2BYTES_ENABLED, &eepromBrandDetails.rawBytes[0],
          EEPROM_BRAND_DETAILS_SIZE - 2U));
}

/* Builds a brand record holding the compile-time default names and writes it
 * to the EEPROM. Called at boot when the stored record is blank or invalid so
 * that, from then on, the EEPROM record is the single source of truth that
 * host software can read back. */
static void ShimEeprom_seedBrandDefaults(void)
{
  memset((uint8_t *) &eepromBrandDetails, 0, sizeof(eepromBrandDetails));
  eepromBrandDetails.magic = EEPROM_BRAND_MAGIC;
  eepromBrandDetails.layoutVer = EEPROM_BRAND_LAYOUT_VER;
  eepromBrandDetails.flags = (BRAND_PLATFORM_CURRENT << EEPROM_BRAND_PLATFORM_SHIFT)
      & EEPROM_BRAND_PLATFORM_MASK;
  ShimEeprom_setBrandNameField(&eepromBrandDetails.btClassic[0],
      &eepromBrandDetails.btClassicLen, BRAND_DEFAULT_BT_CLASSIC,
      EEPROM_BRAND_BT_CLASSIC_MAX_CHARS);
  ShimEeprom_setBrandNameField(&eepromBrandDetails.ble[0],
      &eepromBrandDetails.bleLen, BRAND_DEFAULT_BLE, EEPROM_BRAND_BLE_MAX_CHARS);
  ShimEeprom_setBrandNameField(&eepromBrandDetails.usbProduct[0],
      &eepromBrandDetails.usbProductLen, BRAND_DEFAULT_USB_PRODUCT,
      EEPROM_BRAND_USB_PRODUCT_MAX_CHARS);
  ShimEeprom_setBrandNameField(&eepromBrandDetails.usbManufacturer[0],
      &eepromBrandDetails.usbManufacturerLen, BRAND_DEFAULT_USB_MANUFACTURER,
      EEPROM_BRAND_USB_MANUFACTURER_MAX_CHARS);
  calculateCrcAndInsert(CRC_2BYTES_ENABLED, &eepromBrandDetails.rawBytes[0],
      EEPROM_BRAND_DETAILS_SIZE - 2U);

  eepromWrite(EEPROM_ADDRESS_BRAND_DETAILS, sizeof(eepromBrandDetails.rawBytes),
      &eepromBrandDetails.rawBytes[0]);
}

/* Refreshes the NUL-terminated working strings from the brand record when it
 * is valid, or from the compile-time defaults when it is not. */
static void ShimEeprom_updateBrandStrings(void)
{
  if (eepromBrandIsValid)
  {
    memcpy(brandBtClassicStr, eepromBrandDetails.btClassic, eepromBrandDetails.btClassicLen);
    brandBtClassicStr[eepromBrandDetails.btClassicLen] = '\0';
    memcpy(brandBleStr, eepromBrandDetails.ble, eepromBrandDetails.bleLen);
    brandBleStr[eepromBrandDetails.bleLen] = '\0';
    memcpy(brandUsbProductStr, eepromBrandDetails.usbProduct, eepromBrandDetails.usbProductLen);
    brandUsbProductStr[eepromBrandDetails.usbProductLen] = '\0';
    memcpy(brandUsbManufacturerStr, eepromBrandDetails.usbManufacturer,
        eepromBrandDetails.usbManufacturerLen);
    brandUsbManufacturerStr[eepromBrandDetails.usbManufacturerLen] = '\0';
  }
  else
  {
    strcpy(brandBtClassicStr, BRAND_DEFAULT_BT_CLASSIC);
    strcpy(brandBleStr, BRAND_DEFAULT_BLE);
    strcpy(brandUsbProductStr, BRAND_DEFAULT_USB_PRODUCT);
    strcpy(brandUsbManufacturerStr, BRAND_DEFAULT_USB_MANUFACTURER);
  }
}

/**
 * Reads and validates the brand (advertising name) record from the EEPROM.
 *
 * @param seedIfInvalid  When set (boot path), a blank or invalid record is
 *        overwritten with this platform's compile-time defaults. Must be 0
 *        when re-reading after a host expansion-board-memory write so that a
 *        multi-chunk host write sequence is never raced by a seed write.
 */
void ShimEeprom_readBrandDetails(uint8_t seedIfInvalid)
{
  eepromRead(EEPROM_ADDRESS_BRAND_DETAILS, sizeof(eepromBrandDetails.rawBytes),
      &eepromBrandDetails.rawBytes[0]);

  eepromBrandIsValid = ShimEeprom_isBrandRecordValid();

  if (seedIfInvalid && !eepromBrandIsValid)
  {
    /* Blank or invalid record: seed this platform's defaults so that, from
     * here on, the EEPROM record is the single source of truth host software
     * can read back. A valid record is always honoured - there is no
     * re-seeding based on which platform wrote it, as the unified PCB means
     * the EEPROM cannot move between hardware models. */
    ShimEeprom_seedBrandDefaults();
    eepromBrandIsValid = ShimEeprom_isBrandRecordValid();
  }

  ShimEeprom_updateBrandStrings();
}

uint8_t ShimEeprom_isBrandValid(void)
{
  return eepromBrandIsValid;
}

const char *ShimEeprom_getBrandBtClassic(void)
{
  return &brandBtClassicStr[0];
}

const char *ShimEeprom_getBrandBle(void)
{
  return &brandBleStr[0];
}

const char *ShimEeprom_getBrandUsbProduct(void)
{
  return &brandUsbProductStr[0];
}

const char *ShimEeprom_getBrandUsbManufacturer(void)
{
  return &brandUsbManufacturerStr[0];
}

#if defined(SHIMMER3R)
/* Shimmer3R stores baud rate as the actual baud rate value, but Shimmer3 and
 * Shimmer4 store it as an index representing the baud rate to use. To
 * maintain compatibility with both formats, we need to convert the actual
 * baud rate value to the corresponding index when updating/checking the radio
 * details for Shimmer3R. */
static uint8_t ShimEeprom_baudRateToEnum(uint32_t baudRateToUse)
{
  switch (baudRateToUse)
  {
    case 0: //Default to 115200 if baud rate not set
    case 115200L:
      return BAUD_115200;
    case 1200L:
      return BAUD_1200;
    case 2400L:
      return BAUD_2400;
    case 4800L:
      return BAUD_4800;
    case 9600L:
      return BAUD_9600;
    case 19200L:
      return BAUD_19200;
    case 38400L:
      return BAUD_38400;
    case 57600L:
      return BAUD_57600;
    case 230400L:
      return BAUD_230400;
    case 460800L:
      return BAUD_460800;
    case 921600L:
      return BAUD_921600;
    case 1000000L:
      return BAUD_1000000;
    case 2000000L:
      return BAUD_2000000;
    default:
      return BAUD_INVALID; //Invalid/unknown baud rate
  }
}
#endif

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
  eepromSensorSettingsPage.baudRate
      = ShimEeprom_baudRateToEnum(ShimBt_getBtBaudRateToUse());
#endif
  //leave eepromSensorSettingsPage.bleDisabled as is
}

uint8_t ShimEeprom_areRadioDetailsIncorrect(void)
{
  return (eepromSensorSettingsPage.radioHwVer != ShimEeprom_getRadioHwVersion()
      || eepromSensorSettingsPage.baudRate == BAUD_INVALID
#if defined(SHIMMER3) || defined(SHIMMER4_SDK)
      || (isBtDeviceRn41orRN42() && eepromSensorSettingsPage.baudRate != BAUD_115200)
      || (isBtDeviceRn4678() && eepromSensorSettingsPage.baudRate != ShimBt_getBtBaudRateToUse())
      || (!ShimBrd_doesDeviceSupportBle() && eepromSensorSettingsPage.bleEnabled != 0)
#else
      || eepromSensorSettingsPage.baudRate
          != ShimEeprom_baudRateToEnum(ShimBt_getBtBaudRateToUse())
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
 * Resets all Bluetooth error counters in eepromSensorSettingsPage to zero.
 */
void ShimEeprom_resetBtErrorCounts(void)
{
  eepromSensorSettingsPage.btCntDisconnectWhileStreaming = 0;
  eepromSensorSettingsPage.btCntUnsolicitedReboot = 0;
  eepromSensorSettingsPage.btCntRtsLockup = 0;
  eepromSensorSettingsPage.btCntDataRateTestBlockage = 0;
}
#endif

gEepromSensorSettings *ShimEeprom_getSensorSettingsPage(void)
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
  uint16_t targetAddr = EEPROM_ADDRESS_BLUETOOTH_DETAILS_HOST_OFFSET_ALIAS + RADIO_SETTINGS_IDX;
  uint16_t brandStart = EEPROM_ADDRESS_BRAND_DETAILS - CAT24C16_PAGE_SIZE;
  uint16_t brandEnd = brandStart + EEPROM_BRAND_DETAILS_SIZE - 1;

  /* memLength == 0 has to be rejected before writeEnd is used: it makes
   * writeEnd = memOffset - 1, an inverted range that underflows to 0xFFFF at
   * offset 0 and, anywhere inside the brand record, satisfies the overlap test
   * below and forces a re-read for a write that never happened. It also hands
   * the platform's eepromWrite() a zero-length transfer. A write of nothing
   * has no meaning in this command, so NACK it rather than define one. */
  if ((memLength > 0) && (memLength <= 128) && (writeEnd < EEPROM_AVAILABLE_SIZE))
  {
    eepromWrite(memOffset + CAT24C16_PAGE_SIZE, (uint16_t) memLength, buf);

    /* Handle if the BLE/BT state is being changed */
    if (writeStart <= targetAddr && writeEnd >= targetAddr)
    {
      ShimEeprom_readSensorSettingsPage();
    }

    /* Handle if the brand record is being provisioned by the host. No
     * seeding here - the record only re-seeds at boot - so a multi-chunk
     * host write can't be raced. New names apply at the next BT init. */
    if (writeStart <= brandEnd && writeEnd >= brandStart)
    {
      ShimEeprom_readBrandDetails(0U);
    }

    return 1;
  }
  return 0;
}
