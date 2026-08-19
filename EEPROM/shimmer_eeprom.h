/*
 * shimmer_eeprom.h
 *
 *  Created on: 31 Jul 2025
 *      Author: MarkNolan
 */

#ifndef LOG_AND_STREAM_COMMON_COMMS_SHIMMER_EEPROM_H_
#define LOG_AND_STREAM_COMMON_COMMS_SHIMMER_EEPROM_H_

#include <stdint.h>

#include "CAT24C16/CAT24C16.h"

//Bluetooth information is stored in the last EEPROM page
#define EEPROM_ADDRESS_HW_DETAILS 0
#define EEPROM_ADDRESS_BLUETOOTH_DETAILS \
  (CAT24C16_TOTAL_SIZE - CAT24C16_PAGE_SIZE)
/* Host-facing daughter-card-memory offsets skip the first (HW details) page,
 * so the Bluetooth details page sits at this offset in host address space.
 * Note: this is NOT a marker for a free/spare EEPROM page. */
#define EEPROM_ADDRESS_BLUETOOTH_DETAILS_HOST_OFFSET_ALIAS \
  (EEPROM_ADDRESS_BLUETOOTH_DETAILS - CAT24C16_PAGE_SIZE)

#define EEPROM_AVAILABLE_SIZE     (CAT24C16_TOTAL_SIZE - CAT24C16_PAGE_SIZE)

/* Brand (advertising name) record, stored in the 4 pages directly below the
 * Bluetooth details page. Absolute bytes 1968-2031 (host offset 1952). */
#define EEPROM_BRAND_DETAILS_SIZE (4 * CAT24C16_PAGE_SIZE)
#define EEPROM_ADDRESS_BRAND_DETAILS \
  (EEPROM_ADDRESS_BLUETOOTH_DETAILS - EEPROM_BRAND_DETAILS_SIZE)

#define EEPROM_BRAND_MAGIC \
  0x5342 /* "SB"; stored little-endian, so a raw hexdump reads 0x42,0x53 ("BS") */
#define EEPROM_BRAND_LAYOUT_VER            1

#define EEPROM_BRAND_BT_CLASSIC_MAX_CHARS  16
#define EEPROM_BRAND_BLE_MAX_CHARS         10
#define EEPROM_BRAND_USB_MAX_CHARS         16

/* Flags byte in the brand record */
#define EEPROM_BRAND_FLAG_CUSTOMER_BRANDED 0x01
#define EEPROM_BRAND_PLATFORM_MASK         0x06
#define EEPROM_BRAND_PLATFORM_SHIFT        1

enum EEPROM_BRAND_PLATFORM
{
  BRAND_PLATFORM_UNKNOWN = 0,
  BRAND_PLATFORM_SHIMMER3 = 1,
  BRAND_PLATFORM_SHIMMER3R = 2,
  BRAND_PLATFORM_SHIMMER4_SDK = 3,
};

/* Default advertising/product name prefixes. These are the stock values that
 * get seeded into the EEPROM brand record when it is blank or invalid, and
 * are the fallback when no EEPROM is fitted. */
#if defined(SHIMMER3R)
#define BRAND_DEFAULT_BT_CLASSIC "Shimmer3R"
#define BRAND_DEFAULT_BLE        "Shimmer3R"
#define BRAND_DEFAULT_USB        "Shimmer"
#define BRAND_PLATFORM_CURRENT   BRAND_PLATFORM_SHIMMER3R
#elif defined(SHIMMER4_SDK)
#define BRAND_DEFAULT_BT_CLASSIC "Shimmer4"
#define BRAND_DEFAULT_BLE        "S4BLE"
#define BRAND_DEFAULT_USB        "Shimmer"
#define BRAND_PLATFORM_CURRENT   BRAND_PLATFORM_SHIMMER4_SDK
#else /* SHIMMER3 */
#define BRAND_DEFAULT_BT_CLASSIC "Shimmer3"
#define BRAND_DEFAULT_BLE        "S3BLE"
#define BRAND_DEFAULT_USB        "Shimmer"
#define BRAND_PLATFORM_CURRENT   BRAND_PLATFORM_SHIMMER3
#endif

//Indices of important daughter card information
enum EEPROM_HARDWARE_REVISON
{
  DAUGHT_CARD_ID = 0,
  DAUGHT_CARD_REV = 1,
  DAUGHT_CARD_SPECIAL_REV = 2,
};

/* Order here needs to be maintained as it's saved to the EEPROM */
enum RADIO_HARDWARE_VERSION
{
  RN42 = 0U,
  RN4678 = 1U,
  RN41 = 2U,
  CYW20820 = 3U,
  BT_HW_VER_UNKNOWN = 0xFF,
};

enum EEPROM_BLUETOOTH_SETTINGS
{
  RADIO_TYPE_IDX = 0,
  BAUD_RATE_IDX = 1,
  RADIO_SETTINGS_IDX = 2,
};

typedef struct
{
  uint8_t exp_brd_id;
  uint8_t exp_brd_major;
  uint8_t exp_brd_minor;
} shimmer_expansion_brd;

typedef union
{
  uint8_t raw[CAT24C16_PAGE_SIZE];

  struct __attribute__((packed))
  {
    shimmer_expansion_brd expansion_brd;
    uint8_t padding[13];
  };
} daughter_card_id_page;

typedef union
{
  uint8_t rawBytes[CAT24C16_PAGE_SIZE];

  struct __attribute__((packed))
  {
    uint8_t radioHwVer;
    uint8_t baudRate;

    //Byte index 2
    uint8_t bleEnabled       : 1;
    uint8_t btClassicEnabled : 1;
    /* USB Speed. 1 = HS, 0 = FS (only applicable for Shimmer3R) */
    uint8_t usbHighSpeed   : 1;
    uint8_t unusedIdx3Bit3 : 1;
    uint8_t unusedIdx3Bit4 : 1;
    uint8_t unusedIdx3Bit5 : 1;
    uint8_t unusedIdx3Bit6 : 1;
    uint8_t unusedIdx3Bit7 : 1;

    uint16_t btCntDisconnectWhileStreaming; //Shimmer3 RN4678 error count
    uint16_t btCntUnsolicitedReboot;        //Shimmer3 RN4678 error count
    uint16_t btCntRtsLockup;                //Shimmer3 RN4678 error count
    uint16_t btCntDataRateTestBlockage;     //Shimmer3 RN4678 error count

    uint8_t padding[5];
  };
} gEepromSensorSettings;

/* Brand (advertising name) record. Name fields are length-prefixed via the
 * *Len bytes and are NOT NUL-terminated in the EEPROM. The record is only
 * honoured when magic, layout version, lengths, character set and CRC all
 * check out - otherwise it is re-seeded with the platform defaults at boot. */
typedef union
{
  uint8_t rawBytes[EEPROM_BRAND_DETAILS_SIZE];

  struct __attribute__((packed))
  {
    uint16_t magic;    /* EEPROM_BRAND_MAGIC */
    uint8_t layoutVer; /* EEPROM_BRAND_LAYOUT_VER */
    uint8_t flags;     /* EEPROM_BRAND_FLAG_* + EEPROM_BRAND_PLATFORM_* */
    uint8_t btClassicLen;
    uint8_t bleLen;
    uint8_t usbLen;
    char btClassic[EEPROM_BRAND_BT_CLASSIC_MAX_CHARS]; /* Classic BT name prefix */
    char ble[EEPROM_BRAND_BLE_MAX_CHARS];              /* BLE name prefix */
    char usb[EEPROM_BRAND_USB_MAX_CHARS]; /* USB product prefix / manufacturer */
    uint8_t padding[13];
    uint8_t crc[2]; /* 2-byte CRC over rawBytes[0..61] (comms CRC) */
  };
} gEepromBrandDetails;

void ShimEeprom_init(void);
void ShimEeprom_setIsPresent(uint8_t eeprom_is_preset);
uint8_t ShimEeprom_isPresent(void);
void ShimEeprom_readAll(void);
void ShimEeprom_readHwDetails(void);
void ShimEeprom_readSensorSettingsPage(void);
void ShimEeprom_writeSensorSettingsPage(void);
void ShimEeprom_updateRadioDetails(void);
uint8_t ShimEeprom_areRadioDetailsIncorrect(void);
#if defined(SHIMMER3)
uint8_t ShimEeprom_checkBtErrorCounts(void);
void ShimEeprom_resetBtErrorCounts(void);
#endif
gEepromSensorSettings *ShimEeprom_getSensorSettingsPage(void);
void ShimEeprom_readBrandDetails(uint8_t seedIfInvalid);
uint8_t ShimEeprom_isBrandValid(void);
uint8_t ShimEeprom_isBrandCustomer(void);
const char *ShimEeprom_getBrandBtClassic(void);
const char *ShimEeprom_getBrandBle(void);
const char *ShimEeprom_getBrandUsb(void);
uint8_t ShimEeprom_isBleEnabled(void);
uint8_t ShimEeprom_isBtClassicEnabled(void);
enum RADIO_HARDWARE_VERSION ShimEeprom_getRadioHwVersion(void);
uint8_t ShimEeprom_writeDaughterCardMem(uint16_t memOffset, uint8_t memLength, uint8_t *buf);

#endif /* LOG_AND_STREAM_COMMON_COMMS_SHIMMER_EEPROM_H_ */
