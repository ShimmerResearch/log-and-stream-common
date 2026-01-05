#include <stdint.h>

#include "shimmer_crc.h"

#include "Platform/platform_api.h"

/**
 * Calculates the CRC for a payload and appends the CRC bytes to the same buffer.
 *
 * @param crcMode  CRC mode control. If CRC_OFF, no CRC is calculated or appended.
 *                 If CRC_2BYTES_ENABLED, two CRC bytes are appended; otherwise one byte.
 * @param aryPtr   Pointer to the buffer that contains the payload. The buffer must have
 *                 enough free space after the payload to hold the CRC bytes.
 * @param len      Length of the payload in bytes, before any CRC bytes are added.
 *
 * The function modifies the buffer in-place by writing the CRC byte(s) immediately
 * after the payload bytes at positions aryPtr[len] and, if enabled, aryPtr[len + 1].
 */
void calculateCrcAndInsert(COMMS_CRC_MODE crcMode, uint8_t *aryPtr, uint8_t len)
{
  uint32_t crc_value;
  if (crcMode != CRC_OFF)
  {
    crc_value = platform_crcData(aryPtr, len);
    *(aryPtr + len++) = crc_value & 0xFF;

    if (crcMode == CRC_2BYTES_ENABLED)
    {
      *(aryPtr + len++) = (crc_value & 0xFF00) >> 8;
    }
  }
}

/**
 * @brief Verify the CRC bytes appended to a payload buffer.
 *
 * This function recomputes the CRC over a payload and compares it against
 * one or two CRC bytes stored immediately after the payload in the same
 * buffer. The number of CRC bytes to check depends on @p crcMode.
 *
 * @param crcMode     CRC mode selector. If set to CRC_OFF, no CRC check is
 *                    performed and the function returns success. If set to
 *                    CRC_2BYTES_ENABLED, both low and high CRC bytes are
 *                    verified. For other non-CRC_OFF modes, only the low
 *                    CRC byte is verified.
 * @param aryPtr      Pointer to the buffer that contains the payload
 *                    followed by its CRC byte(s). The payload occupies the
 *                    first @p payloadLen bytes of this buffer; the CRC
 *                    bytes must be stored immediately after the payload.
 * @param payloadLen  Length of the payload in bytes, excluding any CRC
 *                    bytes already present in the buffer.
 *
 * @return 1 if the CRC is considered valid (including when CRC is disabled
 *         via CRC_OFF), or 0 if a CRC mismatch is detected.
 */
uint8_t checkCrc(COMMS_CRC_MODE crcMode, uint8_t *aryPtr, uint8_t payloadLen)
{
  uint32_t crc_value_calc;
  if (crcMode != CRC_OFF)
  {
    crc_value_calc = platform_crcData(aryPtr, payloadLen);

    if ((crc_value_calc & 0xFF) != *(aryPtr + payloadLen))
    {
      return 0;
    }

    if (crcMode == CRC_2BYTES_ENABLED
        && (((crc_value_calc >> 8) & 0xFF) != *(aryPtr + payloadLen + 1)))
    {
      return 0;
    }
  }

  return 1;
}


/**
 * @brief  Self-test for the CRC driver.
 *
 * @return 0 if all CRC test cases pass; non-zero (1) if any test case fails.
 */
uint8_t testCrcDriver(void)
{
  uint8_t crcTestArray0Remainder[]
      = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x00, 0x00 };
  calculateCrcAndInsert(CRC_2BYTES_ENABLED, crcTestArray0Remainder, 8);
  if (crcTestArray0Remainder[8] != 0xAA || crcTestArray0Remainder[9] != 0x48)
  {
    return 1;
  }
  uint8_t crcTestArray1Remainder[]
      = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x00, 0x00 };
  calculateCrcAndInsert(CRC_2BYTES_ENABLED, crcTestArray1Remainder, 9);
  if (crcTestArray1Remainder[9] != 0x5D || crcTestArray1Remainder[10] != 0x2A)
  {
    return 1;
  }
  uint8_t crcTestArray2Remainder[]
      = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x00, 0x00 };
  calculateCrcAndInsert(CRC_2BYTES_ENABLED, crcTestArray2Remainder, 10);
  if (crcTestArray2Remainder[10] != 0x17 || crcTestArray2Remainder[11] != 0x8B)
  {
    return 1;
  }
  uint8_t crcTestArray3Remainder[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x00, 0x00 };
  calculateCrcAndInsert(CRC_2BYTES_ENABLED, crcTestArray3Remainder, 11);
  if (crcTestArray3Remainder[11] != 0x4E || crcTestArray3Remainder[12] != 0x79)
  {
    return 1;
  }
  return 0;
}
