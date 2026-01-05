
#ifndef SHIMMER_CRC_H
#define SHIMMER_CRC_H

#include <stdint.h>

#define CRC_INIT 0xB0CA

// CRC modes for communication payloads.
// Note: CRC_MAX_SUPPORTED_BYTES is a sentinel value used for validation
// and bounds checking and MUST NOT be used as an actual CRC mode.
typedef enum
{
  CRC_OFF,
  CRC_1BYTE_ENABLED,
  CRC_2BYTES_ENABLED,
  CRC_MAX_SUPPORTED_BYTES
} COMMS_CRC_MODE;

void calculateCrcAndInsert(COMMS_CRC_MODE crcMode, uint8_t *aryPtr, uint8_t len);
uint8_t checkCrc(COMMS_CRC_MODE crcMode, uint8_t *aryPtr, uint8_t payloadLen);
uint8_t testCrcDriver(void);

#endif //SHIMMER_CRC_H
