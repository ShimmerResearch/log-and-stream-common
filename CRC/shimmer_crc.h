
#ifndef SHIMMER_CRC_H
#define SHIMMER_CRC_H

#include <stdint.h>

#define CRC_INIT 0xB0CA

//CRC modes - the ordinal corresponds to the number of CRC bytes
typedef enum
{
  CRC_OFF,
  CRC_1BYTES_ENABLED,
  CRC_2BYTES_ENABLED,
  CRC_MAX_SUPPORTED_BYTES
} COMMS_CRC_MODE;

void calculateCrcAndInsert(uint8_t crcMode, uint8_t *aryPtr, uint8_t len);
uint8_t checkCrc(uint8_t crcMode, uint8_t *aryPtr, uint8_t payloadLen);
uint8_t testCrcDriver(void);

#endif //SHIMMER_CRC_H
