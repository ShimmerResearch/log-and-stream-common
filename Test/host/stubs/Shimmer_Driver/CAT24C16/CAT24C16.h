/*
 * Host-test stub for the CAT24C16 EEPROM driver.
 *
 * The real header lives in the consuming firmware
 * (Shimmer_Driver/CAT24C16/CAT24C16.h), not in this repository, but
 * EEPROM/shimmer_eeprom.h and Boards/shimmer_boards.h both include it for the
 * page geometry. These two values size daughter_card_id_page and every EEPROM
 * offset derived from it, so they MUST match the real driver.
 */
#ifndef HOST_TEST_STUB_CAT24C16_H
#define HOST_TEST_STUB_CAT24C16_H

#include <stdint.h>

#define CAT24C16_PAGE_SIZE  16
#define CAT24C16_TOTAL_SIZE 2048

uint8_t CAT24C16_init(void);
void CAT24C16_read(uint16_t address, uint16_t length, uint8_t *outBuffer);
void CAT24C16_write(uint16_t address, uint16_t length, uint8_t *inBuffer);

#endif /* HOST_TEST_STUB_CAT24C16_H */
