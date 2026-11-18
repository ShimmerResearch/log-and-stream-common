/*
 * cat24c16.h
 *
 * The CAT24C16 serial EEPROM: 2048 bytes in 8 blocks of 256, written 16 bytes
 * (one page) at a time. Shimmer3 and Shimmer3R both fit it, on the expansion
 * board, and this one driver serves both (DEV-1496). It reaches the bus only
 * through the callbacks in cat24c16dev_ctx_t, which each firmware fills in from
 * its own I2C HAL in Shimmer_Driver/CAT24C16/hal_CAT24C16.c; power, bus
 * bring-up and which bus the part is on stay there too.
 *
 * What lives at which address is EEPROM/shimmer_eeprom.h and
 * docs/SHIMMER3_EEPROM_MEMORY_MAP.md.
 *
 *  Created on: 21 Nov 2013 (Shimmer3), May 2016 (Shimmer3R)
 *      Author: Weibo Pan
 */

#ifndef EEPROM_CAT24C16_H_
#define EEPROM_CAT24C16_H_

#include <stdint.h>

//7-bit I2C address. Its low 3 bits carry bits 10:8 of the memory address, so
//each 256-byte block answers at its own address, 0x50 to 0x57.
#define CAT24C16_ADDR                0x50
#define CAT24C16_PAGE_SIZE           16
#define CAT24C16_BLOCK_SIZE          256
#define CAT24C16_TOTAL_SIZE          2048
//Write cycle time, tWR: 5 ms max (CAT24C16 datasheet, A.C. Characteristics).
//The part does not acknowledge its address until a write cycle has ended
//(Acknowledge Polling), so a transfer started inside one fails.
#define CAT24C16_WRITE_CYCLE_MS      5
#define CAT24C16_OUT_OF_BOUNDS_ERROR 1000

//The region CAT24C16_test() uses as scratch: absolute bytes 16-143, which are
//host offsets 0-127 of the daughter-card memory
#define CAT24C16_TEST_OFFSET         16
#define CAT24C16_TEST_SIZE           (8 * CAT24C16_PAGE_SIZE)
//How many times CAT24C16_test() tries to put the test region back
#define CAT24C16_TEST_RESTORE_TRIES  3

enum CAT24C16_TEST
{
  EEPROM_TEST_PASS = 0,
  EEPROM_TEST_FAIL_INITIAL_BACKUP,
  EEPROM_TEST_FAIL_WRITING_BUF,
  EEPROM_TEST_FAIL_READING_BUF,
  EEPROM_TEST_FAIL_BUF_COMPARISON,
  //Every pass succeeded but the test region could not be put back
  EEPROM_TEST_FAIL_RESTORING_BACKUP,
};

//OR'd into EEPROM_TEST_FAIL_WRITING_BUF, _READING_BUF or _BUF_COMPARISON when,
//after that failure, the test region could not be put back either. Its bytes
//then hold test data, not what was there before the test.
#define EEPROM_TEST_FAIL_NOT_RESTORED 0x80

/* The bus, as the firmware provides it. Each returns 0 on success and anything
 * else on failure, a NACK included.
 *
 * write: one transfer to the 7-bit address devAddr - START, the address with
 *   the write bit, the len bytes of buf, STOP. buf[0] is the word address.
 * read: set the word address to wordAddr, then read len bytes from devAddr into
 *   outBuffer. The part's address counter runs on across pages and blocks, so
 *   one read may cross either.
 * mdelay: wait at least ms milliseconds. */
typedef int32_t (
    *cat24c16dev_write_ptr)(void *handle, uint8_t devAddr, uint8_t *buf, uint16_t len);
typedef int32_t (*cat24c16dev_read_ptr)(void *handle,
    uint8_t devAddr,
    uint8_t wordAddr,
    uint8_t *outBuffer,
    uint16_t len);
typedef void (*cat24c16dev_mdelay_ptr)(uint32_t ms);

typedef struct
{
  cat24c16dev_write_ptr write;
  cat24c16dev_read_ptr read;
  cat24c16dev_mdelay_ptr mdelay;
  //Passed to write and read unchanged, e.g. which I2C bus the part is on
  void *handle;
} cat24c16dev_ctx_t;

//Read length bytes from address. address + length must not pass 2048.
//Returns 0 on success, CAT24C16_OUT_OF_BOUNDS_ERROR, or the read callback's
//failure, in which case outBuffer is filled with 0xFF: what an erased part
//holds, so a failed read reads as unprogrammed (see ShimBrd_resetDaughterCardId).
int32_t CAT24C16_read(cat24c16dev_ctx_t *ctx, uint16_t address, uint16_t length, uint8_t *outBuffer);

//Write length bytes to address, one page per transfer. address + length must
//not pass 2048. It waits out each page's write cycle before going on, the last
//page's and a failed page's included, so it returns with the part idle: a read,
//another write or a power-off can follow at once. 5 ms a page, so 128 bytes on
//page boundaries take about 40 ms.
//Returns 0 on success, CAT24C16_OUT_OF_BOUNDS_ERROR, or the write callback's
//failure, with the pages before the failed one written.
int32_t CAT24C16_write(cat24c16dev_ctx_t *ctx, uint16_t address, uint16_t length, uint8_t *data);

//Write/read test over CAT24C16_TEST_SIZE bytes from CAT24C16_TEST_OFFSET. The
//bytes there are backed up first and, once anything has been written, put back
//and verified whatever the outcome (DEV-1494, DEV-1495).
//Returns EEPROM_TEST_PASS or an enum CAT24C16_TEST failure, with
//EEPROM_TEST_FAIL_NOT_RESTORED set if the region could not be put back.
uint8_t CAT24C16_test(cat24c16dev_ctx_t *ctx);

//For the factory report's CAT24C16 line. The first is why a CAT24C16_test()
//result failed, one of the SELF_TEST_STR_EEPROM_* strings, or "" for a pass and
//for a failure that is only the restore. The second is non-zero when the test
//region was left holding test data: bytes CAT24C16_TEST_OFFSET to
//CAT24C16_TEST_OFFSET + CAT24C16_TEST_SIZE - 1.
const char *CAT24C16_testFailReason(uint8_t testResult);
uint8_t CAT24C16_testLeftTestData(uint8_t testResult);

#endif /* EEPROM_CAT24C16_H_ */
