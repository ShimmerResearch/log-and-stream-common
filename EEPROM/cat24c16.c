/*
 * Copyright (c) 2016, Shimmer Research, Ltd.
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *    * Neither the name of Shimmer Research, Ltd. nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *    * You may not use or distribute this Software or any derivative works
 *      in any form for commercial purposes with the exception of commercial
 *      purposes when used in conjunction with Shimmer products purchased
 *      from Shimmer or their designated agent or with permission from
 *      Shimmer.
 *      Examples of commercial purposes would be running business
 *      operations, licensing, leasing, or selling the Software, or
 *      distributing the Software for use with commercial products.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Weibo Pan
 * @date November, 2013 (Shimmer3), May, 2016 (Shimmer3R)
 * @author Mark Nolan
 * @date June, 2024; November, 2026 (one driver for both, DEV-1496)
 */

#include "EEPROM/cat24c16.h"

#include <string.h>

#include "Test/shimmer_test.h"

//Rejects address + length past the end without computing address + length,
//which wraps in the MSP430's 16-bit arithmetic for an address near 65535
static uint8_t CAT24C16_isOutOfBounds(uint16_t address, uint16_t length)
{
  return (length == 0) || (address >= CAT24C16_TOTAL_SIZE)
      || (length > (uint16_t) (CAT24C16_TOTAL_SIZE - address));
}

int32_t CAT24C16_read(cat24c16dev_ctx_t *ctx, uint16_t address, uint16_t length, uint8_t *outBuffer)
{
  int32_t result;

  if (CAT24C16_isOutOfBounds(address, length))
  {
    return CAT24C16_OUT_OF_BOUNDS_ERROR;
  }

  result = ctx->read(ctx->handle, (uint8_t) (CAT24C16_ADDR | (address >> 8)),
      (uint8_t) (address & 0xFF), outBuffer, length);
  if (result)
  {
    memset(outBuffer, 0xFF, length);
  }
  return result;
}

int32_t CAT24C16_write(cat24c16dev_ctx_t *ctx, uint16_t address, uint16_t length, uint8_t *data)
{
  uint8_t buf[CAT24C16_PAGE_SIZE + 1]; //word address, then one page at most
  uint16_t done = 0;
  uint16_t thisWrite;
  uint16_t memPtr;
  int32_t result = 0;

  if (CAT24C16_isOutOfBounds(address, length))
  {
    return CAT24C16_OUT_OF_BOUNDS_ERROR;
  }

  while (done < length)
  {
    memPtr = address + done;

    //A write may not cross a page: the part wraps within the page instead
    thisWrite = CAT24C16_PAGE_SIZE - (memPtr % CAT24C16_PAGE_SIZE);
    if (thisWrite > length - done)
    {
      thisWrite = length - done;
    }

    buf[0] = (uint8_t) (memPtr & 0xFF);
    memcpy(&buf[1], &data[done], thisWrite);

    result = ctx->write(ctx->handle, (uint8_t) (CAT24C16_ADDR | (memPtr >> 8)),
        buf, (uint16_t) (thisWrite + 1));

    //Wait out the write cycle even when the transfer reported an error: data
    //that reached the part before the error is written at the STOP, and until
    //that cycle ends the part acknowledges nothing. Waiting here, after every
    //page, means a caller never meets a cycle still running, whether it reads
    //next, writes again or powers the part off.
    ctx->mdelay(CAT24C16_WRITE_CYCLE_MS);

    if (result)
    {
      break;
    }
    done += thisWrite;
  }

  return result;
}

//Write the backed-up bytes back over the test region and read them back.
//Returns 0 once the region holds the backup again, 1 if every try failed.
static uint8_t CAT24C16_restoreTestRegion(cat24c16dev_ctx_t *ctx, uint8_t *backup, uint8_t *readBack)
{
  uint8_t tries;

  for (tries = 0; tries < CAT24C16_TEST_RESTORE_TRIES; tries++)
  {
    //CAT24C16_write() returns with the part idle, however the test before it
    //ended, so the read-back and the next try need no wait of their own
    if (CAT24C16_write(ctx, CAT24C16_TEST_OFFSET, CAT24C16_TEST_SIZE, backup) == 0
        && CAT24C16_read(ctx, CAT24C16_TEST_OFFSET, CAT24C16_TEST_SIZE, readBack) == 0
        && memcmp(backup, readBack, CAT24C16_TEST_SIZE) == 0)
    {
      return 0;
    }
  }

  return 1;
}

uint8_t CAT24C16_test(cat24c16dev_ctx_t *ctx)
{
  uint8_t testResult = EEPROM_TEST_PASS;
  uint8_t pass;
  uint16_t i;
  uint8_t test_eeprom_backup[CAT24C16_TEST_SIZE];
  uint8_t test_eeprom_wr[CAT24C16_TEST_SIZE];
  uint8_t test_eeprom_rd[CAT24C16_TEST_SIZE];

  if (CAT24C16_read(ctx, CAT24C16_TEST_OFFSET, CAT24C16_TEST_SIZE, test_eeprom_backup))
  {
    //Nothing has been written yet, so there is nothing to put back
    return EEPROM_TEST_FAIL_INITIAL_BACKUP;
  }

  for (pass = 1; pass <= 3 && testResult == EEPROM_TEST_PASS; pass++)
  {
    //An incrementing pattern, offset by the pass number, so every byte differs
    //from the previous pass's: a pass whose write never happened cannot read
    //back correctly
    for (i = 0; i < CAT24C16_TEST_SIZE; i++)
    {
      test_eeprom_wr[i] = (uint8_t) (pass + i);
    }
    memset(test_eeprom_rd, 0, CAT24C16_TEST_SIZE);

    if (CAT24C16_write(ctx, CAT24C16_TEST_OFFSET, CAT24C16_TEST_SIZE, test_eeprom_wr))
    {
      testResult = EEPROM_TEST_FAIL_WRITING_BUF;
    }
    else if (CAT24C16_read(ctx, CAT24C16_TEST_OFFSET, CAT24C16_TEST_SIZE, test_eeprom_rd))
    {
      testResult = EEPROM_TEST_FAIL_READING_BUF;
    }
    else if (memcmp(test_eeprom_wr, test_eeprom_rd, CAT24C16_TEST_SIZE))
    {
      testResult = EEPROM_TEST_FAIL_BUF_COMPARISON;
    }
  }

  //From the first write on, the test region may hold test data whatever
  //happened after it, so every outcome puts the backup back (DEV-1494,
  //DEV-1495). Those bytes are host offsets 0-127 of the daughter-card memory.
  if (CAT24C16_restoreTestRegion(ctx, test_eeprom_backup, test_eeprom_rd))
  {
    testResult = (testResult == EEPROM_TEST_PASS) ?
        EEPROM_TEST_FAIL_RESTORING_BACKUP :
        (uint8_t) (testResult | EEPROM_TEST_FAIL_NOT_RESTORED);
  }

  return testResult;
}

const char *CAT24C16_testFailReason(uint8_t testResult)
{
  switch (testResult & (uint8_t) ~EEPROM_TEST_FAIL_NOT_RESTORED)
  {
    case EEPROM_TEST_PASS:
    case EEPROM_TEST_FAIL_RESTORING_BACKUP:
      return SELF_TEST_STR_EMPTY;
    case EEPROM_TEST_FAIL_INITIAL_BACKUP:
      return SELF_TEST_STR_EEPROM_READ;
    case EEPROM_TEST_FAIL_WRITING_BUF:
      return SELF_TEST_STR_EEPROM_WRITE;
    case EEPROM_TEST_FAIL_READING_BUF:
      return SELF_TEST_STR_EEPROM_READ_BACK;
    case EEPROM_TEST_FAIL_BUF_COMPARISON:
      return SELF_TEST_STR_EEPROM_MISMATCH;
    default:
      return SELF_TEST_STR_UNKNOWN;
  }
}

uint8_t CAT24C16_testLeftTestData(uint8_t testResult)
{
  return (testResult == EEPROM_TEST_FAIL_RESTORING_BACKUP)
      || (testResult & EEPROM_TEST_FAIL_NOT_RESTORED);
}
