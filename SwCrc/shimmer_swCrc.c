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
 * @date May, 2016
 */

#include "shimmer_swCrc.h"

uint16_t ShimSwCrc_byte(uint16_t crc, uint8_t b)
{
  crc = (uint8_t) (crc >> 8) | (crc << 8);
  crc ^= b;
  crc ^= (uint8_t) (crc & 0xff) >> 4;
  crc ^= crc << 12;
  crc ^= (crc & 0xff) << 5;
  return crc;
}

uint16_t ShimSwCrc_calc(uint8_t *msg, uint8_t len)
{
  uint16_t crcCalc;
  uint8_t i;

  crcCalc = ShimSwCrc_byte(CRC_INIT, *msg);
  for (i = 1; i < len; i++)
  {
    crcCalc = ShimSwCrc_byte(crcCalc, *(msg + i));
  }
  if (len % 2)
  {
    crcCalc = ShimSwCrc_byte(crcCalc, 0x00);
  }
  return crcCalc;
}

uint8_t ShimSwCrc_check(uint8_t *msg, uint8_t len)
{
  uint16_t crc;

  crc = ShimSwCrc_calc(msg, len - 2);

  if (((crc & 0xFF) == msg[len - 2]) && (((crc & 0xFF00) >> 8) == msg[len - 1]))
  {
    return 1; //TRUE
  }
  else
  {
    return 0; //FALSE
  }
}
