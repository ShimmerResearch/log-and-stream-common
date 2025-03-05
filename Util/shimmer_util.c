/*
 * shimmer_util.c
 *
 *  Created on: 4 Mar 2025
 *      Author: MarkNolan
 */

#include <stdint.h>

void ShimUtil_ItoaWith0(uint64_t num, uint8_t *buf, uint8_t len)
{ //len = actual len + 1 extra '\0' at the end
  memset(buf, 0, len--);
  while (len--)
  {
    buf[len] = '0' + num % 10;
    num /= 10;
  }
}

void ShimUtil_ItoaNo0(uint64_t num, uint8_t *buf, uint8_t max_len)
{ //len = actual len + 1 extra '\0' at the end
  uint8_t idx, i_move;
  memset(buf, 0, max_len);
  if (!num)
  {
    buf[0] = '0';
  }
  for (idx = 0; (idx < max_len - 1) && (num > 0); idx++)
  {
    for (i_move = idx; i_move > 0; i_move--)
    {
      buf[i_move] = buf[i_move - 1];
    }
    buf[0] = '0' + num % 10;
    num /= 10;
  }
}
