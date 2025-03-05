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

uint64_t ShimUtil_Atol64(uint8_t *buf)
{
  uint8_t temp_str_1[6], temp_str_2[6], temp_str_3[6], temp_str_4[6];
  uint32_t temp_int_1, temp_int_2, temp_int_3, temp_int_4;
  uint64_t ret_val;

  memcpy(temp_str_1, buf, 5);
  memcpy(temp_str_2, buf + 5, 5);
  memcpy(temp_str_3, buf + 10, 5);
  memcpy(temp_str_4, buf + 15, 5);

  temp_str_1[5] = 0;
  temp_str_2[5] = 0;
  temp_str_3[5] = 0;
  temp_str_4[5] = 0;

  temp_int_1 = atol((char *) temp_str_1);
  temp_int_2 = atol((char *) temp_str_2);
  temp_int_3 = atol((char *) temp_str_3);
  temp_int_4 = atol((char *) temp_str_4);

  ret_val = temp_int_1;
  ret_val *= 100000;
  ret_val += temp_int_2;
  ret_val *= 100000;
  ret_val += temp_int_3;
  ret_val *= 100000;
  ret_val += temp_int_4;

  return ret_val;
}
