/*
 * shimmer_util.c
 *
 *  Created on: 4 Mar 2025
 *      Author: MarkNolan
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

void ShimUtil_ItoaWith0(uint64_t num, uint8_t *buf, uint8_t len)
{ //len = actual len + 1 extra '\0' at the end
  memset(buf, 0, len--);
  while (len--)
  {
    buf[len] = '0' + num % 10;
    num /= 10;
  }
}

void ShimUtil_ItoaNo0(uint64_t num, char *buf, uint8_t max_len)
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

volatile void *ShimUtil_memset_v(volatile void *dest, uint8_t value, size_t n)
{
  volatile uint8_t *dest_c = (volatile uint8_t *) dest;

  while (n > 0)
  {
    n--;
    dest_c[n] = value;
  }

  return dest;
}

//https://stackoverflow.com/questions/54964154/is-memcpyvoid-dest-src-n-with-a-volatile-array-safe
volatile void *ShimUtil_memcpy_v(volatile void *dest, const void *src, size_t n)
{
  const uint8_t *src_c = (const uint8_t *) src;
  volatile uint8_t *dest_c = (volatile uint8_t *) dest;

  while (n > 0)
  {
    n--;
    dest_c[n] = src_c[n];
  }

  return dest;
}

volatile void *ShimUtil_memcpy_vv(volatile void *dest, volatile void *src, size_t n)
{
  volatile uint8_t *src_c = (volatile uint8_t *) src;
  volatile uint8_t *dest_c = (volatile uint8_t *) dest;

  while (n > 0)
  {
    n--;
    dest_c[n] = src_c[n];
  }

  return dest;
}

size_t ShimUtil_strlen_v(volatile void *dest, size_t maxSize)
{
  volatile char *dest_c = (volatile char *) dest;

  uint16_t n;
  for (n = 0; n < maxSize; n++)
  {
    if (dest_c[n] == 0)
    {
      break;
    }
  }
  return n;

  ////TODO switch to more efficient approach based on stirng.h?
  //size_t      n = (size_t)-1;
  //const char *s = string;
  //
  //do n++; while (*s++);
  //return n;
}
