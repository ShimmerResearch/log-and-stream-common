/*
 * shimmer_util.c
 *
 *  Created on: 4 Mar 2025
 *      Author: MarkNolan
 */

#include <stdint.h>
#include <stdio.h>
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

/* Writes a non-negative value as a decimal string for atof() to read back,
 * e.g. the sdlog.cfg sample rate: "51.2", "50.027481", "1024".
 *
 * The fraction is zero-padded to its width, which is what keeps 50.0275 from
 * being written as "50.3". The width is the fewest digits, up to six, that
 * atof() reads back as the same float, so 51.2 is "51.2" rather than the
 * float's exact 51.200001. Six is what lets every sampling divider round-trip:
 * two are enough only for dividers up to 1824 (rates down to about 18 Hz).
 *
 * No %f, because Shimmer3 builds with --printf_support=nofloat, and the
 * integer part is printed as unsigned long because the MSP430's int is 16
 * bits, so %d would print 32768 Hz as -32768. */
void ShimUtil_FtoaDecimal(float value, char *buf, uint8_t max_len)
{
  uint32_t intPart, fracPart, scale;
  uint8_t digits, i;
  size_t len = 0;

  for (digits = 0, scale = 1; digits <= 6; digits++, scale *= 10)
  {
    intPart = (uint32_t) value;
    fracPart = (uint32_t) ((value - (float) intPart) * (float) scale + 0.5f);
    if (fracPart >= scale)
    {
      fracPart -= scale;
      intPart++;
    }

    snprintf(buf, max_len, "%lu", (unsigned long) intPart);
    len = strlen(buf);
    if (digits > 0 && len + 1 + digits < max_len)
    {
      buf[len] = '.';
      for (i = digits; i > 0; i--)
      {
        buf[len + i] = '0' + fracPart % 10;
        fracPart /= 10;
      }
      len += 1 + digits;
      buf[len] = '\0';
    }

    if ((float) atof(buf) == value)
    {
      return;
    }
  }

  /* Six digits that still differ from the float - a value carried up to the
   * next whole number, say. Drop the trailing zeros and any bare point. */
  if (strchr(buf, '.') != NULL)
  {
    while (len > 0 && buf[len - 1] == '0')
    {
      buf[--len] = '\0';
    }
    if (len > 0 && buf[len - 1] == '.')
    {
      buf[--len] = '\0';
    }
  }
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

void ShimUtil_reverseArray(uint8_t *arr, size_t length)
{
  size_t i;
  for (i = 0; i < length / 2; i++)
  {
    uint8_t temp = arr[i];
    arr[i] = arr[length - 1 - i];
    arr[length - 1 - i] = temp;
  }
}
