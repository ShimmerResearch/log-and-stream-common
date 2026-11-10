/*
 * shimmer_dock_args.c
 *
 * The LEN check behind ShimDock_processCmd. Kept apart from the dock module so
 * the host tests can build it on its own: it depends on nothing but the
 * protocol's constants.
 */

#include "shimmer_dock_args.h"

#include "shimmer_dock_usart.h"

/* The arguments of each request (SHIMMER3_DOCK_PROTOCOL.md Appendix A). */
#define LEN_HEAD        2U /* component and property */
#define ARGS_INFOMEM    3U /* length, offset (2 bytes) */
#define ARGS_CARD_ID    2U /* length, offset (1 byte) */
#define ARGS_CARD_MEM   3U /* length, offset (2 bytes) */
#define ARGS_BOOTLOADER 1U /* seconds */

/* A SET with `args` bytes of length and offset, then that many data bytes. */
static uint8_t setLenOk(uint8_t len, uint8_t args, uint8_t dataLen)
{
  return (len >= LEN_HEAD + args) && ((uint16_t) len == LEN_HEAD + args + dataLen);
}

uint8_t ShimDock_isArgLenOk(uint8_t action, uint8_t comp, uint8_t prop, uint8_t len, uint8_t firstArg)
{
  if (action == UART_GET)
  {
    if (comp == UART_COMP_SHIMMER && prop == UART_PROP_INFOMEM)
    {
      return len == LEN_HEAD + ARGS_INFOMEM;
    }
    if (comp == UART_COMP_DAUGHTER_CARD && prop == UART_PROP_CARD_ID)
    {
      /* Also one byte over: pyshimmer (ShimmerDock.get_infomem) sends this
       * GET before every InfoMem read with a 2-byte offset, "<BH" 0, 0. Its
       * high byte is ignored, as it always was. Refusing it would break every
       * released pyshimmer's InfoMem read. */
      return (len == LEN_HEAD + ARGS_CARD_ID) || (len == LEN_HEAD + ARGS_CARD_ID + 1U);
    }
    if (comp == UART_COMP_DAUGHTER_CARD && prop == UART_PROP_CARD_MEM)
    {
      return len == LEN_HEAD + ARGS_CARD_MEM;
    }
  }
  else if (action == UART_SET)
  {
    if (comp == UART_COMP_SHIMMER && prop == UART_PROP_INFOMEM)
    {
      return setLenOk(len, ARGS_INFOMEM, firstArg);
    }
#if defined(SHIMMER3R)
    if (comp == UART_COMP_SHIMMER && prop == UART_PROP_ENTER_BOOTLOADER)
    {
      return len == LEN_HEAD + ARGS_BOOTLOADER;
    }
#endif
    if (comp == UART_COMP_DAUGHTER_CARD && prop == UART_PROP_CARD_ID)
    {
      return setLenOk(len, ARGS_CARD_ID, firstArg);
    }
    if (comp == UART_COMP_DAUGHTER_CARD && prop == UART_PROP_CARD_MEM)
    {
      return setLenOk(len, ARGS_CARD_MEM, firstArg);
    }
    if (comp == UART_COMP_TEST)
    {
      /* The suite is the property; a factory test takes no arguments. */
      return len == LEN_HEAD;
    }
  }
  return 1U;
}
