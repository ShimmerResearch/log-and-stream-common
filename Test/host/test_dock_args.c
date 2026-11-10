/*
 * Host-side test for Comms/shimmer_dock_args.c: the LEN each dock request must
 * carry before ShimDock_processCmd reads its arguments (DEV-1279).
 *
 * Built against the stub platform in Test/host/stubs - see stubs/README.md.
 * Run by .github/workflows/host-tests.yml.
 *
 * WHAT THIS IS FOR. INFOMEM, CARD_ID and CARD_MEM requests read their
 * arguments at fixed offsets in the receive buffer. Before this check a frame
 * shorter than its arguments had its own CRC, then whatever an earlier frame
 * left there, taken as length, offset and data: a short GET was answered with
 * data nobody asked for, and a short SET wrote stale bytes into the
 * configuration or the expansion-board EEPROM. Each case below is a LEN either
 * side of the one the request needs, and each fails with the check taken out.
 *
 * One form is accepted on purpose: GET CARD_ID with a 2-byte offset (LEN 5),
 * which pyshimmer sends before every InfoMem read.
 *
 * Built TWICE, once per platform: ENTER_BOOTLOADER exists only on Shimmer3R.
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions, so every .c file
 * under it is compiled into the firmware. This file defines main(). Only the
 * host-test build passes -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include <stdint.h>
#include <stdio.h>

#include "Comms/shimmer_dock_args.h"
#include "Comms/shimmer_dock_usart.h"
#include "host_test.h"

#if defined(SHIMMER3)
#define PLATFORM_NAME "Shimmer3"
#else
#define PLATFORM_NAME "Shimmer3R"
#endif

#define GET  UART_GET
#define SET  UART_SET
#define SHIM UART_COMP_SHIMMER
#define CARD UART_COMP_DAUGHTER_CARD

static uint8_t ok(uint8_t action, uint8_t comp, uint8_t prop, uint8_t len, uint8_t firstArg)
{
  return ShimDock_isArgLenOk(action, comp, prop, len, firstArg);
}

/* GET INFOMEM and GET CARD_MEM: a length byte and a 2-byte offset, LEN 5. */
static void test_get_with_a_2_byte_offset(void)
{
  testCase("test_get_with_a_2_byte_offset");
  expectU("GET INFOMEM, LEN 5", ok(GET, SHIM, UART_PROP_INFOMEM, 5, 0), 1);
  expectU("GET INFOMEM with no arguments, LEN 2",
      ok(GET, SHIM, UART_PROP_INFOMEM, 2, 0), 0);
  expectU("GET INFOMEM one byte short, LEN 4", ok(GET, SHIM, UART_PROP_INFOMEM, 4, 0), 0);
  expectU("GET INFOMEM one byte over, LEN 6", ok(GET, SHIM, UART_PROP_INFOMEM, 6, 0), 0);
  expectU("GET CARD_MEM, LEN 5", ok(GET, CARD, UART_PROP_CARD_MEM, 5, 0), 1);
  expectU("GET CARD_MEM with no arguments, LEN 2",
      ok(GET, CARD, UART_PROP_CARD_MEM, 2, 0), 0);
  expectU("GET CARD_MEM one byte short, LEN 4",
      ok(GET, CARD, UART_PROP_CARD_MEM, 4, 0), 0);
  expectU("GET CARD_MEM one byte over, LEN 6", ok(GET, CARD, UART_PROP_CARD_MEM, 6, 0), 0);
}

/* GET CARD_ID: a length byte and a 1-byte offset, LEN 4, and pyshimmer's
 * 2-byte offset, LEN 5. */
static void test_get_card_id(void)
{
  testCase("test_get_card_id");
  expectU("LEN 4", ok(GET, CARD, UART_PROP_CARD_ID, 4, 0), 1);
  expectU("LEN 5, as pyshimmer sends it", ok(GET, CARD, UART_PROP_CARD_ID, 5, 0), 1);
  expectU("with no arguments, LEN 2", ok(GET, CARD, UART_PROP_CARD_ID, 2, 0), 0);
  expectU("one byte short, LEN 3", ok(GET, CARD, UART_PROP_CARD_ID, 3, 0), 0);
  expectU("two bytes over, LEN 6", ok(GET, CARD, UART_PROP_CARD_ID, 6, 0), 0);
}

/* A SET carries its arguments, then as many data bytes as its length byte says. */
static void test_set_carries_its_data(void)
{
  testCase("test_set_carries_its_data");
  expectU("SET INFOMEM, 1 byte, LEN 6", ok(SET, SHIM, UART_PROP_INFOMEM, 6, 1), 1);
  expectU("SET INFOMEM, 128 bytes, LEN 133",
      ok(SET, SHIM, UART_PROP_INFOMEM, 133, 128), 1);
  expectU("SET INFOMEM, 0 bytes, LEN 5", ok(SET, SHIM, UART_PROP_INFOMEM, 5, 0), 1);
  expectU("SET INFOMEM with no arguments, LEN 2",
      ok(SET, SHIM, UART_PROP_INFOMEM, 2, 7), 0);
  expectU("SET INFOMEM with no offset, LEN 3", ok(SET, SHIM, UART_PROP_INFOMEM, 3, 1), 0);
  expectU("SET INFOMEM, 4 bytes said, 3 sent", ok(SET, SHIM, UART_PROP_INFOMEM, 8, 4), 0);
  expectU("SET INFOMEM, 4 bytes said, 5 sent", ok(SET, SHIM, UART_PROP_INFOMEM, 10, 4), 0);
  expectU("SET INFOMEM, 255 bytes said: the sum does not wrap",
      ok(SET, SHIM, UART_PROP_INFOMEM, 4, 255), 0);

  expectU("SET CARD_ID, 16 bytes, LEN 20", ok(SET, CARD, UART_PROP_CARD_ID, 20, 16), 1);
  expectU("SET CARD_ID with no arguments, LEN 2",
      ok(SET, CARD, UART_PROP_CARD_ID, 2, 0), 0);
  expectU("SET CARD_ID, 2 bytes said, 1 sent", ok(SET, CARD, UART_PROP_CARD_ID, 5, 2), 0);
  expectU("SET CARD_ID, 2 bytes said, 3 sent", ok(SET, CARD, UART_PROP_CARD_ID, 7, 2), 0);

  expectU("SET CARD_MEM, 128 bytes, LEN 133",
      ok(SET, CARD, UART_PROP_CARD_MEM, 133, 128), 1);
  expectU("SET CARD_MEM with no arguments, LEN 2",
      ok(SET, CARD, UART_PROP_CARD_MEM, 2, 0), 0);
  expectU("SET CARD_MEM, 2 bytes said, 1 sent",
      ok(SET, CARD, UART_PROP_CARD_MEM, 6, 2), 0);
  expectU("SET CARD_MEM, 2 bytes said, 3 sent",
      ok(SET, CARD, UART_PROP_CARD_MEM, 8, 2), 0);
}

/* A factory test takes no arguments; ENTER_BOOTLOADER takes its seconds. */
static void test_fixed_sets(void)
{
  testCase("test_fixed_sets");
  expectU("SET TEST, LEN 2", ok(SET, UART_COMP_TEST, 0, 2, 0), 1);
  expectU("SET TEST with an argument, LEN 3", ok(SET, UART_COMP_TEST, 0, 3, 0), 0);
#if defined(SHIMMER3R)
  expectU("SET ENTER_BOOTLOADER, LEN 3",
      ok(SET, SHIM, UART_PROP_ENTER_BOOTLOADER, 3, 5), 1);
  expectU("SET ENTER_BOOTLOADER with no seconds, LEN 2",
      ok(SET, SHIM, UART_PROP_ENTER_BOOTLOADER, 2, 5), 0);
  expectU("SET ENTER_BOOTLOADER one byte over, LEN 4",
      ok(SET, SHIM, UART_PROP_ENTER_BOOTLOADER, 4, 5), 0);
#endif
}

/* Anything else is the dispatcher's to judge: a fixed-length property checks
 * its own LEN there, and an unknown one gets BAD_CMD rather than BAD_ARG. */
static void test_others_pass_through(void)
{
  testCase("test_others_pass_through");
  expectU("GET VER, LEN 3", ok(GET, SHIM, UART_PROP_VER, 3, 0), 1);
  expectU("GET of an unknown component", ok(GET, 0x7F, 0x00, 9, 0), 1);
  expectU("SET of an unknown property", ok(SET, SHIM, 0x00, 9, 0), 1);
  expectU("an unknown command byte", ok(0x05, SHIM, UART_PROP_INFOMEM, 2, 0), 1);
}

int main(void)
{
  printf("shimmer_dock_args host tests (" PLATFORM_NAME ")\n\n");
  hostTestSilenceUnused();

  test_get_with_a_2_byte_offset();
  test_get_card_id();
  test_set_carries_its_data();
  test_fixed_sets();
  test_others_pass_through();

  return hostTestReport("shimmer_dock_args (" PLATFORM_NAME ")");
}

#endif /* SHIMMER_HOST_TEST */
