/*
 * Host-test replacement for the library's own log_and_stream_includes.h.
 *
 * THIS IS THE ONE FILE HERE THAT SHADOWS A REAL HEADER. -I Test/host/stubs is
 * searched before -I ., so a module under test picks this up instead of the
 * aggregate at the repository root. Nothing else in stubs/ shadows anything.
 *
 * WHY. The real aggregate includes every subsystem header, and three of them
 * (Comms/, SDCard/, SDSync/) include firmware-side headers that do not exist in
 * this repository - RN4678.h, ff.h, fx_api.h, shimmer_include.h, msp430.h. A
 * host compile of even a leaf module like Battery/ therefore fails on a header
 * it never uses. Cutting the aggregate is what makes the leaves reachable.
 *
 * WHAT THIS DOES NOT DO - and this is the point. It does not redefine a single
 * type, struct, bitfield or constant. Every subsystem header listed below is
 * the REAL one, resolved through -I . exactly as the firmware resolves it, so a
 * test sees the real gConfigBytes layout, the real BattStatus bitfields, the
 * real daughter_card_id_page and the real LED colour values. A stubbed struct
 * would drift from the firmware's and the tests would stay green while the
 * device was wrong.
 *
 * What it does stub is FUNCTION DECLARATIONS from the subsystems that are not
 * host-clean, defined as no-ops in host_stubs.c. Those are behaviour, not
 * layout: if one drifts, it shows up as a compile or link error here rather
 * than as a silently passing test.
 *
 * ADDING A SUBSYSTEM. When a subsystem's own header becomes host-clean, move it
 * from the stub block at the bottom up into the include list and delete its
 * no-op. That is the whole cost of bringing a new module under host test.
 */
#ifndef HOST_TEST_LOG_AND_STREAM_INCLUDES_H_
#define HOST_TEST_LOG_AND_STREAM_INCLUDES_H_

/* The real aggregate pulls <string.h> and <stdio.h> in transitively, through
 * SDCard/ and Comms/. Several subsystems lean on that rather than including
 * them themselves - Battery/shimmer_battery.c calls memset() with no <string.h>
 * of its own, for one - so the shadow has to offer them too. Without this the
 * host build fails on modules that compile perfectly well for both targets. */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ---- compiler intrinsics the two toolchains provide and a host does not --- *
 * log_and_stream_definitions.h maps __NOP() onto the TI MSP430 intrinsic
 * __no_operation() under SHIMMER3, and Button/shimmer_button.c uses __NOP() in
 * its debounce path. On Shimmer3R the same name is a CMSIS intrinsic, stubbed
 * in stubs/shimmer_include.h. Neither exists on a PC. An empty statement is the
 * right stand-in: the host tests assert state transitions, never timing.
 * -------------------------------------------------------------------------- */
#if defined(SHIMMER3)
#ifndef __no_operation
#define __no_operation() \
  do                     \
  {                      \
  } while (0)
#endif
#endif

/* ---- real subsystem headers, unmodified ---------------------------------- */
#include "Battery/shimmer_battery.h"
#include "Boards/shimmer_boards.h"
#include "CRC/shimmer_crc.h"
#include "CRC/shimmer_swCrc.h"
#include "Configuration/shimmer_config.h"
#include "EEPROM/shimmer_eeprom.h"
#include "LEDs/shimmer_leds.h"
#include "Platform/platform_api.h"
#include "RTC/shimmer_rtc.h"
#include "Sensing/shimmer_packet_ring.h"
#include "Util/shimmer_util.h"
#include "log_and_stream_common.h"
#include "log_and_stream_definitions.h"

/* The real externs contract. It is host-clean once the HAL headers it includes
 * are stubbed, and it is where shimmerStatus and batteryStatus are declared.
 * Several subsystems (Button/, LEDs/) read those two without including this
 * themselves, relying on the firmware aggregate to have pulled it in. */
#include "log_and_stream_externs.h"

/* ---- stubbed declarations ------------------------------------------------ *
 * From subsystems whose headers are not host-clean. Signatures must match the
 * real ones; host_stubs.c defines them. Tests that need to steer one of these
 * use the hostStub_* setters in host_stubs.h.
 * -------------------------------------------------------------------------- */

/* TaskList/shimmer_taskList.h - needs firmware-side shimmer_definitions.h */
void ShimTask_setStopSensing(void);
void ShimTask_setStopLogging(void);
void ShimTask_setStopStreaming(void);
void ShimTask_setStartLoggingIfReady(void);
void ShimTask_setStartStreamingIfReady(void);

/* Sensing/shimmer_sensing.h - needs firmware-side shimmer_include.h */
uint8_t ShimSens_isSamplingRateInvalid(void);
void ShimSens_gatherData(void);

#endif /* HOST_TEST_LOG_AND_STREAM_INCLUDES_H_ */
