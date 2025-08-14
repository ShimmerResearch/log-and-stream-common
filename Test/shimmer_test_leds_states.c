/*
 * shimmer_test_leds_states.c
 *
 *  Created on: Aug 14, 2025
 *      Author: MarkNolan
 */

#include "shimmer_test_leds_states.h"

#include <stdint.h>

#include "log_and_stream_includes.h"

void ShimLeds_testOperationalStatesBtDisabled(void);
void ShimLeds_testOperationalStatesBtEnabled(void);
void ShimLeds_testOperationalStatesSdSyncEnabled(void);
void ShimLeds_testOperationalStatesOther(void);

/**
 * @brief Test function to simulate different states of the LEDs based on the
 * shimmerStatus. This function is intended for testing purposes.
 */
void ShimLeds_testOperationalStates(void)
{
  //Backup current shimmerStatus
  STATTypeDef shimmerStatusBckup = shimmerStatus;

  send_test_report("Testing Operational LED states - Start\r\n");

  ShimLeds_testOperationalStatesBtDisabled();
  ShimLeds_testOperationalStatesBtEnabled();
  ShimLeds_testOperationalStatesSdSyncEnabled();
  ShimLeds_testOperationalStatesOther();

  //Restore original shimmerStatus
  shimmerStatus = shimmerStatusBckup;

  send_test_report("Testing Operational LED states - End\r\n");
}

void ShimLeds_testOperationalStatesBtDisabled(void)
{
  send_test_report("BT Disabled:\r\n");
  send_test_report("\t-> Idle...\r\n");
  shimmerStatus.sensing = 0;
  shimmerStatus.btPowerOn = 0;
  shimmerStatus.btConnected = 0;
  shimmerStatus.btInSyncMode = 0;
  shimmerStatus.btIsInitialised = 0;
  shimmerStatus.btSupportEnabled = 0;
  shimmerStatus.btstreamReady = 0;
  shimmerStatus.btStreaming = 0;
  shimmerStatus.sdSyncEnabled = 0;
  shimmerStatus.sdlogReady = 1;
  shimmerStatus.sdLogging = 0;
  delay_ms(SHIMMER_LEDS_TEST_INTERVAL_MS);

  send_test_report("\t-> SD Logging...\r\n");
  shimmerStatus.sensing = 1;
  shimmerStatus.sdLogging = 1;
  delay_ms(SHIMMER_LEDS_TEST_INTERVAL_MS);
}

void ShimLeds_testOperationalStatesBtEnabled(void)
{
  send_test_report("BT Enabled:\r\n");
  send_test_report("\t-> Idle...\r\n");
  shimmerStatus.sensing = 0;
  shimmerStatus.btPowerOn = 1;
  shimmerStatus.btConnected = 0;
  shimmerStatus.btInSyncMode = 0;
  shimmerStatus.btIsInitialised = 1;
  shimmerStatus.btSupportEnabled = 1;
  shimmerStatus.btstreamReady = 1;
  shimmerStatus.btStreaming = 0;
  shimmerStatus.sdSyncEnabled = 0;
  shimmerStatus.sdlogReady = 1;
  shimmerStatus.sdLogging = 0;
  delay_ms(SHIMMER_LEDS_TEST_INTERVAL_MS);

  send_test_report("\t-> SD Logging...\r\n");
  shimmerStatus.sensing = 1;
  shimmerStatus.sdLogging = 1;
  shimmerStatus.btStreaming = 0;
  shimmerStatus.btConnected = 0;
  delay_ms(SHIMMER_LEDS_TEST_INTERVAL_MS);

  send_test_report("\t-> BT Streaming...\r\n");
  shimmerStatus.sensing = 1;
  shimmerStatus.sdLogging = 0;
  shimmerStatus.btStreaming = 1;
  shimmerStatus.btConnected = 1;
  delay_ms(SHIMMER_LEDS_TEST_INTERVAL_MS);

  send_test_report("\t-> BT Streaming and SD Logging...\r\n");
  shimmerStatus.sensing = 1;
  shimmerStatus.sdLogging = 1;
  shimmerStatus.btStreaming = 1;
  shimmerStatus.btConnected = 1;
  delay_ms(SHIMMER_LEDS_TEST_INTERVAL_MS);

  send_test_report("\t-> BT Connected...\r\n");
  shimmerStatus.sensing = 0;
  shimmerStatus.sdLogging = 0;
  shimmerStatus.btStreaming = 0;
  shimmerStatus.btConnected = 1;
  delay_ms(SHIMMER_LEDS_TEST_INTERVAL_MS);

  send_test_report("\t-> BT Connected and SD Logging...\r\n");
  shimmerStatus.sensing = 1;
  shimmerStatus.sdLogging = 1;
  shimmerStatus.btStreaming = 0;
  shimmerStatus.btConnected = 1;
  delay_ms(SHIMMER_LEDS_TEST_INTERVAL_MS);
}

void ShimLeds_testOperationalStatesSdSyncEnabled(void)
{
  send_test_report("SD Sync Enabled:\r\n");
  uint8_t trialConfig0Bckup = ShimSdHead_sdHeadTextGetByte(SDH_TRIAL_CONFIG0);
  send_test_report("\t-> Idle...\r\n");
  shimmerStatus.sensing = 0;
  shimmerStatus.btPowerOn = 0;
  shimmerStatus.btConnected = 0;
  shimmerStatus.btInSyncMode = 0;
  shimmerStatus.btIsInitialised = 0;
  shimmerStatus.btSupportEnabled = 0;
  shimmerStatus.btstreamReady = 0;
  shimmerStatus.btStreaming = 0;
  shimmerStatus.sdSyncEnabled = 1;
  shimmerStatus.sdlogReady = 1;
  shimmerStatus.sdLogging = 0;
  ShimSdSync_rcFirstOffsetRxedSet(0);
  delay_ms(SHIMMER_LEDS_TEST_INTERVAL_MS);

  send_test_report("\t-> SD Logging waiting for initial sync (slave)...\r\n");
  shimmerStatus.sensing = 1;
  shimmerStatus.sdLogging = 1;
  shimmerStatus.btPowerOn = 1;
  shimmerStatus.btIsInitialised = 1;
  shimmerStatus.btConnected = 0;
  shimmerStatus.btInSyncMode = 1;
  ShimSdSync_rcFirstOffsetRxedSet(0);
  ShimSdHead_sdHeadTextSetByte(SDH_TRIAL_CONFIG0, trialConfig0Bckup &= !SDH_IAMMASTER);
  delay_ms(SHIMMER_LEDS_TEST_INTERVAL_MS);

  send_test_report("\t-> SD Logging waiting for initial sync (master)...\r\n");
  shimmerStatus.sensing = 1;
  shimmerStatus.sdLogging = 1;
  shimmerStatus.btPowerOn = 1;
  shimmerStatus.btIsInitialised = 1;
  shimmerStatus.btConnected = 0;
  shimmerStatus.btInSyncMode = 1;
  ShimSdSync_rcFirstOffsetRxedSet(0);
  ShimSdHead_sdHeadTextSetByte(SDH_TRIAL_CONFIG0, trialConfig0Bckup |= SDH_IAMMASTER);
  delay_ms(SHIMMER_LEDS_TEST_INTERVAL_MS);

  send_test_report("\t-> SD Logging and BT advertising...\r\n");
  shimmerStatus.sensing = 1;
  shimmerStatus.sdLogging = 1;
  shimmerStatus.btPowerOn = 1;
  shimmerStatus.btIsInitialised = 1;
  shimmerStatus.btConnected = 0;
  shimmerStatus.btInSyncMode = 1;
  ShimSdSync_rcFirstOffsetRxedSet(1);
  delay_ms(SHIMMER_LEDS_TEST_INTERVAL_MS);

  send_test_report("\t-> SD Logging and syncing...\r\n");
  shimmerStatus.sensing = 1;
  shimmerStatus.sdLogging = 1;
  shimmerStatus.btPowerOn = 1;
  shimmerStatus.btIsInitialised = 1;
  shimmerStatus.btConnected = 1;
  shimmerStatus.btInSyncMode = 1;
  ShimSdSync_rcFirstOffsetRxedSet(0);
  delay_ms(SHIMMER_LEDS_TEST_INTERVAL_MS);

  ShimSdHead_sdHeadTextSetByte(SDH_TRIAL_CONFIG0, trialConfig0Bckup);
  ShimSdSync_rcFirstOffsetRxedSet(0);
}

void ShimLeds_testOperationalStatesOther(void)
{
  uint8_t rwcErrorFlashBckup = ShimLeds_getRtcErrorFlash();

  send_test_report("Other:\r\n");
  shimmerStatus.sensing = 0;
  shimmerStatus.btPowerOn = 0;
  shimmerStatus.btConnected = 0;
  shimmerStatus.btInSyncMode = 0;
  shimmerStatus.btIsInitialised = 0;
  shimmerStatus.btSupportEnabled = 0;
  shimmerStatus.btstreamReady = 0;
  shimmerStatus.btStreaming = 0;
  shimmerStatus.sdSyncEnabled = 0;
  shimmerStatus.sdlogReady = 0;
  shimmerStatus.sdLogging = 0;

  send_test_report("\t-> Configuring...\r\n");
  shimmerStatus.configuring = 1;
  delay_ms(SHIMMER_LEDS_TEST_INTERVAL_MS);
  shimmerStatus.configuring = 0;

  send_test_report("\t-> Time not set...\r\n");
  ShimLeds_setRtcErrorFlash(1);
  delay_ms(SHIMMER_LEDS_TEST_INTERVAL_MS);
  ShimLeds_setRtcErrorFlash(rwcErrorFlashBckup);
}
