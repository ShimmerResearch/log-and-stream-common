/*
 * shimmer_test.c
 *
 *  Created on: Aug 14, 2025
 *      Author: MarkNolan
 */

#include "shimmer_test.h"

#include <inttypes.h>
#include <stdint.h>

#include "log_and_stream_includes.h"

factory_test_target_t factoryTestTarget = PRINT_TO_DEBUGGER;
factory_test_t factoryTestToRun;

char outputBuffer[100];

uint32_t run_factory_test(void)
{
  send_test_report("//**************************** TEST START "
                   "************************************//\r\n");

  shimmerStatus.testResult = 0;

  if (FACTORY_TEST_LED_STATES)
  {
    ShimLeds_testOperationalStates();
  }
  else
  {
    hal_run_factory_test(factoryTestToRun, &outputBuffer[0]);
  }

  if (factoryTestToRun == FACTORY_TEST_MAIN || factoryTestToRun == FACTORY_TEST_ICS)
  {
    if (shimmerStatus.testResult)
    {
      sprintf(outputBuffer, "\r\nOverall Result = FAIL (0x%08" PRIX32 ")\r\n",
          shimmerStatus.testResult);
    }
    else
    {
      sprintf(outputBuffer, "\r\nOverall Result = PASS\r\n");
    }
    send_test_report(outputBuffer);
  }

  send_test_report("//***************************** TEST END "
                   "*************************************//\r\n");

  return shimmerStatus.testResult;
}

void setup_factory_test(factory_test_target_t target, factory_test_t testToRun)
{
  factoryTestTarget = target;
  factoryTestToRun = testToRun;
}

void send_test_report(const char *str)
{
  if (str == NULL || strlen(str) == 0)
  {
    return;
  }

  //If the string is too long, truncate it
  if (strlen(str) > MAX_TEST_REPORT_LENGTH)
  {
    char truncatedStr[MAX_TEST_REPORT_LENGTH + 1];
    strncpy(truncatedStr, str, MAX_TEST_REPORT_LENGTH);
    truncatedStr[MAX_TEST_REPORT_LENGTH] = '\0';
    send_test_report_impl(truncatedStr, factoryTestTarget);
  }
  else
  {
    send_test_report_impl(str, factoryTestTarget);
  }
}

__weak void send_test_report_impl(const char *str, factory_test_target_t factoryTestTarget)
{
  //This function can be overridden by the main application to send a test
  //report. The default implementation does nothing.
  (void) str; //Suppress unused parameter warning
}
