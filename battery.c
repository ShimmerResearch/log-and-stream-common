/*
 * battery.c
 *
 *  Created on: Jan 28, 2025
 *      Author: MarkNolan
 */

#include "battery.h"
#include "log_and_stream_externs.h"

#include "hal_Board.h"

uint8_t battCriticalCount = 0;
battAlarmInterval_t battAlarmInterval;

void updateBatteryStatus(uint16_t adc_battVal, uint16_t battValMV)
{
  batteryStatus.battStatusRaw.adcBattVal = adc_battVal;
  batteryStatus.battStatusRaw.rawBytes[2] = 0;
  batteryStatus.battStatusRaw.STAT2 = LM3658SD_STAT2;
  batteryStatus.battStatusRaw.STAT1 = LM3658SD_STAT1;
  batteryStatus.battValMV = battValMV;

  rankBatt();
  rankBattChargingStatus();
}

void rankBatt(void)
{
  if (batteryStatus.battStat == BATT_MID)
  {
    if (batteryStatus.battStatusRaw.adcBattVal < BATT_MID_MIN)
    {
      batteryStatus.battStat = BATT_LOW;
    }
    else if (batteryStatus.battStatusRaw.adcBattVal < BATT_MID_MAX)
    {
      batteryStatus.battStat = BATT_MID;
    }
    else
    {
      batteryStatus.battStat = BATT_HIGH;
    }
  }
  else if (batteryStatus.battStat == BATT_LOW)
  {
    if (batteryStatus.battStatusRaw.adcBattVal < BATT_LOW_MAX)
    {
      batteryStatus.battStat = BATT_LOW;
    }
    else if (batteryStatus.battStatusRaw.adcBattVal < BATT_MID_MAX)
    {
      batteryStatus.battStat = BATT_MID;
    }
    else
    {
      batteryStatus.battStat = BATT_HIGH;
    }
  }
  else
  { //high
    if (batteryStatus.battStatusRaw.adcBattVal < BATT_MID_MIN)
    {
      batteryStatus.battStat = BATT_LOW;
    }
    else if (batteryStatus.battStatusRaw.adcBattVal < BATT_HIGH_MIN)
    {
      batteryStatus.battStat = BATT_MID;
    }
    else
    {
      batteryStatus.battStat = BATT_HIGH;
    }
  }

  switch (batteryStatus.battStat)
  {
  case BATT_LOW:
#if defined(SHIMMER3)
    batteryStatus.battStatLed = LED_RED;
#elif defined(SHIMMER3R)
    batteryStatus.battStatLed = LED_RGB_RED;
#endif
    break;
  case BATT_MID:
#if defined(SHIMMER3)
    batteryStatus.battStatLed = LED_YELLOW;
#elif defined(SHIMMER3R)
    batteryStatus.battStatLed = LED_RGB_YELLOW;
#endif
    break;
  case BATT_HIGH:
#if defined(SHIMMER3)
    batteryStatus.battStatLed = LED_GREEN0;
#elif defined(SHIMMER3R)
    batteryStatus.battStatLed = LED_RGB_GREEN;
#endif
    break;
  default:
#if defined(SHIMMER3)
    batteryStatus.battStatLed = LED_RED;
#elif defined(SHIMMER3R)
    batteryStatus.battStatLed = LED_RGB_RED;
#endif
    break;
  }
}

void rankBattChargingStatus(void)
{
  if (batteryStatus.battValMV > BATTERY_ERROR_VOLTAGE_MAX)
  {
    batteryStatus.battChargingStatus = CHARGING_STATUS_CHECKING;
  }
  else
  {
    switch (batteryStatus.battStatusRaw.rawBytes[2])
    {
    case CHRG_CHIP_STATUS_SUSPENDED:
      if (batteryStatus.battValMV <= BATTERY_ERROR_VOLTAGE_MIN)
      {
        batteryStatus.battChargingStatus = CHARGING_STATUS_BAD_BATTERY;
      }
      else
      {
        batteryStatus.battChargingStatus = CHARGING_STATUS_SUSPENDED;
      }
      break;
    case CHRG_CHIP_STATUS_FULLY_CHARGED:
      batteryStatus.battChargingStatus = CHARGING_STATUS_FULLY_CHARGED;
      break;
    case CHRG_CHIP_STATUS_PRECONDITIONING:
      batteryStatus.battChargingStatus = CHARGING_STATUS_CHARGING;
      break;
    case CHRG_CHIP_STATUS_BAD_BATTERY:
      batteryStatus.battChargingStatus = CHARGING_STATUS_BAD_BATTERY;
      break;
    case CHRG_CHIP_STATUS_UNKNOWN:
      batteryStatus.battChargingStatus = CHARGING_STATUS_UNKNOWN;
      break;
    default:
      batteryStatus.battChargingStatus = CHARGING_STATUS_ERROR;
      break;
    }
  }

  batteryStatus.battStatLedFlash = 0;
  if (shimmerStatus.docked)
  {
    if (batteryStatus.battChargingStatus == CHARGING_STATUS_SUSPENDED)
    {
#if defined(SHIMMER3)
      batteryStatus.battStatLedCharging = LED_YELLOW;
#elif defined(SHIMMER3R)
      batteryStatus.battStatLedCharging = LED_RGB_YELLOW;
#endif
    }
    else if (batteryStatus.battChargingStatus == CHARGING_STATUS_UNKNOWN
        || batteryStatus.battChargingStatus == CHARGING_STATUS_BAD_BATTERY
        || batteryStatus.battChargingStatus == CHARGING_STATUS_ERROR)
    {
#if defined(SHIMMER3)
      batteryStatus.battStatLedCharging = LED_RED;
#elif defined(SHIMMER3R)
      batteryStatus.battStatLedCharging = LED_RGB_RED;
#endif
      batteryStatus.battStatLedFlash = 1;
    }
    else if (batteryStatus.battChargingStatus == CHARGING_STATUS_CHECKING
        || batteryStatus.battChargingStatus == CHARGING_STATUS_CHARGING)
    {
#if defined(SHIMMER3)
      batteryStatus.battStatLedCharging = LED_RED;
#elif defined(SHIMMER3R)
      batteryStatus.battStatLedCharging = LED_RGB_RED;
#endif
    }
    else if (batteryStatus.battChargingStatus == CHARGING_STATUS_FULLY_CHARGED)
    {
#if defined(SHIMMER3)
      batteryStatus.battStatLedCharging = LED_GREEN0;
#elif defined(SHIMMER3R)
      batteryStatus.battStatLedCharging = LED_RGB_GREEN;
#endif
    }
    else
    {
#if defined(SHIMMER3)
      batteryStatus.battStatLedCharging = LED_ALL_OFF;
#elif defined(SHIMMER3R)
      batteryStatus.battStatLedCharging = LED_RGB_ALL_OFF;
#endif
    }
  }
}

void setBatteryInterval(battAlarmInterval_t value)
{
  battAlarmInterval = value;
}

battAlarmInterval_t getBatteryInterval(void)
{
  return battAlarmInterval;
}

void resetBatteryCriticalCount(void)
{
  battCriticalCount = 0;
}

