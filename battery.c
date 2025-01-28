/*
 * battery.c
 *
 *  Created on: Jan 28, 2025
 *      Author: MarkNolan
 */

#include "battery.h"
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

  S4_ADC_rankBatt();
  rankBattChargingStatus();
}

void S4_NORM_ADC_rankBatt(void)
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
    batteryStatus.battStatLed = LED_RGB_RED;
    break;
  case BATT_MID:
    batteryStatus.battStatLed = LED_RGB_YELLOW;
    break;
  case BATT_HIGH:
    batteryStatus.battStatLed = LED_RGB_GREEN;
    break;
  default:
    batteryStatus.battStatLed = LED_RGB_RED;
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
      batteryStatus.battStatLedCharging = LED_RGB_YELLOW;
    }
    else if (batteryStatus.battChargingStatus == CHARGING_STATUS_UNKNOWN
        || batteryStatus.battChargingStatus == CHARGING_STATUS_BAD_BATTERY
        || batteryStatus.battChargingStatus == CHARGING_STATUS_ERROR)
    {
      batteryStatus.battStatLedCharging = LED_RGB_RED;
      batteryStatus.battStatLedFlash = 1;
    }
    else if (batteryStatus.battChargingStatus == CHARGING_STATUS_CHECKING
        || batteryStatus.battChargingStatus == CHARGING_STATUS_CHARGING)
    {
      batteryStatus.battStatLedCharging = LED_RGB_RED;
    }
    else if (batteryStatus.battChargingStatus == CHARGING_STATUS_FULLY_CHARGED)
    {
      batteryStatus.battStatLedCharging = LED_RGB_GREEN;
    }
    else
    {
      batteryStatus.battStatLedCharging = LED_RGB_ALL_OFF;
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

