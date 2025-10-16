/*
 * battery.h
 *
 *  Created on: Jan 28, 2025
 *      Author: MarkNolan
 */

#ifndef BATTERY_H_
#define BATTERY_H_

#include <stdint.h>

#define BATT_LOW                  0x01
#define BATT_MID                  0x02
#define BATT_HIGH                 0x04

#define BATT_LOW_MAX              2618
#define BATT_MID_MIN              2568
#define BATT_MID_MAX              2767
#define BATT_HIGH_MIN             2717

#define BATTERY_ERROR_VOLTAGE_MAX 4500 //mV
#define BATTERY_ERROR_VOLTAGE_MIN 3200 //mV

/* approx. 10% cutoff voltage - 3.65 Volts */
#define BATT_CUTOFF_3_65VOLTS     (2500)

#if defined(SHIMMER3)
#define BATT_INTERVAL_TICKS_UNDOCKED (BATT_INTERVAL_SECS_UNDOCKED * 32768)
#define BATT_INTERVAL_TICKS_DOCKED   (BATT_INTERVAL_SECS_DOCKED * 32768)
#endif

typedef enum
{
  BATT_INTERVAL_SECS_UNDOCKED = 60, //1min interval
  BATT_INTERVAL_SECS_DOCKED = 2     //2sec interval
} battAlarmInterval_t;

enum
{
  //STAT2 = bit7, STAT1 = bit 6
  CHRG_CHIP_STATUS_SUSPENDED = 0xC0,       //STAT2 high (off), STAT1 high (off)
  CHRG_CHIP_STATUS_PRECONDITIONING = 0x80, //STAT2 high (off), STAT1 low (on)
  CHRG_CHIP_STATUS_FULLY_CHARGED = 0x40,   //STAT2 low (on), STAT1 high (off)
  CHRG_CHIP_STATUS_BAD_BATTERY = 0x00,     //STAT2 low (on), STAT1 low (on)
  CHRG_CHIP_STATUS_UNKNOWN = 0xFF,
};

typedef enum
{
  CHARGING_STATUS_UNKNOWN,
  CHARGING_STATUS_CHECKING,
  CHARGING_STATUS_SUSPENDED,
  CHARGING_STATUS_FULLY_CHARGED,
  CHARGING_STATUS_CHARGING,
  CHARGING_STATUS_BAD_BATTERY,
  CHARGING_STATUS_ERROR
} chargingStatus_t;

typedef union
{
  uint8_t rawBytes[3];

  struct __attribute__((packed))
  {
    uint16_t adcBattVal;

    //STAT2 sits in Bit7 and STAT1 in Bit6
    uint8_t unusedBits : 6;
    uint8_t STAT1      : 1;
    uint8_t STAT2      : 1;
  };
} BattStatusRaw;

typedef volatile struct batt_status_t
{
  /* General battery level based on ADC voltage with buffered min/max values */
  uint8_t battStat;
  /* LED colour to show when undocked based on the latest battStat */
  uint32_t battStatLed;
  /* LED colour to show when docked */
  uint32_t battStatLedCharging;
  /* Lets the LED timer know whether the LED should flash or stay solid */
  uint8_t battStatLedFlash : 1;
  /* The ADC value and charger status bytes which are sent via dock/BT */
  BattStatusRaw battStatusRaw;
  /* Latest measured battery voltage in mV */
  uint16_t battValMV;
  /* Overall status based on batt mV, charger chip status and docked/undocked */
  chargingStatus_t battChargingStatus;
#if defined(SHIMMER4_SDK)
  uint8_t battDigital[10];
#endif
  uint8_t battCritical : 1;
  uint8_t battCriticalCount;
} BattStatus;

void ShimBatt_init(void);
void ShimBatt_updateStatus(uint16_t adc_battVal,
    uint16_t battValMV,
    uint8_t lm3658sdStat1,
    uint8_t lm3658sdStat2);
void ShimBatt_rankBattUndockedVoltage(void);
void ShimBatt_rankBattChargingStatus(void);
void ShimBatt_determineChargingLedState(void);
void ShimBatt_determineUndockedLedState(void);
void ShimBatt_resetBatteryChargingStatus(void);
void ShimBatt_resetBatteryUndockedStatus(void);
void ShimBatt_setBatteryInterval(battAlarmInterval_t value);
battAlarmInterval_t ShimBatt_getBatteryInterval(void);
#if defined(SHIMMER3)
uint32_t ShimBatt_getBatteryIntervalTicks(void);
#endif
void ShimBatt_resetBatteryCriticalCount(void);
void ShimBatt_incrementBatteryCriticalCount(void);
void ShimBatt_setBattCritical(uint8_t state);
uint8_t ShimBatt_checkIfBatteryCritical(void);

#endif /* BATTERY_H_ */
