/*
 * battery.h
 *
 *  Created on: Jan 28, 2025
 *      Author: MarkNolan
 */

#ifndef BATTERY_H_
#define BATTERY_H_

#include <stdint.h>

#define BATT_LOW  0x01
#define BATT_MID  0x02
#define BATT_HIGH 0x04
#if defined(SHIMMER4_SDK)
#define BATT_INTERVAL   600 //600 seconds = 10min interval
#define BATT_INTERVAL_D 30  //30 seconds
#endif

#define BATT_LOW_MAX              2618
#define BATT_MID_MIN              2568
#define BATT_MID_MAX              2767
#define BATT_HIGH_MIN             2717

#define BATTERY_ERROR_VOLTAGE_MAX 4500 //mV
#define BATTERY_ERROR_VOLTAGE_MIN 3200 //mV

#if defined(SHIMMER3)
#define BATT_INTERVAL   1966080 //10min interval
#define BATT_INTERVAL_D 65535   //30sec interval
#endif

typedef enum
{
  BATT_INTERVAL_UNDOCKED, //10 Minutes
  BATT_INTERVAL_DOCKED    //30 Seconds
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

void batteryInit(void);
void updateBatteryStatus(uint16_t adc_battVal, uint16_t battValMV, uint8_t lm3658sdStat1, uint8_t lm3658sdStat2);
void rankBattUndockedVoltage(void);
void rankBattChargingStatus(void);
void determineChargingLedState(void);
void determineUndockedLedState(void);
void resetBatteryChargingStatus(void);
void resetBatteryUndockedStatus(void);
void setBatteryInterval(battAlarmInterval_t value);
battAlarmInterval_t getBatteryInterval(void);
#if defined(SHIMMER3)
uint32_t getBatteryIntervalTicks(void);
#endif
void resetBatteryCriticalCount(void);
void incrementBatteryCriticalCount(void);
void setBattCritical(uint8_t state);
uint8_t checkIfBatteryCritical(void);

#endif /* BATTERY_H_ */
