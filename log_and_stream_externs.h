/*
 * shimmer_externs.h
 *
 *  Created on: Sep 23, 2024
 *      Author: MarkNolan
 */

#ifndef SHIMMER_EXTERNS_H_
#define SHIMMER_EXTERNS_H_

#include <Battery/shimmer_battery.h>
#include <log_and_stream_definitions.h>

#include "hal_FactoryTest.h"

#if defined(SHIMMER3)
#include "../Shimmer_Driver/BMPX80/bmpX80.h"
#include "../Shimmer_Driver/MPU9150/mpu9150.h"
#include "hal_InfoMem.h"
#endif

extern STATTypeDef shimmerStatus;
extern BattStatus batteryStatus;

#if defined(SHIMMER3)
extern char *HAL_GetUID(void);
extern void RwcCheck(void);

extern void triggerShimmerErrorState(void);
extern uint64_t *getRwcTimeDiffPtr(void);
extern uint8_t isBtModuleOverflowPinHigh(void);

/* Task list */
extern void checkSetupDock(void);
extern void MPU9150_startMagMeasurement(void);
extern void BMPX80_startMeasurement(void);
//extern void checkStreamData(void);
//extern void checkStartSensing(void);
//extern void setStopSensingFlag(uint8_t state);
//extern void setStopLoggingFlag(uint8_t state);
//extern void setStopStreamingFlag(uint8_t state);

extern void SampleTimerStart(void);
extern void SampleTimerStop(void);
extern void Board_setExpansionBrdPower(uint8_t state);
#endif // SHIMMER3

extern void DockUart_enable(void);
extern void DockUart_disable(void);

extern void InitialiseBtAfterBoot(void);
extern void BtStop(uint8_t isCalledFromMain);

extern void sleepWhenNoTask(void);
extern uint8_t CheckSdInslot(void);
extern uint64_t RTC_get64(void);
extern void manageReadBatt(uint8_t isBlockingRead);
extern float samplingClockFreqGet(void);
extern void Board_setSdPower(uint8_t state);
extern uint8_t getDefaultBaudForBtVersion(void);
extern uint8_t InfoMem_write(uint16_t addr, uint8_t *buf, uint16_t size);
extern uint8_t InfoMem_read(uint16_t addr, uint8_t *buf, uint16_t size);

extern void setDmaWaitingForResponse(uint16_t count);

extern void setup_factory_test(factory_test_target_t target, factory_test_t testToRun);
extern void eepromRead(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf);
extern void eepromWrite(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf);

/* Sensing */
extern void ADC_configureChannels(void);
extern void I2C_configureChannels(void);
extern void SPI_configureChannels(void);

extern void ADC_startSensing(void);
extern void I2C_startSensing(void);
extern void SPI_startSensing(void);

extern void ADC_stopSensing(void);
extern void I2C_stopSensing(void);
extern void SPI_stopSensing(void);
extern void stopSensingWrapup(void);

extern void ADC_gatherDataStart(void);
extern void I2C_pollSensors(void);
extern void SPI_pollSensors(void);

#if defined(SHIMMER3)
extern void calculateClassicBtTxSampleSetBufferSize(uint8_t len, uint16_t samplingRateTicks);
#endif // SHIMMER3

#endif /* SHIMMER_EXTERNS_H_ */
