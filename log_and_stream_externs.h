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
#include "../shimmer3_common_source/BMPX80/bmpX80.h"
#include "../shimmer3_common_source/MPU9150/mpu9150.h"
#include "hal_InfoMem.h"
#endif

extern STATTypeDef shimmerStatus;
extern BattStatus batteryStatus;

#if defined(SHIMMER3)
extern char *HAL_GetUID(void);
extern void RwcCheck(void);
extern uint8_t* getMacIdBytesPtr(void);

/* Task list */
extern void checkSetupDock(void);
extern void ConfigureChannels(void);
extern void MPU9150_startMagMeasurement(void);
extern void BMPX80_startMeasurement(void);
extern void checkStreamData(void);
extern void checkStartSensing(void);
extern void setStopSensingFlag(uint8_t state);
extern void setStopLoggingFlag(uint8_t state);
extern void setStopStreamingFlag(uint8_t state);
#endif

extern void sleepNoTask(void);
extern uint8_t CheckSdInslot(void);
extern uint64_t RTC_get64(void);
extern uint8_t ShimTask_set(uint32_t task_id);
extern void manageReadBatt(uint8_t isBlockingRead);
extern float samplingClockFreqGet(void);
extern void Board_setSdPower(uint8_t state);
extern uint8_t getDefaultBaudForBtVersion(void);
extern uint8_t InfoMem_write(uint16_t addr, uint8_t *buf, uint16_t size);
extern uint8_t InfoMem_read(uint16_t addr, uint8_t *buf, uint16_t size);
extern float samplingClockFreqGet(void);

extern void setup_factory_test(factory_test_target_t target, factory_test_t testToRun);
extern void eepromRead(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf);
extern void eepromWrite(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf);

#endif /* SHIMMER_EXTERNS_H_ */
