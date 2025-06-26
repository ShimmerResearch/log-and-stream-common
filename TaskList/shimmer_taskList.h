/*
 * Copyright (c) 2013, Shimmer Research, Ltd.
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *    * Neither the name of Shimmer Research, Ltd. nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *    * You may not use or distribute this Software or any derivative works
 *      in any form for commercial purposes with the exception of commercial
 *      purposes when used in conjunction with Shimmer products purchased
 *      from Shimmer or their designated agent or with permission from
 *      Shimmer.
 *      Examples of commercial purposes would be running business
 *      operations, licensing, leasing, or selling the Software, or
 *      distributing the Software for use with commercial products.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Weibo Pan
 * @date May, 2013
 */

#ifndef S4_TASKLIST_H
#define S4_TASKLIST_H

#include <stdint.h>

#if defined(SHIMMER3R)
#include "shimmer_definitions.h"
#include <shimmer_include.h>
#endif

#if USE_FREERTOS
#define ShimTask_init       S4_RTOS_Task_init
#define ShimTask_manage     S4_RTOS_Task_manage
#define ShimTask_getCurrent S4_RTOS_Task_getCurrent
#define ShimTask_clear      S4_RTOS_Task_clear
#define ShimTask_set        S4_RTOS_Task_set
#define ShimTask_getList    S4_RTOS_Task_getList
#else
#define ShimTask_init       ShimTask_NORM_init
#define ShimTask_manage     ShimTask_NORM_manage
#define ShimTask_getCurrent ShimTask_NORM_getCurrent
#define ShimTask_clear      ShimTask_NORM_clear
#define ShimTask_set        ShimTask_NORM_set
#define ShimTask_getList    ShimTask_NORM_getList
#endif //USE_FREERTOS

#define TASK_SIZE 32

/* In order of priority */
typedef enum
{
  TASK_NONE = 0,
  TASK_SETUP_DOCK = (0x00000001UL << 0U),
  TASK_BATT_READ = (0x00000001UL << 1U),
  TASK_DOCK_PROCESS_CMD = (0x00000001UL << 2U),
  TASK_DOCK_RESPOND = (0x00000001UL << 3U),
  TASK_BT_PROCESS_CMD = (0x00000001UL << 4U),
#if defined(SHIMMER3)
  TASK_CFGCH = (0x00000001UL << 5U),
#endif
  TASK_BT_RESPOND = (0x00000001UL << 6U),
  TASK_RCCENTERR1 = (0x00000001UL << 7U),
  TASK_RCNODER10 = (0x00000001UL << 8U),
#if defined(SHIMMER3)
  TASK_SAMPLE_MPU9150_MAG = (0x00000001UL << 9U),
  TASK_SAMPLE_BMPX80_PRESS = (0x00000001UL << 10U),
#endif
  TASK_STREAMDATA = (0x00000001UL << 11U),
  TASK_STOPSENSING = (0x00000001UL << 12U),
  TASK_SDLOG_CFG_UPDATE = (0x00000001UL << 13U),
  TASK_STARTSENSING = (0x00000001UL << 14U),
  TASK_SAVEDATA = (0x00000001UL << 15U),
  TASK_SDWRITE = (0x00000001UL << 16U),
  TASK_FACTORY_TEST = (0x00000001UL << 17U),
#if defined(SHIMMER3R) || defined(SHIMMER4_SDK)
  TASK_USB_SETUP = (0x00000001UL << 18U),
#endif
  TASK_BT_TX_BUF_CLEAR = (0x00000001UL << 19U),
  TASK_BT_TURN_ON_AFTER_BOOT = (0x00000001UL << 20U)

} TASK_FLAGS;
//return the task id of the current task

void ShimTask_NORM_init(void);

void ShimTask_NORM_manage(void);

uint32_t ShimTask_NORM_getCurrent(void);

//clear the task from the tasklist
void ShimTask_NORM_clear(uint32_t task_id);

//add the task to the tasklist
uint8_t ShimTask_NORM_set(uint32_t task_id);

//return the whole tasklist
uint32_t ShimTask_NORM_getList(void);

void ShimTask_setRestartSensing(void);
void ShimTask_setStartSensing(void);
void ShimTask_setStopSensing(void);
void ShimTask_setStopSdLogging(void);
void ShimTask_setStopBtStreaming(void);
void ShimTask_setInitialiseBluetooth(void);

#endif //S4_TASKLIST_H
