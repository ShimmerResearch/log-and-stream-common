/*
 * platform_api.h
 *
 *  Created on: Oct 16, 2025
 *      Author: MarkNolan
 */

#ifndef PLATFORM_API_H_
#define PLATFORM_API_H_

#include <stdint.h>

#ifndef __weak
#define __weak __attribute__((weak))
#endif /* __weak */

__weak void delay_ms(const uint32_t delay_time_ms);
__weak void ProcessHwRevision(void);
__weak void Board_initGpioForRevision(void);

#endif /* PLATFORM_API_H_ */
