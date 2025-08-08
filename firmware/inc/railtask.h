/* SPDX-License-Identifier: MIT */

#ifndef INC_RAILTASK_H_
#define INC_RAILTASK_H_

#include "FreeRTOS.h"
#include "semphr.h"

void railtask_main(void *);
void railtask_rtos_init(void);
extern RAIL_Handle_t rail;
extern int16_t RSSI_read;
void ARDF_RSSI ();
int16_t getRssi(RAIL_Handle_t rail);

// Semaphore used to wake up RAIL task when it needs to do something.
extern xSemaphoreHandle railtask_sem;

#endif
