/* SPDX-License-Identifier: MIT */
#ifndef INC_DSP_DRIVER_H_
#define INC_DSP_DRIVER_H_

#include "rail.h"

void dsp_hw_init(void);
void dsp_rtos_init(void);
void fast_dsp_task(void *);
void audio_tone ( duration, tone); // ARDF OH2BTG
int start_rx_dsp(RAIL_Handle_t rail);
int start_tx_dsp(RAIL_Handle_t rail);
int read_batt_adc(); // ARDF
extern uint32_t readADC; 

#endif
