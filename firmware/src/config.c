/* SPDX-License-Identifier: MIT */

#include "config.h"
#include "em_gpio.h"
#include "InitDevice.h"

bool tx_freq_allowed(uint32_t frequency)
{
	if (frequency >= 1UL && frequency <= 2500000000UL) {
		// Test mode: allow transmitting on any frequency
		return 1;
	}
	if (frequency >= 100000UL && frequency <= 1438000000UL)  // min and max tx on 
		return 1;
	if (frequency >= 2300000000UL && frequency <= 2450000000UL)
		return 1;
	return 0;
}
