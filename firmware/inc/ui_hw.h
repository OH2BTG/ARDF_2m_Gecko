/* SPDX-License-Identifier: MIT */

/*
 * ui_hw.h
 * Some hardware-specific parts of UI code
 */

#ifndef INC_UI_HW_H_
#define INC_UI_HW_H_

#include "em_timer.h"
#include "em_gpio.h"
#include "InitDevice.h"

static inline unsigned get_encoder_position() {
	return TIMER_CounterGet(TIMER1);
}
static inline unsigned get_encoder_button() {
	return GPIO_PinInGet(ENCP_PORT, ENCP_PIN) == 0;
}
static inline unsigned get_ptt() { // ARDF changed to get_pwr_on
	return GPIO_PinInGet(PTT_PORT, PTT_PIN) == 0;
}
static inline unsigned get_pwr_on() { // ARDF changed to get_pwr_on
	return GPIO_PinInGet(PTT_PORT, PTT_PIN) == 0;
}
static inline unsigned get_ardf_F11_button()  {
	return GPIO_PinInGet(ARDF_F11_PORT, ARDF_F11_PIN) == 0;
}
static inline unsigned get_ardf_pa5_button()  {
	return GPIO_PinInGet(ARDF_PA5_PORT, ARDF_PA5_PIN) == 0;
}

#endif /* INC_UI_HW_H_ */
