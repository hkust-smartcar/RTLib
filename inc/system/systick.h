/*
 * systick.h
 *
 *  Created on: Aug 22, 2018
 *      Author: LeeChunHei
 */

#ifndef INC_DRIVER_SYSTICK_H_
#define INC_DRIVER_SYSTICK_H_

#include "system/cmsis/access_layer/access_layer.h"
#include "system/system.h"
#include "../config/config.h"

namespace System {

class Systick {
public:
	static void Init();
	static inline uint32_t GetTimeInS() {
		return ((SysTick_LOAD_RELOAD_Msk - SysTick->VAL + tick_interrupt_count * SysTick_LOAD_RELOAD_Msk) / system_core_clock);
	}
	static inline uint32_t GetTimeInMS() {
		return ((SysTick_LOAD_RELOAD_Msk - SysTick->VAL + tick_interrupt_count * SysTick_LOAD_RELOAD_Msk) / system_core_clock * 1000);
	}
	static inline uint32_t GetTimeInNS() {
		return ((SysTick_LOAD_RELOAD_Msk - SysTick->VAL + tick_interrupt_count * SysTick_LOAD_RELOAD_Msk) / system_core_clock * 1000000);
	}
	static inline uint32_t GetTickCount() {
		return (SysTick_LOAD_RELOAD_Msk - SysTick->VAL + tick_interrupt_count * SysTick_LOAD_RELOAD_Msk);
	}
	static inline void DelayS(uint32_t second) {
		while (second--) {
			DelayMS(1000);
		}
	}
	static inline void DelayMS(uint32_t milisecond) {
		while (milisecond--) {
			DelayUS(1000);
		}
	}
	static inline void DelayUS(uint32_t microsecond) {
		uint32_t load_val = (uint32_t)(1000 * (system_core_clock / 1000000000.0));
		while (microsecond--) {
			DelayCount(load_val);
		}
	}
	static inline void DelayNS(uint32_t nanosecond) {
		volatile uint32_t temp = SysTick->VAL;
		SysTick->CTRL = 0x00;
		SysTick->LOAD = (uint32_t)(nanosecond * (system_core_clock / 1000000000.0));
		SysTick->VAL = 0x00;
		SysTick->CTRL = (0 | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk);
		while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk))
			;
		SysTick->LOAD = (uint32_t)(SysTick_LOAD_RELOAD_Msk);
		SysTick->VAL = temp;
		SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; //Enable back systick interrupt
	}
	static inline void DelayCount(uint32_t count) {
		volatile uint32_t temp = SysTick->VAL;
		SysTick->CTRL = 0x00;
		SysTick->LOAD = count - 8;
		SysTick->VAL = 0x00;
		SysTick->CTRL = (0 | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk);
		while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk))
			;
		SysTick->LOAD = (uint32_t)(SysTick_LOAD_RELOAD_Msk);
		SysTick->VAL = temp;
		SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; //Enable back systick interrupt
	}
	static volatile uint32_t tick_interrupt_count;
};

}

#endif /* INC_DRIVER_SYSTICK_H_ */
