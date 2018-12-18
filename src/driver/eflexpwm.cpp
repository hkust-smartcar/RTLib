/*
 * eflexpwm.cpp
 *
 *  Created on: Aug 12, 2018
 *      Author: LeeChunHei
 */

#include "driver/eflexpwm.h"
#include "system/clock.h"
#include "system/system.h"
#include <assert.h>

namespace Driver {

eFlexPWM::eFlexPWM(Config& config) :
		sub_module(config.sub_module), alignment(config.alignment) {
	//Source clock for submodule 0 cannot be itself
	assert(config.clock_source != Config::ClockSource::kSubmodule0Clock || config.sub_module != Config::SubModule::kSubModule0);
	//Reload source for submodule 0 cannot be master reload
	assert(!config.is_master_reload || config.sub_module != Config::SubModule::kSubModule0);
	System::CLOCK_EnableClock((System::clock_ip_name_t)(1040 + (uint8_t)(config.pwm_module) * 2));
	pwm_base = (PWM_Type*) (0x403DC000u + (uint8_t)(config.pwm_module) * 0x4000);
	pwm_base->FSTS |= PWM_FSTS_FFLAG_MASK;
	uint16_t reg = pwm_base->SM[(uint8_t) sub_module].CTRL2;
	reg &= ~(PWM_CTRL2_CLK_SEL_MASK | PWM_CTRL2_FORCE_SEL_MASK | PWM_CTRL2_INIT_SEL_MASK | PWM_CTRL2_INDEP_MASK | PWM_CTRL2_WAITEN_MASK | PWM_CTRL2_DBGEN_MASK | PWM_CTRL2_RELOAD_SEL_MASK);
	reg |= (PWM_CTRL2_CLK_SEL(config.clock_source) | PWM_CTRL2_FORCE_SEL(config.force_trigger) | PWM_CTRL2_INIT_SEL(config.initialization_control) | PWM_CTRL2_DBGEN(config.enable_debug_mode) | PWM_CTRL2_WAITEN(config.enable_wait) | PWM_CTRL2_RELOAD_SEL(config.is_master_reload));
	switch (config.channel_pair_operation) {
	case Config::ChannelPairOperation::kIndependent:
		reg |= PWM_CTRL2_INDEP_MASK;
		break;
	case Config::ChannelPairOperation::kComplementaryPwmA:
		pwm_base->MCTRL &= ~(1U << (PWM_MCTRL_IPOL_SHIFT + (uint8_t) sub_module));
		break;
	case Config::ChannelPairOperation::kComplementaryPwmB:
		pwm_base->MCTRL |= (1U << (PWM_MCTRL_IPOL_SHIFT + (uint8_t) sub_module));
		break;
	default:
		break;
	}
	pwm_base->SM[(uint8_t) sub_module].CTRL2 = reg;
	reg = pwm_base->SM[(uint8_t) sub_module].CTRL;

	/* Setup the clock prescale, load mode and frequency */
	period = (System::ipg_clock >> 16) / config.frequency;
	uint8_t prescale = 0;
	for (; (period >> prescale) >= 1; ++prescale) {
	}
	period = (System::ipg_clock >> prescale) / config.frequency;
	uint16_t mod = period >> 1;
	if ((uint8_t) config.alignment % 2) {
		pwm_base->SM[(uint8_t) sub_module].INIT = 0;
		pwm_base->SM[(uint8_t) sub_module].VAL0 = mod;
		pwm_base->SM[(uint8_t) sub_module].VAL1 = period;
	} else {
		pwm_base->SM[(uint8_t) sub_module].INIT = -mod;
		pwm_base->SM[(uint8_t) sub_module].VAL0 = 0;
		pwm_base->SM[(uint8_t) sub_module].VAL1 = mod;
	}
	assert(prescale < 8);
	reg &= ~(PWM_CTRL_PRSC_MASK | PWM_CTRL_LDFQ_MASK | PWM_CTRL_LDMOD_MASK);
	reg |= (PWM_CTRL_PRSC(prescale) | PWM_CTRL_LDFQ(config.reload_frequence));

	/* Setup register reload logic */
	switch (config.reload_logic) {
	case Config::ReloadLogic::kReloadImmidate:
		reg |= PWM_CTRL_LDMOD_MASK;
		break;
	case Config::ReloadLogic::kReloadHalfCycle:
		reg |= PWM_CTRL_HALF_MASK;
		reg &= ~PWM_CTRL_FULL_MASK;
		break;
	case Config::ReloadLogic::kReloadFullCycle:
		reg &= ~PWM_CTRL_HALF_MASK;
		reg |= PWM_CTRL_FULL_MASK;
		break;
	case Config::ReloadLogic::kReloadHalfAndFullCycle:
		reg |= PWM_CTRL_HALF_MASK;
		reg |= PWM_CTRL_FULL_MASK;
		break;
	default:
		break;
	}
	pwm_base->SM[(uint8_t) sub_module].CTRL = reg;

	/* Setup the fault filter */
	if (pwm_base->FFILT & PWM_FFILT_FILT_PER_MASK) {
		/* When changing values for fault period from a non-zero value, first write a value of 0
		 * to clear the filter
		 */
		pwm_base->FFILT &= ~(PWM_FFILT_FILT_PER_MASK);
	}

	pwm_base->FFILT = (PWM_FFILT_FILT_CNT(0) | PWM_FFILT_FILT_PER(0));

	/* Issue a Force trigger event when configured to trigger locally */
	if (config.force_trigger == Config::ForceTrigger::kForce_Local) {
		pwm_base->SM[(uint8_t) sub_module].CTRL2 |= PWM_CTRL2_FORCE(1U);
	}

	pwm_base->MCTRL |= PWM_MCTRL_LDOK(1u << (uint8_t) sub_module);
	pwm_base->MCTRL |= PWM_MCTRL_RUN(1u << (uint8_t) sub_module);
	pwm_base->SM[(uint8_t) sub_module].DISMAP[0] = 0;
	pwm_base->SM[(uint8_t) sub_module].DISMAP[1] = 0;
}

void eFlexPWM::InitChannel(ChannelConfig& config) {
	assert(config.duty_cycle <= 1000);
	uint16_t high_count = period * config.duty_cycle / 1000;
	switch (alignment) {
	case Config::Alignment::kSignedCenterAligned:
		if (config.channel == Channel::kPWMA) {
			pwm_base->SM[(uint8_t) sub_module].VAL2 = -(high_count >> 1);
			pwm_base->SM[(uint8_t) sub_module].VAL3 = high_count >> 1;
		} else {
			pwm_base->SM[(uint8_t) sub_module].VAL4 = -(high_count >> 1);
			pwm_base->SM[(uint8_t) sub_module].VAL5 = high_count >> 1;
		}
		break;
	case Config::Alignment::kCenterAligned:
		if (config.channel == Channel::kPWMA) {
			pwm_base->SM[(uint8_t) sub_module].VAL2 = (period - high_count) >> 1;
			pwm_base->SM[(uint8_t) sub_module].VAL3 = (period + high_count) >> 1;
		} else {
			pwm_base->SM[(uint8_t) sub_module].VAL4 = (period - high_count) >> 1;
			pwm_base->SM[(uint8_t) sub_module].VAL5 = (period + high_count) >> 1;
		}
		break;
	case Config::Alignment::kSignedEdgeAligned:
		if (config.channel == Channel::kPWMA) {
			uint16_t mod = period >> 1;
			pwm_base->SM[(uint8_t) sub_module].VAL2 = -mod;
			pwm_base->SM[(uint8_t) sub_module].VAL3 = -mod + high_count;
		} else {
			uint16_t mod = period >> 1;
			pwm_base->SM[(uint8_t) sub_module].VAL4 = -mod;
			pwm_base->SM[(uint8_t) sub_module].VAL5 = -mod + high_count;
		}
		break;
	case Config::Alignment::kEdgeAligned:
		if (config.channel == Channel::kPWMA) {
			pwm_base->SM[(uint8_t) sub_module].VAL2 = 0;
			pwm_base->SM[(uint8_t) sub_module].VAL3 = high_count;
		} else {
			pwm_base->SM[(uint8_t) sub_module].VAL4 = 0;
			pwm_base->SM[(uint8_t) sub_module].VAL5 = high_count;
		}
		break;
	default:
		break;
	}
	uint8_t polarity_shift;
	uint8_t output_enable_shift;
	if (config.channel == Channel::kPWMA) {
		polarity_shift = PWM_OCTRL_POLA_SHIFT;
		output_enable_shift = PWM_OUTEN_PWMA_EN_SHIFT;
		pwm_base->SM[(uint8_t) sub_module].DTCNT0 = PWM_DTCNT0_DTCNT0(config.dead_time);
	} else {
		polarity_shift = PWM_OCTRL_POLB_SHIFT;
		output_enable_shift = PWM_OUTEN_PWMB_EN_SHIFT;
		pwm_base->SM[(uint8_t) sub_module].DTCNT1 = PWM_DTCNT1_DTCNT1(config.dead_time);
	}

	/* Setup signal active level */
	if (config.polarity_inverted) {
		pwm_base->SM[(uint8_t) sub_module].OCTRL |= (1U << polarity_shift);
	} else {
		pwm_base->SM[(uint8_t) sub_module].OCTRL &= ~(1U << polarity_shift);
	}
	/* Enable PWM output */
	pwm_base->OUTEN |= (1U << (output_enable_shift + (uint8_t) sub_module));
	pwm_base->MCTRL |= PWM_MCTRL_LDOK(1u << (uint8_t) sub_module);
}

bool eFlexPWM::OpenPWMPin(System::Pinout::Config& config) {
	if (System::Pinout::GetPWMPinConfig(config, pwm_base, (uint8_t) sub_module)) {
		System::Pinout::InitPin(config);
		return true;
	} else {
		return false;
	}
}

void eFlexPWM::ClosePWMPin(System::Pinout::Name pin_name) {

}

void eFlexPWM::SetDutyCycle(Channel channel, uint16_t duty_cycle, bool duty_cycle_in_percentage) {
	uint16_t high_count = duty_cycle_in_percentage ? period * duty_cycle / 1000 : duty_cycle;
	assert(high_count <= period);
	switch (alignment) {
	case Config::Alignment::kSignedCenterAligned:
		if (channel == Channel::kPWMA) {
			pwm_base->SM[(uint8_t) sub_module].VAL2 = -(high_count >> 1);
			pwm_base->SM[(uint8_t) sub_module].VAL3 = high_count >> 1;
		} else {
			pwm_base->SM[(uint8_t) sub_module].VAL4 = -(high_count >> 1);
			pwm_base->SM[(uint8_t) sub_module].VAL5 = high_count >> 1;
		}
		break;
	case Config::Alignment::kCenterAligned:
		if (channel == Channel::kPWMA) {
			pwm_base->SM[(uint8_t) sub_module].VAL2 = (period - high_count) >> 1;
			pwm_base->SM[(uint8_t) sub_module].VAL3 = (period + high_count) >> 1;
		} else {
			pwm_base->SM[(uint8_t) sub_module].VAL4 = (period - high_count) >> 1;
			pwm_base->SM[(uint8_t) sub_module].VAL5 = (period + high_count) >> 1;
		}
		break;
	case Config::Alignment::kSignedEdgeAligned:
		if (channel == Channel::kPWMA) {
			uint16_t mod = period >> 1;
			pwm_base->SM[(uint8_t) sub_module].VAL2 = -mod;
			pwm_base->SM[(uint8_t) sub_module].VAL3 = -mod + high_count;
		} else {
			uint16_t mod = period >> 1;
			pwm_base->SM[(uint8_t) sub_module].VAL4 = -mod;
			pwm_base->SM[(uint8_t) sub_module].VAL5 = -mod + high_count;
		}
		break;
	case Config::Alignment::kEdgeAligned:
		if (channel == Channel::kPWMA) {
			pwm_base->SM[(uint8_t) sub_module].VAL2 = 0;
			pwm_base->SM[(uint8_t) sub_module].VAL3 = high_count;
		} else {
			pwm_base->SM[(uint8_t) sub_module].VAL4 = 0;
			pwm_base->SM[(uint8_t) sub_module].VAL5 = high_count;
		}
		break;
	default:
		break;
	}
	pwm_base->MCTRL |= PWM_MCTRL_LDOK(1u << (uint8_t) sub_module);
}

}
