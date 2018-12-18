/*
 * gpio_array.cpp
 *
 *  Created on: Sep 19, 2018
 *      Author: LeeChunHei
 */

#include "driver/gpio_array.h"
#include "system/clock.h"

namespace Driver {

GPIOArray::GPIOArray(Config& config) {
	System::Pinout::Config pin_config;
	pin_config.mux_mode = System::Pinout::Config::MuxMode::kAlt5;
	pin_config.pin_config = config.pin_config;
	pin_config.force_input=true;
	for (uint32_t i = 0; i < config.pin.size(); i++) {
		pin_config.pin = config.pin[i];
		System::Pinout::InitPin(pin_config);
		GPIO_Type* gpio_base;
		uint8_t gpio_pin;
		if (config.pin[i] <= System::Pinout::Name::kGPIO_EMC_31) {
			gpio_base = GPIO4;
			gpio_pin = (uint8_t) config.pin[i];
			System::CLOCK_EnableClock(System::kCLOCK_Gpio4);
		} else if (config.pin[i] <= System::Pinout::Name::kGPIO_EMC_41) {
			gpio_base = GPIO3;
			gpio_pin = ((uint8_t) config.pin[i] - (uint8_t) System::Pinout::Name::kGPIO_EMC_32 + 18);
			System::CLOCK_EnableClock(System::kCLOCK_Gpio3);
		} else if (config.pin[i] <= System::Pinout::Name::kGPIO_AD_B1_15) {
			gpio_base = GPIO1;
			gpio_pin = ((uint8_t) config.pin[i] - (uint8_t) System::Pinout::Name::kGPIO_AD_B0_00);
			System::CLOCK_EnableClock(System::kCLOCK_Gpio1);
		} else if (config.pin[i] <= System::Pinout::Name::kGPIO_B1_15) {
			gpio_base = GPIO2;
			gpio_pin = ((uint8_t) config.pin[i] - (uint8_t) System::Pinout::Name::kGPIO_B0_00);
			System::CLOCK_EnableClock(System::kCLOCK_Gpio2);
		} else if (config.pin[i] <= System::Pinout::Name::kGPIO_SD_B0_05) {
			gpio_base = GPIO3;
			gpio_pin = ((uint8_t) config.pin[i] - (uint8_t) System::Pinout::Name::kGPIO_SD_B0_00 + 12);
			System::CLOCK_EnableClock(System::kCLOCK_Gpio3);
		} else if (config.pin[i] <= System::Pinout::Name::kGPIO_SD_B1_11) {
			gpio_base = GPIO3;
			gpio_pin = ((uint8_t) config.pin[i] - (uint8_t) System::Pinout::Name::kGPIO_SD_B1_00);
			System::CLOCK_EnableClock(System::kCLOCK_Gpio3);
		}
		gpio_base->IMR &= ~(1u << gpio_pin);
		if (config.gpio_dir == Direction::kDigitalInput) {
			gpio_base->GDIR &= ~(1u << gpio_pin);
		} else {
			gpio_base->GDIR |= (1u << gpio_pin);
		}
		if (pin_info.size()) {
			if (gpio_base == pin_info.back().gpio_base) {
				pin_info.back().gpio_pin.push_back(gpio_pin);
			} else {
				PinInfo pin_info_temp;
				pin_info_temp.gpio_base = gpio_base;
				pin_info_temp.gpio_pin.push_back(gpio_pin);
				pin_info.push_back(pin_info_temp);
			}
		} else {
			PinInfo pin_info_temp;
			pin_info_temp.gpio_base = gpio_base;
			pin_info_temp.gpio_pin.push_back(gpio_pin);
			pin_info.push_back(pin_info_temp);
		}
	}
	if (pin_info.size() == 1) {
		shift = pin_info.back().gpio_pin[0];
		mask = 1;
		for (int i = 1; i < pin_info.back().gpio_pin.size(); i++) {
			if ((pin_info.back().gpio_pin[i] - pin_info.back().gpio_pin[i - 1]) == 1) {
				mask |= (1 << i);
			} else {
				mask = 0;
				break;
			}
		}
		if (mask) {
			mask = ~(mask << shift);
			had_mask = true;
		}
	}
}

}
