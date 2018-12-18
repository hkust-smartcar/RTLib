/*
 * gpio_array.h
 *
 *  Created on: Sep 19, 2018
 *      Author: LeeChunHei
 */

#ifndef INC_DRIVER_GPIO_ARRAY_H_
#define INC_DRIVER_GPIO_ARRAY_H_

#include "system/pinout/pinout.h"
#include <vector>

namespace Driver {

class GPIOArray {
public:
	enum struct Direction {
		kDigitalInput, kDigitalOutput
	};
	struct Config {
		std::vector<System::Pinout::Name> pin;	//First pin is the LSB
		System::Pinout::Config::PinConfig pin_config;
		Direction gpio_dir;
		bool default_high = false;
	};
	GPIOArray(Config& config);
	/*
	 * @brief Set the output logic level, do nothing when it is a digital input
	 *
	 * @param output_level: pin logic level.
	 */
	void Set(uint32_t output) {
		if (had_mask) {
			uint32_t dr = pin_info[0].gpio_base->DR & mask;
			dr |= (output << shift);
			pin_info[0].gpio_base->DR = dr;
		} else {
			uint8_t shift_count = 0;
			for (uint8_t i = 0; i < pin_info.size(); i++) {
				uint32_t dr = pin_info[i].gpio_base->DR;
				for (uint8_t j = 0; j < pin_info[i].gpio_pin.size(); j++) {
					if ((output >> shift_count++) & 1) {
						dr |= (1u << pin_info[i].gpio_pin[j]);
					} else {
						dr &= ~(1u << pin_info[i].gpio_pin[j]);
					}
				}
				pin_info[i].gpio_base->DR = dr;
			}
		}
	}
	/*
	 * @brief Toggle the output logic level, do nothing when it is a digital input.
	 */
	void Toggle() {
		if (had_mask) {
			pin_info[0].gpio_base->DR_TOGGLE = ~(mask);
		} else {
			for (uint8_t i = 0; i < pin_info.size(); i++) {
				uint32_t toggle_reg = 0;
				for (uint8_t j = 0; j < pin_info[i].gpio_pin.size(); j++) {
					toggle_reg |= 1u << pin_info[i].gpio_pin[j];
				}
				pin_info[i].gpio_base->DR = toggle_reg;
			}
		}
	}
	/*
	 * @brief Get the input logic level.
	 *
	 * @retval input logic level.
	 */
	uint32_t Get() const {
		if (had_mask) {
			return (pin_info[0].gpio_base->DR & (~mask)) >> shift;
		} else {
			uint32_t data = 0;
			uint8_t shift_count = 0;
			for (uint8_t i = 0; i < pin_info.size(); i++) {
				uint32_t dr = pin_info[i].gpio_base->DR;
				for (uint8_t j = 0; j < pin_info[i].gpio_pin.size(); j++) {
					if ((dr >> pin_info[i].gpio_pin[j]) & 1) {
						data |= (1u << shift_count++);
					} else {
						data &= ~(1u << shift_count++);
					}
				}
			}
			return data;
		}
	}

private:
	struct PinInfo {
		GPIO_Type* gpio_base;
		std::vector<uint8_t> gpio_pin;
	};
	std::vector<PinInfo> pin_info;
	uint32_t mask = 0;
	bool had_mask = false;
	uint8_t shift;
};

}

#endif /* INC_DRIVER_GPIO_ARRAY_H_ */
