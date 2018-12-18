/*
 * led.h
 *
 *  Created on: 18 Dec 2018
 *      Author: lch19
 */

#ifndef DEVICE_DRIVER_LED_H_
#define DEVICE_DRIVER_LED_H_

#include "driver/gpio.h"
#include "../config/config.h"

namespace DeviceDriver{

class Led{
public:
	struct Config{
		uint8_t id;
		bool is_enable = true;
		bool is_active_low = true;
	};
	Led(const Config config);
	void Set(bool active){
		led_gpio.Set(is_active_low ^ active);
	}
	bool Toggle(){
		return led_gpio.Toggle() ^ is_active_low;
	}

private:
	Driver::GPIO led_gpio;
	bool is_active_low;
};

}


#endif /* DEVICE_DRIVER_LED_H_ */
