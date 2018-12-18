/*
 * voltage_meter.h
 *
 *  Created on: 1 Dec 2018
 *      Author: lch19
 */

#ifndef SRC_DEVICE_DRIVER_VOLTAGE_METER_H_
#define SRC_DEVICE_DRIVER_VOLTAGE_METER_H_

#include "driver/adc.h"

namespace DeviceDriver{

class VoltageMeter{
public:
	struct Config{
		uint8_t id = 0;
		/*
		 *
		 * (Voltage)----[ R1 ]--|--[ R2 ]----(GND)
		 * 					    |
		 * 				        |
		 * 				   (MCU Input)
		 *
		 * voltage_divider_ratio = R2 / ( R1 + R2 )
		 *
		 */
		float voltage_divider_ratio;
	};
	VoltageMeter(const Config);
	inline float GetVoltage() const{
		adc.StartChannelConversation(input);
		while(!adc.IsComplete(channel));
		return voltage_divider_ratio * adc.GetValue(channel);
	}

private:
	Driver::ADC adc;
	Driver::ADC::Channel channel;
	Driver::ADC::Input input;
	float voltage_divider_ratio;

};

}

#endif /* SRC_DEVICE_DRIVER_VOLTAGE_METER_H_ */
