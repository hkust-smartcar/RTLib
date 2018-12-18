/*
 * MT9V034.h
 *
 *  Created on: Sep 15, 2018
 *      Author: LeeChunHei
 */

#ifndef INC_DEVICE_DRIVER_MT9V034_H_
#define INC_DEVICE_DRIVER_MT9V034_H_

#include "driver/csi.h"
#include "driver/i2c_master.h"
#include "queue"

namespace DeviceDriver {

class MT9V034 {
public:
	struct Config {
		enum struct HDR {
			kDisable, k80dB, k100dB
		};
		uint8_t id = 0;
		uint16_t width = 752;
		uint16_t height = 480;
		HDR hdr_mode = HDR::kDisable;
		Driver::I2CMaster* i2c_master = nullptr;
	};
	MT9V034(const Config& config);
	void Start();
	void Stop();
	inline uint16_t GetWidth() const {
		return width;
	}
	inline uint16_t GetHeight() const {
		return height;
	}
	inline bool IsAvailable() const {
		return (bool) (ready_buffer_addr.size());
	}
	inline const uint8_t* LockBuffer() {
		return (uint8_t*) (ready_buffer_addr.front());
	}
	void UnlockBuffer();
private:
	void RegSet(uint8_t reg_addr, uint16_t value);

	Driver::I2CMaster* i2c_master;
	Driver::Csi csi;
	uint16_t width, height;
	bool is_started = false;

	std::queue<uint32_t> ready_buffer_addr;
	std::queue<uint32_t> empty_buffer_addr;
};

}



#endif /* INC_DEVICE_DRIVER_MT9V034_H_ */
