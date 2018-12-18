/*
 * soft_i2c_master.h
 *
 *  Created on: Sep 27, 2018
 *      Author: LeeChunHei
 */

#ifndef INC_DRIVER_SOFT_I2C_MASTER_H_
#define INC_DRIVER_SOFT_I2C_MASTER_H_

#include "driver/i2c_master.h"
#include "driver/gpio.h"

namespace Driver {

class SoftI2CMaster: public I2CMaster {
public:
	SoftI2CMaster(const I2CMaster::Config& config);

	/*
	 * @brief Send data to i2c device with corresponding slave address and 8-bit register address
	 *
	 * @param slave_addr: i2c device slave address, reg_addr: target register address, data: data to send.
	 *
	 * @retval status of transmission, true when transfer success.
	 */
	bool SendByte(const uint8_t slave_addr, const uint8_t reg_addr, const uint8_t data);

	/*
	 * @brief Send data to i2c device with corresponding slave address and 16-bit register address
	 *
	 * @param slave_addr: i2c device slave address, reg_addr: target register address, data: data to send.
	 *
	 * @retval status of transmission, true when transfer success.
	 */
	bool SendByte(const uint8_t slave_addr, const uint16_t reg_addr, const uint8_t data);

	/*
	 * @brief Send required number of data to i2c device with corresponding slave address and 8-bit register address
	 *
	 * @param slave_addr: i2c device slave address, reg_addr: target register address, data: data to send, size: required data number.
	 *
	 * @retval status of transmission, true when transfer success.
	 */
	bool SendBytes(const uint8_t slave_addr, const uint8_t reg_addr, const uint8_t* data, uint8_t size);

	/*
	 * @brief Send required number of data to i2c device with corresponding slave address and 16-bit register address
	 *
	 * @param slave_addr: i2c device slave address, reg_addr: target register address, data: data to send, size: required data number.
	 *
	 * @retval status of transmission, true when transfer success.
	 */
	bool SendBytes(const uint8_t slave_addr, const uint16_t reg_addr, const uint8_t* data, uint8_t size);

	/*
	 * @brief Get data from i2c device with corresponding slave address and 8-bit register address
	 *
	 * @param slave_addr: i2c device slave address, reg_addr: target register address, data: received data.
	 *
	 * @retval status of transmission, true when transfer success.
	 */
	bool GetByte(const uint8_t slave_addr, const uint8_t reg_addr, uint8_t& data);

	/*
	 * @brief Get data to i2c device with corresponding slave address and 16-bit register address
	 *
	 * @param slave_addr: i2c device slave address, reg_addr: target register address, data: received data.
	 *
	 * @retval status of transmission, true when transfer success.
	 */
	bool GetByte(const uint8_t slave_addr, const uint16_t reg_addr, uint8_t& data);

	/*
	 * @brief Get required number of data from i2c device with corresponding slave address and 8-bit register address
	 *
	 * @param slave_addr: i2c device slave address, reg_addr: target register address, data: received data, size: required data number.
	 *
	 * @retval status of transmission, true when transfer success.
	 */
	bool GetBytes(const uint8_t slave_addr, const uint8_t reg_addr, uint8_t* data, uint8_t size);

	/*
	 * @brief Get required number of data from i2c device with corresponding slave address and 16-bit register address
	 *
	 * @param slave_addr: i2c device slave address, reg_addr: target register address, data: received data, size: required data number.
	 *
	 * @retval status of transmission, true when transfer success.
	 */
	bool GetBytes(const uint8_t slave_addr, const uint16_t reg_addr, uint8_t* data, uint8_t size);
private:
	void SendStartCMD();
	void SendStopCMD();
	bool SendByte(const uint8_t byte);
	void GetByte(uint8_t* byte, bool last_byte = true);

	GPIO scl, sda;
	uint32_t wait_time_ns;
};

}



#endif /* INC_DRIVER_SOFT_I2C_MASTER_H_ */
