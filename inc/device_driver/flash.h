/*
 * flash.h
 *
 *  Created on: Oct 11, 2018
 *      Author: LeeChunHei
 */

#ifndef INC_DEVICE_DRIVER_FLASH_H_
#define INC_DEVICE_DRIVER_FLASH_H_

#include "driver/flexspi.h"

namespace DeviceDriver {

class Flash {
public:
	Flash();
	bool EraseSector(uint32_t sector_num);
	bool PageProgram(uint32_t addr, uint8_t* data, uint8_t size);	//real size is size + 1
	bool ReadBuffer(uint32_t addr, uint8_t* data, uint16_t size);	//real size is size + 1

private:
	static Driver::FlexSpi::Config GetFlexspiConfig();
	uint32_t CheckVendorID();
	bool EnableQuadMode();
	bool WriteEnable(uint32_t addr);
	bool WaitBusBusy();

	Driver::FlexSpi flexspi;
};

}


#endif /* INC_DEVICE_DRIVER_FLASH_H_ */
