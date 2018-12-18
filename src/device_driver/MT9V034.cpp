/*
 * MT9V034.cpp
 *
 *  Created on: Sep 15, 2018
 *      Author: LeeChunHei
 */

#include "device_driver/MT9V034.h"
#include "driver/soft_i2c_master.h"
#include "driver/hard_i2c_master.h"
#include "../config/config.h"
#include <assert.h>

namespace DeviceDriver {

__attribute__((section("NonCacheable,\"aw\",%nobits @")))            static uint8_t frame_buffer[4][360960] __attribute__((aligned(64)));

Driver::I2CMaster::Config GetI2CConfig(System::Pinout::Name sck, System::Pinout::Name sda) {
	Driver::I2CMaster::Config config;
	config.scl = sck;
	config.sda = sda;
	config.send_wait_time = 200000;
	config.recieve_wait_time = 200000;
	config.debug_enable = true;
	config.ignore_ack = true;
	config.baud_rate_Hz = 100000;
	return config;
}

Driver::Csi::Config GetCsiConfig(const MT9V034::Config& camera_config) {
	assert(MT9V034_DATABUS == 8 || MT9V034_DATABUS == 10 || MT9V034_DATABUS == 16);
	Driver::Csi::Config config;
#if MT9V034_DATABUS == 10
	config.pin_list.data0 = MT9V034_D0(camera_config.id);
	config.pin_list.data1 = MT9V034_D1(camera_config.id);
#endif
	config.pin_list.data2 = MT9V034_D2(camera_config.id);
	config.pin_list.data3 = MT9V034_D3(camera_config.id);
	config.pin_list.data4 = MT9V034_D4(camera_config.id);
	config.pin_list.data5 = MT9V034_D5(camera_config.id);
	config.pin_list.data6 = MT9V034_D6(camera_config.id);
	config.pin_list.data7 = MT9V034_D7(camera_config.id);
	config.pin_list.data8 = MT9V034_D8(camera_config.id);
	config.pin_list.data9 = MT9V034_D9(camera_config.id);
	config.pin_list.pclk = MT9V034_PCLK(camera_config.id);
	config.pin_list.vsync = MT9V034_VSYNC(camera_config.id);
	config.pin_list.hsync = MT9V034_HSYNC(camera_config.id);
	config.is_10_bit_data = (MT9V034_DATABUS == 10);
	config.bytes_per_pixel = MT9V034_DATABUS == 8 ? 1 : 2;
	config.data_bus = Driver::Csi::Config::DataBus::k8Bit;
	volatile uint16_t width = camera_config.width >> 3;
	volatile uint16_t height = camera_config.height >> 3;
	config.height = height << 3;
	config.width = width << 3;
	config.work_mode = Driver::Csi::Config::WorkMode::kGatedClockMode;
	config.line_pitch_bytes = config.width * config.bytes_per_pixel;
	config.vsync_active_low = false;
	return config;
}

inline void MT9V034::RegSet(uint8_t reg_addr, uint16_t value) {
	assert(i2c_master->SendByte(0xB8 >> 1, (uint8_t) reg_addr, value >> 8));
	assert(i2c_master->SendByte(0xB8 >> 1, (uint8_t) 0xF0, value & 0xFF));
}

MT9V034::MT9V034(const Config& config) :
		csi(GetCsiConfig(config)), width((config.width >> 3) << 3), height((config.height >> 3) << 3) {
	if (config.i2c_master) {
		i2c_master = config.i2c_master;
	} else {
		System::Pinout::Config scl_config, sda_config;
		uint8_t scl_module, sda_module;
		scl_config.pin = MT9V034_SCK(config.id);
		sda_config.pin = MT9V034_SDA(config.id);
		Driver::I2CMaster::Config i2c_config = GetI2CConfig(scl_config.pin, sda_config.pin);
		if (!System::Pinout::GetI2CSclPinConfig(scl_config, scl_module) || !System::Pinout::GetI2CSdaPinConfig(sda_config, sda_module) || (scl_module != sda_module)) {
			i2c_master = new Driver::SoftI2CMaster(i2c_config);
		} else {
			i2c_master = new Driver::SoftI2CMaster(i2c_config);
		}
	}

	uint8_t out_byte_high, out_byte_low;
	uint16_t result;
	assert(i2c_master->GetByte(0xB8 >> 1, (uint8_t) 0x00, out_byte_high));
	assert(i2c_master->GetByte(0xB8 >> 1, (uint8_t) 0xF0, out_byte_low));
	result = out_byte_high;
	result <<= 8;
	result += out_byte_low;
	assert(result == 0x1324);

//Reset control circuit
	RegSet(0x0C, 1);
	RegSet(0x0C, 0);

	//Load default
	RegSet(0x01, 0x0001);	//COL_WINDOW_START_CONTEXTA_REG
	RegSet(0x02, 0x0004);	//ROW_WINDOW_START_CONTEXTA_REG
	RegSet(0x03, 0x01E0);	//ROW_WINDOW_SIZE_CONTEXTA_REG
	RegSet(0x04, 0x02F0);	//COL_WINDOW_SIZE_CONTEXTA_REG
	RegSet(0x05, 0x005E);	//HORZ_BLANK_CONTEXTA_REG
	RegSet(0x06, 0x0039);	//VERT_BLANK_CONTEXTA_REG
	RegSet(0x07, 0x0188);	//CONTROL_MODE_REG
	RegSet(0x08, 0x0190);	//COARSE_SHUTTER_WIDTH_1_CONTEXTA
	RegSet(0x09, 0x01BD);	//COARSE_SHUTTER_WIDTH_2_CONTEXTA
	RegSet(0x0A, 0x0164);	//SHUTTER_WIDTH_CONTROL_CONTEXTA
	RegSet(0x0B, 0x01C2);	//COARSE_SHUTTER_WIDTH_TOTAL_CONTEXTA
	RegSet(0x0C, 0x0000);	//RESET_REG
	RegSet(0x0D, 0x0300);	//READ_MODE_REG
	RegSet(0x0E, 0x0000);	//READ_MODE2_REG
	RegSet(0x0F, 0x0100);	//PIXEL_OPERATION_MODE
	RegSet(0x10, 0x0040);	//RAMP_START_DELAY
	RegSet(0x11, 0x8042); //OFFSET_CONTROL
	RegSet(0x12, 0x0022); //AMP_RESET_BAR_CONTROL
	RegSet(0x13, 0x2D2E); //5T_PIXEL_RESET_CONTROL
	RegSet(0x14, 0x0E02); //4T_PIXEL_RESET_CONTROL
	RegSet(0x15, 0x0E32); //TX_CONTROL
	RegSet(0x16, 0x2802); //5T_PIXEL_SHS_CONTROL
	RegSet(0x17, 0x3E38); //4T_PIXEL_SHS_CONTROL
	RegSet(0x18, 0x3E38); //5T_PIXEL_SHR_CONTROL
	RegSet(0x19, 0x2802); //4T_PIXEL_SHR_CONTROL
	RegSet(0x1A, 0x0428); //COMPARATOR_RESET_CONTROL
	RegSet(0x1B, 0x0000); //LED_OUT_CONTROL
	RegSet(0x1C, 0x0302); //DATA_COMPRESSION
	RegSet(0x1D, 0x0040); //ANALOG_TEST_CONTROL
	RegSet(0x1E, 0x0000); //SRAM_TEST_DATA_ODD
	RegSet(0x1F, 0x0000); //SRAM_TEST_DATA_EVEN
	RegSet(0x20, 0x03C7); //BOOST_ROW_EN
	RegSet(0x21, 0x0020); //I_VLN_CONTROL
	RegSet(0x22, 0x0020); //I_VLN_AMP_CONTROL
	RegSet(0x23, 0x0010); //I_VLN_CMP_CONTROL
	RegSet(0x24, 0x001B); //I_OFFSET_CONTROL
	RegSet(0x26, 0x0004);		//I_VLN_VREF_ADC_CONTROL
	RegSet(0x27, 0x000C);		//I_VLN_STEP_CONTROL
	RegSet(0x28, 0x0010);		//I_VLN_BUF_CONTROL
	RegSet(0x29, 0x0010);		//I_MASTER_CONTROL
	RegSet(0x2A, 0x0020);		//I_VLN_AMP_60MHZ_CONTROL
	RegSet(0x2B, 0x0003);		//VREF_AMP_CONTROL
	RegSet(0x2C, 0x0004);		//VREF_ADC_CONTROL
	RegSet(0x2D, 0x0004);		//VBOOST_CONTROL
	RegSet(0x2E, 0x0007);		//V_HI_CONTROL
	RegSet(0x2F, 0x0003);		//V_LO_CONTROL
	RegSet(0x30, 0x0003);		//V_AMP_CAS_CONTROL
	RegSet(0x31, 0x001F);		//V1_CONTROL_CONTEXTA
	RegSet(0x32, 0x001A);		//V2_CONTROL_CONTEXTA
	RegSet(0x33, 0x0012);		//V3_CONTROL_CONTEXTA
	RegSet(0x34, 0x0003);		//V4_CONTROL_CONTEXTA
	RegSet(0x35, 0x0020);		//GLOBAL_GAIN_CONTEXTA_REG
	RegSet(0x36, 0x0010);		//GLOBAL_GAIN_CONTEXTB_REG
	RegSet(0x37, 0x0000);		//VOLTAGE_CONTROL
	RegSet(0x38, 0x0000);		//IDAC_VOLTAGE_MONITOR
	RegSet(0x39, 0x0025);		//V1_CONTROL_CONTEXTB
	RegSet(0x3A, 0x0020);		//V2_CONTROL_CONTEXTB
	RegSet(0x3B, 0x0003);		//V3_CONTROL_CONTEXTB
	RegSet(0x3C, 0x0003);		//V4_CONTROL_CONTEXTB
	RegSet(0x46, 0x231D);		//DARK_AVG_THRESHOLDS
	RegSet(0x47, 0x0080);		//CALIB_CONTROL_REG (AUTO)
	RegSet(0x4C, 0x0002);		//STEP_SIZE_AVG_MODE
	RegSet(0x70, 0x0000);		//ROW_NOISE_CONTROL
	RegSet(0x71, 0x002A);		//NOISE_CONSTANT
	RegSet(0x72, 0x0000);		//PIXCLK_CONTROL
	RegSet(0x7F, 0x0000);		//TEST_DATA
	RegSet(0x80, 0x04F4);		//TILE_X0_Y0
	RegSet(0x81, 0x04F4);		//TILE_X1_Y0
	RegSet(0x82, 0x04F4);		//TILE_X2_Y0
	RegSet(0x83, 0x04F4);		//TILE_X3_Y0
	RegSet(0x84, 0x04F4);		//TILE_X4_Y0
	RegSet(0x85, 0x04F4);		//TILE_X0_Y1
	RegSet(0x86, 0x04F4);		//TILE_X1_Y1
	RegSet(0x87, 0x04F4);		//TILE_X2_Y1
	RegSet(0x88, 0x04F4);		//TILE_X3_Y1
	RegSet(0x89, 0x04F4);		//TILE_X4_Y1
	RegSet(0x8A, 0x04F4);		//TILE_X0_Y2
	RegSet(0x8B, 0x04F4);		//TILE_X1_Y2
	RegSet(0x8C, 0x04F4);		//TILE_X2_Y2
	RegSet(0x8D, 0x04F4);		//TILE_X3_Y2
	RegSet(0x8E, 0x04F4);		//TILE_X4_Y2
	RegSet(0x8F, 0x04F4);		//TILE_X0_Y3
	RegSet(0x90, 0x04F4);		//TILE_X1_Y3
	RegSet(0x91, 0x04F4);		//TILE_X2_Y3
	RegSet(0x92, 0x04F4);		//TILE_X3_Y3
	RegSet(0x93, 0x04F4);		//TILE_X4_Y3
	RegSet(0x94, 0x04F4);		//TILE_X0_Y4
	RegSet(0x95, 0x04F4);		//TILE_X1_Y4
	RegSet(0x96, 0x04F4);		//TILE_X2_Y4
	RegSet(0x97, 0x04F4);		//TILE_X3_Y4
	RegSet(0x98, 0x04F4);		//TILE_X4_Y4
	RegSet(0x99, 0x0000);		//X0_SLASH5
	RegSet(0x9A, 0x0096);		//X1_SLASH5
	RegSet(0x9B, 0x012C);		//X2_SLASH5
	RegSet(0x9C, 0x01C2);		//X3_SLASH5
	RegSet(0x9D, 0x0258);		//X4_SLASH5
	RegSet(0x9E, 0x02F0);		//X5_SLASH5
	RegSet(0x9F, 0x0000);		//Y0_SLASH5
	RegSet(0xA0, 0x0060);		//Y1_SLASH5
	RegSet(0xA1, 0x00C0);		//Y2_SLASH5
	RegSet(0xA2, 0x0120);		//Y3_SLASH5
	RegSet(0xA3, 0x0180);		//Y4_SLASH5
	RegSet(0xA4, 0x01E0);		//Y5_SLASH5
	RegSet(0xA5, 0x003A);		//DESIRED_BIN
	RegSet(0xA6, 0x0002);		//EXP_SKIP_FRM_H
	RegSet(0xA8, 0x0000);		//EXP_LPF
	RegSet(0xA9, 0x0002);		//GAIN_SKIP_FRM
	RegSet(0xAA, 0x0002);		//GAIN_LPF_H
	RegSet(0xAB, 0x0040);		//MAX_GAIN
	RegSet(0xAC, 0x0001);		//MIN_COARSE_EXPOSURE
	RegSet(0xAD, 0x01E0);		//MAX_COARSE_EXPOSURE
	RegSet(0xAE, 0x0014);		//BIN_DIFF_THRESHOLD
	RegSet(0xAF, 0x0000);		//AUTO_BLOCK_CONTROL
	RegSet(0xB0, 0xABE0);		//PIXEL_COUNT
	RegSet(0xB1, 0x0002);		//LVDS_MASTER_CONTROL
	RegSet(0xB2, 0x0010);		//LVDS_SHFT_CLK_CONTROL
	RegSet(0xB3, 0x0010);		//LVDS_DATA_CONTROL
	RegSet(0xB4, 0x0000);		//LVDS_DATA_STREAM_LATENCY
	RegSet(0xB5, 0x0000);		//LVDS_INTERNAL_SYNC
	RegSet(0xB6, 0x0000);		//LVDS_USE_10BIT_PIXELS
	RegSet(0xB7, 0x0000);		//STEREO_ERROR_CONTROL
	RegSet(0xBF, 0x0016);		//INTERLACE_FIELD_VBLANK
	RegSet(0xC0, 0x000A);		//IMAGE_CAPTURE_NUM
	RegSet(0xC2, 0x18D0);		//ANALOG_CONTROLS
	RegSet(0xC3, 0x007F);		//AB_PULSE_WIDTH_REG
	RegSet(0xC4, 0x007F);		//TX_PULLUP_PULSE_WIDTH_REG
	RegSet(0xC5, 0x007F);		//RST_PULLUP_PULSE_WIDTH_REG
	RegSet(0xC6, 0x0000);		//NTSC_FV_CONTROL
	RegSet(0xC7, 0x4416);		//NTSC_HBLANK
	RegSet(0xC8, 0x4421);		//NTSC_VBLANK
	RegSet(0xC9, 0x0002);		//COL_WINDOW_START_CONTEXTB_REG
	RegSet(0xCA, 0x0004);		//ROW_WINDOW_START_CONTEXTB_REG
	RegSet(0xCB, 0x01E0);		//ROW_WINDOW_SIZE_CONTEXTB_REG
	RegSet(0xCC, 0x02EE);		//COL_WINDOW_SIZE_CONTEXTB_REG
	RegSet(0xCD, 0x0100);		//HORZ_BLANK_CONTEXTB_REG
	RegSet(0xCE, 0x0100);		//VERT_BLANK_CONTEXTB_REG
	RegSet(0xCF, 0x0190);		//COARSE_SHUTTER_WIDTH_1_CONTEXTB
	RegSet(0xD0, 0x01BD);		//COARSE_SHUTTER_WIDTH_2_CONTEXTB
	RegSet(0xD1, 0x0064);		//SHUTTER_WIDTH_CONTROL_CONTEXTB
	RegSet(0xD2, 0x01C2);		//COARSE_SHUTTER_WIDTH_TOTAL_CONTEXTB
	RegSet(0xD3, 0x0000);		//FINE_SHUTTER_WIDTH_1_CONTEXTA
	RegSet(0xD4, 0x0000);		//FINE_SHUTTER_WIDTH_2_CONTEXTA
	RegSet(0xD5, 0x0000);		//FINE_SHUTTER_WIDTH_TOTAL_CONTEXTA
	RegSet(0xD6, 0x0000);		//FINE_SHUTTER_WIDTH_1_CONTEXTB
	RegSet(0xD7, 0x0000);		//FINE_SHUTTER_WIDTH_2_CONTEXTB
	RegSet(0xD8, 0x0000);		//FINE_SHUTTER_WIDTH_TOTAL_CONTEXTB
	RegSet(0xD9, 0x0000);		//MONITOR_MODE_CONTROL

	//Set Resolution
	uint16_t data = 0;
	uint16_t width = this->width;
	uint16_t height = this->height;
	if ((width * 4) <= 752 && (height * 4) <= 480) {
		width *= 4;
		height *= 4;
		data |= 1 << 1 | 1 << 3;
	} else if ((width * 2) <= 752 && (height * 2) <= 480) {
		width *= 2;
		height *= 2;
		data |= 1 << 0 | 1 << 2;
	}
	data |= (3 << 4);       //LQ-MT9V034 needs vertical mirror to capture correct image
	RegSet(0x0D, data);
	RegSet(0x04, width);
	RegSet(0x03, height);
	RegSet(0x01, (752 - width) / 2 + 1);
	RegSet(0x02, (480 - height) / 2 + 4);
	RegSet(0xB0, width * height);
	RegSet(0x1C, 0x0303);       //0x1C  here is the way to regulate darkness :)

//	//Set AEC AGC
//	RegSet(0xAF, 0x0303);       //0xAF  AEC/AGC A~bit0:1AE;bit1:1AG/B~bit2:1AE;bit3:1AG

	switch (config.hdr_mode) {
	case Config::HDR::kDisable:
		RegSet(0x08, 0x01BB);
		RegSet(0x09, 0x01D9);
		RegSet(0x0A, 0x0164);
		RegSet(0x0B, 0x01E0);
		RegSet(0x0F, 0x0100);
		RegSet(0x35, 0x0010);
		break;

	case Config::HDR::k80dB:
		RegSet(0x08, 0x03CA);
		RegSet(0x09, 0x03DE);
		RegSet(0x0A, 0x0064);
		RegSet(0x0B, 0x03E8);
		RegSet(0x0F, 0x0103);
		RegSet(0x35, 0x8010);
		break;

	case Config::HDR::k100dB:
		RegSet(0x08, 0x03D4);
		RegSet(0x09, 0x03E7);
		RegSet(0x0A, 0x0064);
		RegSet(0x0B, 0x03E8);
		RegSet(0x0F, 0x0103);
		RegSet(0x35, 0x8010);
		break;

	default:
		break;
	}

//	RegSet(0x0B, 158);

	//Register fix
	RegSet(0x13, 0x2D2E);       //We also recommended using R0x13 = 0x2D2E with this setting for better column FPN.
	RegSet(0x20, 0x03C7);       //Recommended by design to improve performance in HDR mode and when frame rate is low.
	RegSet(0x24, 0x0010);       //Corrects pixel negative dark offset when global reset in R0x20[9] is enabled.
	RegSet(0x2B, 0x0003);       //Improves column FPN.
	RegSet(0x2F, 0x0003);       //Improves FPN at near-saturation.

	//Anti-eclipse enable
	RegSet(0xC2, 0x18D0);
	//Disable DPC
	RegSet(0x07, 0x0388);	//CONTROL_MODE_REG

////Reset control circuit
//	RegSet(0x0C, 1);
//	RegSet(0x0C, 0);
//
//	//Load default
//	RegSet(0x01, 0x0001); //COL_WINDOW_START_CONTEXTA_REG
//	RegSet(0x02, 0x0004); //ROW_WINDOW_START_CONTEXTA_REG
//	RegSet(0x03, 0x01E0); //ROW_WINDOW_SIZE_CONTEXTA_REG
//	RegSet(0x04, 0x02F0); //COL_WINDOW_SIZE_CONTEXTA_REG
//	RegSet(0x05, 0x005E); //HORZ_BLANK_CONTEXTA_REG
//	RegSet(0x06, 0x0039); //VERT_BLANK_CONTEXTA_REG
//	RegSet(0x07, 0x0188); //CONTROL_MODE_REG
//	RegSet(0x08, 0x0190); //COARSE_SHUTTER_WIDTH_1_CONTEXTA
//	RegSet(0x09, 0x01BD); //COARSE_SHUTTER_WIDTH_2_CONTEXTA
//	RegSet(0x0A, 0x0164); //SHUTTER_WIDTH_CONTROL_CONTEXTA
//	RegSet(0x0B, 0x01C2); //COARSE_SHUTTER_WIDTH_TOTAL_CONTEXTA
//	RegSet(0x0C, 0x0000); //RESET_REG
//	RegSet(0x0D, 0x0300); //READ_MODE_REG
//	RegSet(0x0E, 0x0000); //READ_MODE2_REG
//	RegSet(0x0F, 0x0100); //PIXEL_OPERATION_MODE
//	RegSet(0x10, 0x0040); //RAMP_START_DELAY
//	RegSet(0x11, 0x8042); //OFFSET_CONTROL
//	RegSet(0x12, 0x0022); //AMP_RESET_BAR_CONTROL
//	RegSet(0x13, 0x2D2E); //5T_PIXEL_RESET_CONTROL
//	RegSet(0x14, 0x0E02); //4T_PIXEL_RESET_CONTROL
//	RegSet(0x15, 0x0E32); //TX_CONTROL
//	RegSet(0x16, 0x2802); //5T_PIXEL_SHS_CONTROL
//	RegSet(0x17, 0x3E38); //4T_PIXEL_SHS_CONTROL
//	RegSet(0x18, 0x3E38); //5T_PIXEL_SHR_CONTROL
//	RegSet(0x19, 0x2802); //4T_PIXEL_SHR_CONTROL
//	RegSet(0x1A, 0x0428); //COMPARATOR_RESET_CONTROL
//	RegSet(0x1B, 0x0000); //LED_OUT_CONTROL
//	RegSet(0x1C, 0x0302); //DATA_COMPRESSION
//	RegSet(0x1D, 0x0040); //ANALOG_TEST_CONTROL
//	RegSet(0x1E, 0x0000); //SRAM_TEST_DATA_ODD
//	RegSet(0x1F, 0x0000); //SRAM_TEST_DATA_EVEN
//	RegSet(0x20, 0x03C7); //BOOST_ROW_EN
//	RegSet(0x21, 0x0020); //I_VLN_CONTROL
//	RegSet(0x22, 0x0020); //I_VLN_AMP_CONTROL
//	RegSet(0x23, 0x0010); //I_VLN_CMP_CONTROL
//	RegSet(0x24, 0x001B); //I_OFFSET_CONTROL
//	//RegSet(0x25, 0x001A);); //I_BANDGAP_CONTROL - TRIMMED PER DIE
//	RegSet(0x26, 0x0004);		//I_VLN_VREF_ADC_CONTROL
//	RegSet(0x27, 0x000C);		//I_VLN_STEP_CONTROL
//	RegSet(0x28, 0x0010);		//I_VLN_BUF_CONTROL
//	RegSet(0x29, 0x0010);		//I_MASTER_CONTROL
//	RegSet(0x2A, 0x0020);		//I_VLN_AMP_60MHZ_CONTROL
//	RegSet(0x2B, 0x0003);		//VREF_AMP_CONTROL
//	RegSet(0x2C, 0x0004);		//VREF_ADC_CONTROL
//	RegSet(0x2D, 0x0004);		//VBOOST_CONTROL
//	RegSet(0x2E, 0x0007);		//V_HI_CONTROL
//	RegSet(0x2F, 0x0003);		//V_LO_CONTROL
//	RegSet(0x30, 0x0003);		//V_AMP_CAS_CONTROL
//	RegSet(0x31, 0x001F);		//V1_CONTROL_CONTEXTA
//	RegSet(0x32, 0x001A);		//V2_CONTROL_CONTEXTA
//	RegSet(0x33, 0x0012);		//V3_CONTROL_CONTEXTA
//	RegSet(0x34, 0x0003);		//V4_CONTROL_CONTEXTA
//	RegSet(0x35, 0x0020);		//GLOBAL_GAIN_CONTEXTA_REG
//	RegSet(0x36, 0x0010);		//GLOBAL_GAIN_CONTEXTB_REG
//	RegSet(0x37, 0x0000);		//VOLTAGE_CONTROL
//	RegSet(0x38, 0x0000);		//IDAC_VOLTAGE_MONITOR
//	RegSet(0x39, 0x0025);		//V1_CONTROL_CONTEXTB
//	RegSet(0x3A, 0x0020);		//V2_CONTROL_CONTEXTB
//	RegSet(0x3B, 0x0003);		//V3_CONTROL_CONTEXTB
//	RegSet(0x3C, 0x0003);		//V4_CONTROL_CONTEXTB
//	RegSet(0x46, 0x231D);		//DARK_AVG_THRESHOLDS
//	RegSet(0x47, 0x0080);		//CALIB_CONTROL_REG (AUTO)
//	RegSet(0x4C, 0x0002);		//STEP_SIZE_AVG_MODE
//	RegSet(0x70, 0x0000);		//ROW_NOISE_CONTROL
//	RegSet(0x71, 0x002A);		//NOISE_CONSTANT
//	RegSet(0x72, 0x0000);		//PIXCLK_CONTROL
//	RegSet(0x7F, 0x0000);		//TEST_DATA
//	RegSet(0x80, 0x04F4);		//TILE_X0_Y0
//	RegSet(0x81, 0x04F4);		//TILE_X1_Y0
//	RegSet(0x82, 0x04F4);		//TILE_X2_Y0
//	RegSet(0x83, 0x04F4);		//TILE_X3_Y0
//	RegSet(0x84, 0x04F4);		//TILE_X4_Y0
//	RegSet(0x85, 0x04F4);		//TILE_X0_Y1
//	RegSet(0x86, 0x04F4);		//TILE_X1_Y1
//	RegSet(0x87, 0x04F4);		//TILE_X2_Y1
//	RegSet(0x88, 0x04F4);		//TILE_X3_Y1
//	RegSet(0x89, 0x04F4);		//TILE_X4_Y1
//	RegSet(0x8A, 0x04F4);		//TILE_X0_Y2
//	RegSet(0x8B, 0x04F4);		//TILE_X1_Y2
//	RegSet(0x8C, 0x04F4);		//TILE_X2_Y2
//	RegSet(0x8D, 0x04F4);		//TILE_X3_Y2
//	RegSet(0x8E, 0x04F4);		//TILE_X4_Y2
//	RegSet(0x8F, 0x04F4);		//TILE_X0_Y3
//	RegSet(0x90, 0x04F4);		//TILE_X1_Y3
//	RegSet(0x91, 0x04F4);		//TILE_X2_Y3
//	RegSet(0x92, 0x04F4);		//TILE_X3_Y3
//	RegSet(0x93, 0x04F4);		//TILE_X4_Y3
//	RegSet(0x94, 0x04F4);		//TILE_X0_Y4
//	RegSet(0x95, 0x04F4);		//TILE_X1_Y4
//	RegSet(0x96, 0x04F4);		//TILE_X2_Y4
//	RegSet(0x97, 0x04F4);		//TILE_X3_Y4
//	RegSet(0x98, 0x04F4);		//TILE_X4_Y4
//	RegSet(0x99, 0x0000);		//X0_SLASH5
//	RegSet(0x9A, 0x0096);		//X1_SLASH5
//	RegSet(0x9B, 0x012C);		//X2_SLASH5
//	RegSet(0x9C, 0x01C2);		//X3_SLASH5
//	RegSet(0x9D, 0x0258);		//X4_SLASH5
//	RegSet(0x9E, 0x02F0);		//X5_SLASH5
//	RegSet(0x9F, 0x0000);		//Y0_SLASH5
//	RegSet(0xA0, 0x0060);		//Y1_SLASH5
//	RegSet(0xA1, 0x00C0);		//Y2_SLASH5
//	RegSet(0xA2, 0x0120);		//Y3_SLASH5
//	RegSet(0xA3, 0x0180);		//Y4_SLASH5
//	RegSet(0xA4, 0x01E0);		//Y5_SLASH5
//	RegSet(0xA5, 0x003A);		//DESIRED_BIN
//	RegSet(0xA6, 0x0002);		//EXP_SKIP_FRM_H
//	RegSet(0xA8, 0x0000);		//EXP_LPF
//	RegSet(0xA9, 0x0002);		//GAIN_SKIP_FRM
//	RegSet(0xAA, 0x0002);		//GAIN_LPF_H
//	RegSet(0xAB, 0x0040);		//MAX_GAIN
//	RegSet(0xAC, 0x0001);		//MIN_COARSE_EXPOSURE
//	RegSet(0xAD, 0x01E0);		//MAX_COARSE_EXPOSURE
//	RegSet(0xAE, 0x0014);		//BIN_DIFF_THRESHOLD
//	RegSet(0xAF, 0x0000);		//AUTO_BLOCK_CONTROL
//	RegSet(0xB0, 0xABE0);		//PIXEL_COUNT
//	RegSet(0xB1, 0x0002);		//LVDS_MASTER_CONTROL
//	RegSet(0xB2, 0x0010);		//LVDS_SHFT_CLK_CONTROL
//	RegSet(0xB3, 0x0010);		//LVDS_DATA_CONTROL
//	RegSet(0xB4, 0x0000);		//LVDS_DATA_STREAM_LATENCY
//	RegSet(0xB5, 0x0000);		//LVDS_INTERNAL_SYNC
//	RegSet(0xB6, 0x0000);		//LVDS_USE_10BIT_PIXELS
//	RegSet(0xB7, 0x0000);		//STEREO_ERROR_CONTROL
//	RegSet(0xBF, 0x0016);		//INTERLACE_FIELD_VBLANK
//	RegSet(0xC0, 0x000A);		//IMAGE_CAPTURE_NUM
//	RegSet(0xC2, 0x18D0);		//ANALOG_CONTROLS
//	RegSet(0xC3, 0x007F);		//AB_PULSE_WIDTH_REG
//	RegSet(0xC4, 0x007F);		//TX_PULLUP_PULSE_WIDTH_REG
//	RegSet(0xC5, 0x007F);		//RST_PULLUP_PULSE_WIDTH_REG
//	RegSet(0xC6, 0x0000);		//NTSC_FV_CONTROL
//	RegSet(0xC7, 0x4416);		//NTSC_HBLANK
//	RegSet(0xC8, 0x4421);		//NTSC_VBLANK
//	RegSet(0xC9, 0x0002);		//COL_WINDOW_START_CONTEXTB_REG
//	RegSet(0xCA, 0x0004);		//ROW_WINDOW_START_CONTEXTB_REG
//	RegSet(0xCB, 0x01E0);		//ROW_WINDOW_SIZE_CONTEXTB_REG
//	RegSet(0xCC, 0x02EE);		//COL_WINDOW_SIZE_CONTEXTB_REG
//	RegSet(0xCD, 0x0100);		//HORZ_BLANK_CONTEXTB_REG
//	RegSet(0xCE, 0x0100);		//VERT_BLANK_CONTEXTB_REG
//	RegSet(0xCF, 0x0190);		//COARSE_SHUTTER_WIDTH_1_CONTEXTB
//	RegSet(0xD0, 0x01BD);		//COARSE_SHUTTER_WIDTH_2_CONTEXTB
//	RegSet(0xD1, 0x0064);		//SHUTTER_WIDTH_CONTROL_CONTEXTB
//	RegSet(0xD2, 0x01C2);		//COARSE_SHUTTER_WIDTH_TOTAL_CONTEXTB
//	RegSet(0xD3, 0x0000);		//FINE_SHUTTER_WIDTH_1_CONTEXTA
//	RegSet(0xD4, 0x0000);		//FINE_SHUTTER_WIDTH_2_CONTEXTA
//	RegSet(0xD5, 0x0000);		//FINE_SHUTTER_WIDTH_TOTAL_CONTEXTA
//	RegSet(0xD6, 0x0000);		//FINE_SHUTTER_WIDTH_1_CONTEXTB
//	RegSet(0xD7, 0x0000);		//FINE_SHUTTER_WIDTH_2_CONTEXTB
//	RegSet(0xD8, 0x0000);		//FINE_SHUTTER_WIDTH_TOTAL_CONTEXTB
//	RegSet(0xD9, 0x0000);		//MONITOR_MODE_CONTROL
//
//	RegSet(0xAC, 0x0001);
//	RegSet(0xAD, 0x01E0);
//	RegSet(0x2C, 0x0004);
//
//	RegSet(0x0F, 0x0000);
//	RegSet(0x0F, 0x0101);		// 0x0F bit8:1HDR,0linear; bit1:1color,0gray;bit0:1HDR,0linear
//	RegSet(0x07, 0x0188);		//Context A
//	RegSet(0x70, 0);		//0x70  0x0000
//	RegSet(0xAF, 0x0302);		//0xAF  AEC/AGC A~bit0:1AE;bit1:1AG/B~bit2:1AE;bit3:1AG
//
//	RegSet(0xAC, 0x0001);		//0xAC  min fine width   0x0001
//	RegSet(0xAD, 0x01E0);		//0xAD  max fine width   0x01E0-480
//	RegSet(0xAB, 50);		//0xAB  max analog gain     64
//
//	RegSet(0xB0, 188 * 120);
//	RegSet(0x1C, 0x0303);		//0x1C  here is the way to regulate darkness :)
//
//	RegSet(0x13, 0x2D2E);		//We also recommended using R0x13 = 0x2D2E with this setting for better column FPN.
//	RegSet(0x20, 0x03C7);		//Recommended by design to improve performance in HDR mode and when frame rate is low.
//	RegSet(0x24, 0x0010);		//Corrects pixel negative dark offset when global reset in R0x20[9] is enabled.
//	RegSet(0x2B, 0x0003);		//Improves column FPN.
//	RegSet(0x2F, 0x0003);		//Improves FPN at near-saturation.
//
//	//100DB
//	RegSet(0x08, 0x01BB);		//0x08 Coarse Shutter IMAGEW 1
//	RegSet(0x09, 0x01D9);		//0x09 Coarse Shutter IMAGEW 2
//	RegSet(0x0A, 0x0164);		//0x0A Coarse Shutter IMAGEW Control
//	RegSet(0x32, 0x001A);		//0x32   0x001A
//	RegSet(0x0B, 158);		//0x0B Coarse Shutter IMAGEW Total
//	RegSet(0x0F, 0x0103);		//0x0F High Dynamic Range enable,bit is set (R0x0F[1]=1), the sensor uses black level correction values from one green plane, which are applied to all colors.
//	RegSet(0xA5, 60);		//0xA5  image brightness  50  1-64
//	RegSet(0x35, 0x8010);		//0x35
//	RegSet(0x06, 0x0002);
//
//	//Anti-eclipse enable
//	RegSet(0xC2, 0x18D0);
//
//	//Register Fix
//	RegSet(0x20, 0x03C7);
//	i2c_master.SendByte(0xB8 >> 1, (uint8_t) 0x24, 0x001B >> 8);
//	i2c_master.SendByte(0xB8 >> 1, (uint8_t) 0xF0, 0x001B & 0b11111111);
//	i2c_master.SendByte(0xB8 >> 1, (uint8_t) 0x2B, 0);
//	i2c_master.SendByte(0xB8 >> 1, (uint8_t) 0xF0, 3);
//	i2c_master.SendByte(0xB8 >> 1, (uint8_t) 0x2F, 0);
//	i2c_master.SendByte(0xB8 >> 1, (uint8_t) 0xF0, 3);
//	RegSet(0x13, 0x2D2E);
//
//	//Set Resolution
//	uint16_t data = 0;
//	uint16_t width = this->width;
//	uint16_t height = this->height;
//	if ((width * 4) <= 752 && (height * 4) <= 480) {
//		width *= 4;
//		height *= 4;
//		data |= 1 << 1 | 1 << 3;
//	} else if ((width * 2) <= 752 && (height * 2) <= 480) {
//		width *= 2;
//		height *= 2;
//		data |= 1 << 0 | 1 << 2;
//	}
//	data |= (3 << 4);       //LQ-MT9V034 needs vertical mirror to capture correct image
//	RegSet(0x0D, data);
//	RegSet(0x04, width);
//	RegSet(0x03, height);
//	RegSet(0x01, (752 - width) / 2 + 1);
//	RegSet(0x02, (480 - height) / 2 + 4);
//	RegSet(0xB0, width * height);
//
//	RegSet(0x0C, 0x03);       //0x0c  ¸´Î»

	csi.ConfigTransferBuffer(0, (uint32_t) frame_buffer[0]);
	csi.ConfigTransferBuffer(1, (uint32_t) frame_buffer[1]);
	empty_buffer_addr.push((uint32_t) frame_buffer[2]);
	empty_buffer_addr.push((uint32_t) frame_buffer[3]);
	csi.SetListener([&](Driver::Csi* csi_ptr) {uint8_t state = csi_ptr->IsTransferComplete();
		if (!empty_buffer_addr.size()&&ready_buffer_addr.size()==3) {
			csi_ptr->Stop();
			is_started = false;
		}
		if (state & 1) {
			ready_buffer_addr.push(csi_ptr->GetTransferBuffer(0));
			if (empty_buffer_addr.size()) {
				csi_ptr->ConfigTransferBuffer(0, empty_buffer_addr.front());
				empty_buffer_addr.pop();
			}
		}
		if (state & 2) {
			ready_buffer_addr.push(csi_ptr->GetTransferBuffer(1));
			if (empty_buffer_addr.size()) {
				csi_ptr->ConfigTransferBuffer(1, empty_buffer_addr.front());
				empty_buffer_addr.pop();
			}
		}}, 5);
}

void MT9V034::Start() {
	is_started = true;
	csi.Start();
}

void MT9V034::Stop() {
	is_started = false;
	csi.Stop();
	uint32_t first_addr = ready_buffer_addr.front();
	ready_buffer_addr.pop();
	while (ready_buffer_addr.size()) {
		empty_buffer_addr.push(ready_buffer_addr.front());
		ready_buffer_addr.pop();
	}
	ready_buffer_addr.push(first_addr);
}

void MT9V034::UnlockBuffer() {
	empty_buffer_addr.push(ready_buffer_addr.front());
	ready_buffer_addr.pop();
	if (!is_started && empty_buffer_addr.size() == 2) {
		csi.ConfigTransferBuffer(0, empty_buffer_addr.front());
		empty_buffer_addr.pop();
		csi.ConfigTransferBuffer(1, empty_buffer_addr.front());
		empty_buffer_addr.pop();
		Start();
	}
}

//inline void MT9V034::CsiListener(Driver::Csi* csi_ptr) {
//	uint8_t state = csi_ptr->IsTransferComplete();
//	if (state & 1) {
//		ready_buffer_addr.push(csi_ptr->GetTransferBuffer(0));
//		if (empty_buffer_addr.size()) {
//			csi_ptr->ConfigTransferBuffer(0, empty_buffer_addr.front());
//			empty_buffer_addr.pop();
//		}
//	}
//	if (state & 2) {
//		ready_buffer_addr.push(csi_ptr->GetTransferBuffer(1));
//		if (empty_buffer_addr.size()) {
//			csi_ptr->ConfigTransferBuffer(0, empty_buffer_addr.front());
//			empty_buffer_addr.pop();
//		}
//	}
//	if (!empty_buffer_addr.size()) {
//		csi_ptr->Stop();
//		is_started = false;
//	}
//}

}
