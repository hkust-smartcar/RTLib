/*
 * mpu6050.cpp
 *
 *  Created on: Sep 21, 2018
 *      Author: LeeChunHei
 */

#include "device_driver/mpu6050.h"
#include "driver/hard_i2c_master.h"
#include "../config/config.h"
#include "system/systick.h"
#include <cmath>
#include <vector>
#include <assert.h>

namespace DeviceDriver {

#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

#define MPU6050_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_FF_THR           0x1D
#define MPU6050_RA_FF_DUR           0x1E
#define MPU6050_RA_MOT_THR          0x1F
#define MPU6050_RA_MOT_DUR          0x20
#define MPU6050_RA_ZRMOT_THR        0x21
#define MPU6050_RA_ZRMOT_DUR        0x22
#define MPU6050_RA_FIFO_EN          0x23
#define MPU6050_RA_I2C_MST_CTRL     0x24
#define MPU6050_RA_I2C_SLV0_ADDR    0x25
#define MPU6050_RA_I2C_SLV0_REG     0x26
#define MPU6050_RA_I2C_SLV0_CTRL    0x27
#define MPU6050_RA_I2C_SLV1_ADDR    0x28
#define MPU6050_RA_I2C_SLV1_REG     0x29
#define MPU6050_RA_I2C_SLV1_CTRL    0x2A
#define MPU6050_RA_I2C_SLV2_ADDR    0x2B
#define MPU6050_RA_I2C_SLV2_REG     0x2C
#define MPU6050_RA_I2C_SLV2_CTRL    0x2D
#define MPU6050_RA_I2C_SLV3_ADDR    0x2E
#define MPU6050_RA_I2C_SLV3_REG     0x2F
#define MPU6050_RA_I2C_SLV3_CTRL    0x30
#define MPU6050_RA_I2C_SLV4_ADDR    0x31
#define MPU6050_RA_I2C_SLV4_REG     0x32
#define MPU6050_RA_I2C_SLV4_DO      0x33
#define MPU6050_RA_I2C_SLV4_CTRL    0x34
#define MPU6050_RA_I2C_SLV4_DI      0x35
#define MPU6050_RA_I2C_MST_STATUS   0x36
#define MPU6050_RA_INT_PIN_CFG      0x37
#define MPU6050_RA_INT_ENABLE       0x38
#define MPU6050_RA_DMP_INT_STATUS   0x39
#define MPU6050_RA_INT_STATUS       0x3A
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48
#define MPU6050_RA_EXT_SENS_DATA_00 0x49
#define MPU6050_RA_EXT_SENS_DATA_01 0x4A
#define MPU6050_RA_EXT_SENS_DATA_02 0x4B
#define MPU6050_RA_EXT_SENS_DATA_03 0x4C
#define MPU6050_RA_EXT_SENS_DATA_04 0x4D
#define MPU6050_RA_EXT_SENS_DATA_05 0x4E
#define MPU6050_RA_EXT_SENS_DATA_06 0x4F
#define MPU6050_RA_EXT_SENS_DATA_07 0x50
#define MPU6050_RA_EXT_SENS_DATA_08 0x51
#define MPU6050_RA_EXT_SENS_DATA_09 0x52
#define MPU6050_RA_EXT_SENS_DATA_10 0x53
#define MPU6050_RA_EXT_SENS_DATA_11 0x54
#define MPU6050_RA_EXT_SENS_DATA_12 0x55
#define MPU6050_RA_EXT_SENS_DATA_13 0x56
#define MPU6050_RA_EXT_SENS_DATA_14 0x57
#define MPU6050_RA_EXT_SENS_DATA_15 0x58
#define MPU6050_RA_EXT_SENS_DATA_16 0x59
#define MPU6050_RA_EXT_SENS_DATA_17 0x5A
#define MPU6050_RA_EXT_SENS_DATA_18 0x5B
#define MPU6050_RA_EXT_SENS_DATA_19 0x5C
#define MPU6050_RA_EXT_SENS_DATA_20 0x5D
#define MPU6050_RA_EXT_SENS_DATA_21 0x5E
#define MPU6050_RA_EXT_SENS_DATA_22 0x5F
#define MPU6050_RA_EXT_SENS_DATA_23 0x60
#define MPU6050_RA_MOT_DETECT_STATUS    0x61
#define MPU6050_RA_I2C_SLV0_DO      0x63
#define MPU6050_RA_I2C_SLV1_DO      0x64
#define MPU6050_RA_I2C_SLV2_DO      0x65
#define MPU6050_RA_I2C_SLV3_DO      0x66
#define MPU6050_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU6050_RA_SIGNAL_PATH_RESET    0x68
#define MPU6050_RA_MOT_DETECT_CTRL      0x69
#define MPU6050_RA_USER_CTRL        0x6A
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C
#define MPU6050_RA_BANK_SEL         0x6D
#define MPU6050_RA_MEM_START_ADDR   0x6E
#define MPU6050_RA_MEM_R_W          0x6F
#define MPU6050_RA_DMP_CFG_1        0x70
#define MPU6050_RA_DMP_CFG_2        0x71
#define MPU6050_RA_FIFO_COUNTH      0x72
#define MPU6050_RA_FIFO_COUNTL      0x73
#define MPU6050_RA_FIFO_R_W         0x74
#define MPU6050_RA_WHO_AM_I         0x75

#define MPU6050_TC_PWR_MODE_SHIFT     7
#define MPU6050_TC_OFFSET_SHIFT       6
#define MPU6050_TC_OFFSET_LENGTH    6
#define MPU6050_TC_OTP_BNK_VLD_SHIFT  0

#define MPU6050_VDDIO_LEVEL_VLOGIC  0
#define MPU6050_VDDIO_LEVEL_VDD     1

#define MPU6050_CFG_EXT_SYNC_SET_SHIFT    5
#define MPU6050_CFG_EXT_SYNC_SET_LENGTH 3
#define MPU6050_CFG_DLPF_CFG_SHIFT    2
#define MPU6050_CFG_DLPF_CFG_LENGTH 3

#define MPU6050_EXT_SYNC_DISABLED       0x0
#define MPU6050_EXT_SYNC_TEMP_OUT_L     0x1
#define MPU6050_EXT_SYNC_GYRO_XOUT_L    0x2
#define MPU6050_EXT_SYNC_GYRO_YOUT_L    0x3
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L    0x4
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L   0x5
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L   0x6
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L   0x7

#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

#define MPU6050_GCONFIG_FS_SEL_SHIFT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_ACONFIG_XA_ST_SHIFT           7
#define MPU6050_ACONFIG_YA_ST_SHIFT           6
#define MPU6050_ACONFIG_ZA_ST_SHIFT           5
#define MPU6050_ACONFIG_AFS_SEL_SHIFT         4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      2
#define MPU6050_ACONFIG_ACCEL_HPF_SHIFT       2
#define MPU6050_ACONFIG_ACCEL_HPF_LENGTH    3

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

#define MPU6050_DHPF_RESET          0x00
#define MPU6050_DHPF_5              0x01
#define MPU6050_DHPF_2P5            0x02
#define MPU6050_DHPF_1P25           0x03
#define MPU6050_DHPF_0P63           0x04
#define MPU6050_DHPF_HOLD           0x07

#define MPU6050_TEMP_FIFO_EN_SHIFT    7
#define MPU6050_XG_FIFO_EN_SHIFT      6
#define MPU6050_YG_FIFO_EN_SHIFT      5
#define MPU6050_ZG_FIFO_EN_SHIFT      4
#define MPU6050_ACCEL_FIFO_EN_SHIFT   3
#define MPU6050_SLV2_FIFO_EN_SHIFT    2
#define MPU6050_SLV1_FIFO_EN_SHIFT    1
#define MPU6050_SLV0_FIFO_EN_SHIFT    0

#define MPU6050_MULT_MST_EN_SHIFT     7
#define MPU6050_WAIT_FOR_ES_SHIFT     6
#define MPU6050_SLV_3_FIFO_EN_SHIFT   5
#define MPU6050_I2C_MST_P_NSR_SHIFT   4
#define MPU6050_I2C_MST_CLK_SHIFT     3
#define MPU6050_I2C_MST_CLK_LENGTH  4

#define MPU6050_CLOCK_DIV_348       0x0
#define MPU6050_CLOCK_DIV_333       0x1
#define MPU6050_CLOCK_DIV_320       0x2
#define MPU6050_CLOCK_DIV_308       0x3
#define MPU6050_CLOCK_DIV_296       0x4
#define MPU6050_CLOCK_DIV_286       0x5
#define MPU6050_CLOCK_DIV_276       0x6
#define MPU6050_CLOCK_DIV_267       0x7
#define MPU6050_CLOCK_DIV_258       0x8
#define MPU6050_CLOCK_DIV_500       0x9
#define MPU6050_CLOCK_DIV_471       0xA
#define MPU6050_CLOCK_DIV_444       0xB
#define MPU6050_CLOCK_DIV_421       0xC
#define MPU6050_CLOCK_DIV_400       0xD
#define MPU6050_CLOCK_DIV_381       0xE
#define MPU6050_CLOCK_DIV_364       0xF

#define MPU6050_I2C_SLV_RW_SHIFT      7
#define MPU6050_I2C_SLV_ADDR_SHIFT    6
#define MPU6050_I2C_SLV_ADDR_LENGTH 7
#define MPU6050_I2C_SLV_EN_SHIFT      7
#define MPU6050_I2C_SLV_BYTE_SW_SHIFT 6
#define MPU6050_I2C_SLV_REG_DIS_SHIFT 5
#define MPU6050_I2C_SLV_GRP_SHIFT     4
#define MPU6050_I2C_SLV_LEN_SHIFT     3
#define MPU6050_I2C_SLV_LEN_LENGTH  4

#define MPU6050_I2C_SLV4_RW_SHIFT         7
#define MPU6050_I2C_SLV4_ADDR_SHIFT       6
#define MPU6050_I2C_SLV4_ADDR_LENGTH    7
#define MPU6050_I2C_SLV4_EN_SHIFT         7
#define MPU6050_I2C_SLV4_INT_EN_SHIFT     6
#define MPU6050_I2C_SLV4_REG_DIS_SHIFT    5
#define MPU6050_I2C_SLV4_MST_DLY_SHIFT    4
#define MPU6050_I2C_SLV4_MST_DLY_LENGTH 5

#define MPU6050_MST_PASS_THROUGH_SHIFT    7
#define MPU6050_MST_I2C_SLV4_DONE_SHIFT   6
#define MPU6050_MST_I2C_LOST_ARB_SHIFT    5
#define MPU6050_MST_I2C_SLV4_NACK_SHIFT   4
#define MPU6050_MST_I2C_SLV3_NACK_SHIFT   3
#define MPU6050_MST_I2C_SLV2_NACK_SHIFT   2
#define MPU6050_MST_I2C_SLV1_NACK_SHIFT   1
#define MPU6050_MST_I2C_SLV0_NACK_SHIFT   0

#define MPU6050_INTCFG_INT_LEVEL_SHIFT        7
#define MPU6050_INTCFG_INT_OPEN_SHIFT         6
#define MPU6050_INTCFG_LATCH_INT_EN_SHIFT     5
#define MPU6050_INTCFG_INT_RD_CLEAR_SHIFT     4
#define MPU6050_INTCFG_FSYNC_INT_LEVEL_SHIFT  3
#define MPU6050_INTCFG_FSYNC_INT_EN_SHIFT     2
#define MPU6050_INTCFG_I2C_BYPASS_EN_SHIFT    1
#define MPU6050_INTCFG_CLKOUT_EN_SHIFT        0

#define MPU6050_INTMODE_ACTIVEHIGH  0x00
#define MPU6050_INTMODE_ACTIVELOW   0x01

#define MPU6050_INTDRV_PUSHPULL     0x00
#define MPU6050_INTDRV_OPENDRAIN    0x01

#define MPU6050_INTLATCH_50USPULSE  0x00
#define MPU6050_INTLATCH_WAITCLEAR  0x01

#define MPU6050_INTCLEAR_STATUSREAD 0x00
#define MPU6050_INTCLEAR_ANYREAD    0x01

#define MPU6050_INTERRUPT_FF_SHIFT            7
#define MPU6050_INTERRUPT_MOT_SHIFT           6
#define MPU6050_INTERRUPT_ZMOT_SHIFT          5
#define MPU6050_INTERRUPT_FIFO_OFLOW_SHIFT    4
#define MPU6050_INTERRUPT_I2C_MST_INT_SHIFT   3
#define MPU6050_INTERRUPT_PLL_RDY_INT_SHIFT   2
#define MPU6050_INTERRUPT_DMP_INT_SHIFT       1
#define MPU6050_INTERRUPT_DATA_RDY_SHIFT      0

#define MPU6050_DMPINT_5_SHIFT            5
#define MPU6050_DMPINT_4_SHIFT            4
#define MPU6050_DMPINT_3_SHIFT            3
#define MPU6050_DMPINT_2_SHIFT            2
#define MPU6050_DMPINT_1_SHIFT            1
#define MPU6050_DMPINT_0_SHIFT            0

#define MPU6050_MOTION_MOT_XNEG_SHIFT     7
#define MPU6050_MOTION_MOT_XPOS_SHIFT     6
#define MPU6050_MOTION_MOT_YNEG_SHIFT     5
#define MPU6050_MOTION_MOT_YPOS_SHIFT     4
#define MPU6050_MOTION_MOT_ZNEG_SHIFT     3
#define MPU6050_MOTION_MOT_ZPOS_SHIFT     2
#define MPU6050_MOTION_MOT_ZRMOT_SHIFT    0

#define MPU6050_DELAYCTRL_DELAY_ES_SHADOW_SHIFT   7
#define MPU6050_DELAYCTRL_I2C_SLV4_DLY_EN_SHIFT   4
#define MPU6050_DELAYCTRL_I2C_SLV3_DLY_EN_SHIFT   3
#define MPU6050_DELAYCTRL_I2C_SLV2_DLY_EN_SHIFT   2
#define MPU6050_DELAYCTRL_I2C_SLV1_DLY_EN_SHIFT   1
#define MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_SHIFT   0

#define MPU6050_PATHRESET_GYRO_RESET_SHIFT    2
#define MPU6050_PATHRESET_ACCEL_RESET_SHIFT   1
#define MPU6050_PATHRESET_TEMP_RESET_SHIFT    0

#define MPU6050_DETECT_ACCEL_ON_DELAY_SHIFT       5
#define MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH    2
#define MPU6050_DETECT_FF_COUNT_SHIFT             3
#define MPU6050_DETECT_FF_COUNT_LENGTH          2
#define MPU6050_DETECT_MOT_COUNT_SHIFT            1
#define MPU6050_DETECT_MOT_COUNT_LENGTH         2

#define MPU6050_DETECT_DECREMENT_RESET  0x0
#define MPU6050_DETECT_DECREMENT_1      0x1
#define MPU6050_DETECT_DECREMENT_2      0x2
#define MPU6050_DETECT_DECREMENT_4      0x3

#define MPU6050_USERCTRL_DMP_EN_SHIFT             7
#define MPU6050_USERCTRL_FIFO_EN_SHIFT            6
#define MPU6050_USERCTRL_I2C_MST_EN_SHIFT         5
#define MPU6050_USERCTRL_I2C_IF_DIS_SHIFT         4
#define MPU6050_USERCTRL_DMP_RESET_SHIFT          3
#define MPU6050_USERCTRL_FIFO_RESET_SHIFT         2
#define MPU6050_USERCTRL_I2C_MST_RESET_SHIFT      1
#define MPU6050_USERCTRL_SIG_COND_RESET_SHIFT     0

#define MPU6050_PWR1_DEVICE_RESET_SHIFT   7
#define MPU6050_PWR1_SLEEP_SHIFT          6
#define MPU6050_PWR1_CYCLE_SHIFT          5
#define MPU6050_PWR1_TEMP_DIS_SHIFT       3
#define MPU6050_PWR1_CLKSEL_MASK          0x7u
#define MPU6050_PWR1_CLKSEL_SHIFT         0
#define MPU6050_PWR1_CLKSEL(x)            (((uint8_t)(((uint8_t)(x))<<MPU6050_PWR1_CLKSEL_SHIFT))&MPU6050_PWR1_CLKSEL_MASK)

#define MPU6050_PWR2_LP_WAKE_CTRL_SHIFT       7
#define MPU6050_PWR2_LP_WAKE_CTRL_LENGTH    2
#define MPU6050_PWR2_STBY_XA_SHIFT            5
#define MPU6050_PWR2_STBY_YA_SHIFT            4
#define MPU6050_PWR2_STBY_ZA_SHIFT            3
#define MPU6050_PWR2_STBY_XG_SHIFT            2
#define MPU6050_PWR2_STBY_YG_SHIFT            1
#define MPU6050_PWR2_STBY_ZG_SHIFT            0

#define MPU6050_WAKE_FREQ_1P25      0x0
#define MPU6050_WAKE_FREQ_2P5       0x1
#define MPU6050_WAKE_FREQ_5         0x2
#define MPU6050_WAKE_FREQ_10        0x3

#define MPU6050_BANKSEL_PRFTCH_EN_SHIFT       6
#define MPU6050_BANKSEL_CFG_USER_BANK_SHIFT   5
#define MPU6050_BANKSEL_MEM_SEL_SHIFT         4
#define MPU6050_BANKSEL_MEM_SEL_LENGTH      5

#define MPU6050_WHO_AM_I_MASK           0x7Eu
#define MPU6050_WHO_AM_I_SHIFT          1
#define MPU6050_WHO_AM_I(x)             (((uint8_t)(((uint8_t)(x))<<MPU6050_WHO_AM_I_SHIFT))&MPU6050_WHO_AM_I_MASK)

#define MPU6050_DMP_MEMORY_BANKS        8
#define MPU6050_DMP_MEMORY_BANK_SIZE    256
#define MPU6050_DMP_MEMORY_CHUNK_SIZE   16

Driver::I2CMaster::Config MPUGetI2CConfig(System::Pinout::Name sck, System::Pinout::Name sda) {
	Driver::I2CMaster::Config config;
	config.scl = sck;
	config.sda = sda;
	config.send_wait_time = 200000;
	config.recieve_wait_time = 200000;
	config.baud_rate_Hz = 400000;
	return config;
}

MPU6050::MPU6050(Config& config) :
		temp(0.0f), is_calibrated(false), gyro_range(config.gyro_range), accel_range(config.accel_range), gyro_scale_factor(GetGyroScaleFactor()), accel_scale_factor(GetAccelScaleFactor()) {
	if (config.i2c_master) {
		i2c_master = config.i2c_master;
	} else {
		i2c_master = new Driver::HardI2CMaster(MPUGetI2CConfig(MPU6050_SCK(config.id), MPU6050_SDA(config.id)));
	}

	assert(Verify());
	System::Systick::DelayUS(1);

	assert(i2c_master->SendByte(MPU6050_DEFAULT_ADDRESS, (uint8_t) MPU6050_RA_PWR_MGMT_1, 0x03));
	System::Systick::DelayUS(1);

	//Register 25 鈥� Sample Rate Divider: Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
	//Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or 7)
	assert(i2c_master->SendByte(MPU6050_DEFAULT_ADDRESS, (uint8_t) MPU6050_RA_SMPLRT_DIV, 0x01));
	System::Systick::DelayUS(1);

	//Register 26 - CONFIG: EXT_SYNC_SET[2:0]<<3 | DLPF_CFG[2:0];
	//EXT_SYNC_SET=0, Input disabled;
	//DLPF_CFG=0, Accel = 260Hz, Gyroscope = 256Hz;
	assert(i2c_master->SendByte(MPU6050_DEFAULT_ADDRESS, (uint8_t) MPU6050_RA_CONFIG, 0x00));
	System::Systick::DelayUS(1);

	//Register 27 - GYRO_CONFIG: FS_SEL[1:0] << 3;
	//FS_SEL=0, 卤 250 掳/s; FS_SEL=1, 卤 500 掳/s; FS_SEL=2, 卤 1000 掳/s; FS_SEL=3, 卤 2000 掳/s;
	uint8_t gyro_config = static_cast<int>(gyro_range) << 3;
	assert(i2c_master->SendByte(MPU6050_DEFAULT_ADDRESS, (uint8_t) MPU6050_RA_GYRO_CONFIG, gyro_config));
	System::Systick::DelayUS(1);

	//Register 28 - ACCEL_CONFIG: AFS_SEL[1:0] << 3;
	//AFS_SEL=0, 卤2g; AFS_SEL=1, 卤4g; AFS_SEL=2, 卤8g; AFS_SEL=3, 卤16g;
	uint8_t accel_config = static_cast<int>(accel_range) << 3;
	assert(i2c_master->SendByte(MPU6050_DEFAULT_ADDRESS, (uint8_t) MPU6050_RA_ACCEL_CONFIG, accel_config));
	System::Systick::DelayUS(1);

	for (uint8_t i = 0; i < 3; i++) {
		omega_offset[i] = 0;
		omega_offset_f[i] = 0;
	}

	if (config.cal_drift) {
		Calibrate();
		CalibrateF();
	}
}

bool MPU6050::Verify() {
	uint8_t who_am_i;
	if (!i2c_master->GetByte(MPU6050_DEFAULT_ADDRESS, (uint8_t) MPU6050_RA_WHO_AM_I, who_am_i)) {
		return false;
	} else {
		return (who_am_i == 0x68);
	}
}

void MPU6050::Calibrate() {
	uint32_t t = 0, pt = 0;
	int32_t omega_sum[3] = { };

	int samples = 0, target_samples = 256;
	while (samples < target_samples) {
		t = System::Systick::GetTimeInMS();
		if (t - pt >= 5) {
			pt = t;
			Update(false);
			if (samples >= target_samples / 2) {
				const int32_t* omega_ = GetOmega();
				for (uint8_t i = 0; i < 3; i++) {
					omega_sum[i] += omega_[i];
				}
			}
			samples++;
		}
	}
	for (uint8_t i = 0; i < 3; i++) {
		omega_offset[i] = omega_sum[i] / (target_samples / 2);
	}
	is_calibrated = true;
}
void MPU6050::CalibrateF() {
	uint32_t t = 0, pt = 0;
	float omega_sum[3] = { };

	int samples = 0, target_samples = 256;
	while (samples < target_samples) {
		t = System::Systick::GetTimeInMS();
		if (t - pt >= 5) {
			pt = t;
			UpdateF(false);
			if (samples >= target_samples / 2) {
				const float* omega_ = GetOmegaF();
				for (uint8_t i = 0; i < 3; i++) {
					omega_sum[i] += omega_[i];
				}
			}
			samples++;
		}
	}
	for (uint8_t i = 0; i < 3; i++) {
		omega_offset_f[i] = omega_sum[i] / (target_samples / 2);
	}
	is_calibrated = true;
}

uint16_t MPU6050::GetGyroScaleFactor() {
	switch (gyro_range) {
	default:
		assert(false);
		break;
	case Config::Range::kSmall:
		return 20960;

	case Config::Range::kMid:
		return 10480;

	case Config::Range::kLarge:
		return 5248;

	case Config::Range::kExtreme:
		return 2624;
	}
}

uint16_t MPU6050::GetAccelScaleFactor() {
	switch (accel_range) {
	default:
		assert(false);
		break;
	case Config::Range::kSmall:
		return 16384;

	case Config::Range::kMid:
		return 8192;

	case Config::Range::kLarge:
		return 4096;

	case Config::Range::kExtreme:
		return 2048;
	}
}

bool MPU6050::Update(const bool clamp_) {
	uint8_t* data;
	assert(i2c_master->GetBytes(MPU6050_DEFAULT_ADDRESS, (uint8_t) MPU6050_RA_ACCEL_XOUT_H, data, 14));

	int16_t raw_accel[3] = { 0 };
	int16_t raw_gyro[3] = { 0 };
	for (uint8_t i = 0; i < 14; i += 2) {
		if (i <= 5) {
			const int j = i / 2;
			raw_accel[j] = (data[i] << 8) | data[i + 1];
			accel[j] = raw_accel[j];
		} else if (i == 6) {
			const int16_t raw_temp = (data[i] << 8) | data[i + 1];
			temp = (float) raw_temp / 340 + 36.53;
		} else {
			const int j = (i - 8) / 2;
			raw_gyro[j] = (data[i] << 8) | data[i + 1];
			omega[j] = raw_gyro[j] * 160;
			omega[j] -= omega_offset[j];
			if (clamp_)
				omega[j] = (std::abs(omega[j]) < 3 * gyro_scale_factor) ? 0 : omega[j];
		}
	}
	return true;
}
bool MPU6050::UpdateF(const bool clamp_) {
	uint8_t* data;
	assert(i2c_master->GetBytes(MPU6050_DEFAULT_ADDRESS, (uint8_t) MPU6050_RA_ACCEL_XOUT_H, data, 14));

	int16_t raw_accel[3] = { 0 };
	int16_t raw_gyro[3] = { 0 };
	for (uint8_t i = 0; i < 14; i += 2) {
		if (i <= 5) {
			const int j = i / 2;
			raw_accel[j] = (data[i] << 8) | data[i + 1];
			accel_f[j] = (float) raw_accel[j] / accel_scale_factor;
		} else if (i == 6) {
			const int16_t raw_temp = (data[i] << 8) | data[i + 1];
			temp = (float) raw_temp / 340 + 36.53;
		} else {
			const int j = (i - 8) / 2;
			raw_gyro[j] = (data[i] << 8) | data[i + 1];
			omega_f[j] = (float) raw_gyro[j] * 160 / gyro_scale_factor;
			omega_f[j] -= omega_offset_f[j];
			if (clamp_)
				omega_f[j] = std::abs(omega_f[j]) < 3.0f ? 0.0f : omega_f[j];
		}
	}
	return true;
}

}
