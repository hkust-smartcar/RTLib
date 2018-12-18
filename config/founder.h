/*
 * founder.h
 *
 *  Created on: Aug 3, 2018
 *      Author: LeeChunHei
 */

#ifndef CONFIG_FOUNDER_H_
#define CONFIG_FOUNDER_H_

#define __TARGET_FPU_VFP

#define CPU_XTAL_CLK_HZ 24000000UL //value of external crystal

#define FLASH_SIZE 128*1024*1024u	//128MB remember to modify linker script also
#define FLASH_PAGE_SIZE 256u
#define FLASH_SECTOR_SIZE 4u*1024u
#define FLASH_BLOCK_SIZE 64u*1024u
#define FLASH_FLEXSPI_SCK	System::Pinout::Name::kGPIO_SD_B1_07
#define FLASH_FLEXSPI_CS	System::Pinout::Name::kGPIO_SD_B1_06
#define FLASH_FLEXSPI_SIO0	System::Pinout::Name::kGPIO_SD_B1_08
#define FLASH_FLEXSPI_SIO1	System::Pinout::Name::kGPIO_SD_B1_09
#define FLASH_FLEXSPI_SIO2	System::Pinout::Name::kGPIO_SD_B1_10
#define FLASH_FLEXSPI_SIO3	System::Pinout::Name::kGPIO_SD_B1_11
#define FLASH_FLEXSPI_SIO4	System::Pinout::Name::kDisable
#define FLASH_FLEXSPI_SIO5	System::Pinout::Name::kDisable
#define FLASH_FLEXSPI_SIO6	System::Pinout::Name::kDisable
#define FLASH_FLEXSPI_SIO7	System::Pinout::Name::kDisable
#define FLASH_FLEXSPI_DQS	System::Pinout::Name::kDisable
#define FLEXSPI_LUT_SEQ(cmd0, pad0, op0, cmd1, pad1, op1)                                                              \
    (FLEXSPI_LUT_OPERAND0(op0) | FLEXSPI_LUT_NUM_PADS0(pad0) | FLEXSPI_LUT_OPCODE0(cmd0) | FLEXSPI_LUT_OPERAND1(op1) | \
     FLEXSPI_LUT_NUM_PADS1(pad1) | FLEXSPI_LUT_OPCODE1(cmd1))

#define FLASH_LUT_READ_NORMAL                 0
#define FLASH_LUT_READ_FAST                   1
#define FLASH_LUT_READ_FAST_QUAD              2
#define FLASH_LUT_READSTATUS                  3
#define FLASH_LUT_WRITEENABLE                 4
#define FLASH_LUT_ERASESECTOR                 5
#define FLASH_LUT_PAGEPROGRAM_SINGLE          6
#define FLASH_LUT_PAGEPROGRAM_QUAD            7
#define FLASH_LUT_READID                      8
#define FLASH_LUT_READJEDECID                 9
#define FLASH_LUT_WRITESTATUSREG              10
#define FLASH_LUT_READSTATUSREG               11
#define FLASH_LUT_ERASECHIP                   12
#define FLASH_LUT_AHB_PAGEPROGRAM_QUAD_1      13
#define FLASH_LUT_AHB_PAGEPROGRAM_QUAD_2      14

/* FLASH常用命令 */
#define W25Q_WriteEnable                0x06
#define W25Q_WriteDisable               0x04
#define W25Q_ReadStatusReg              0x05
#define W25Q_WriteStatusReg             0x01
#define W25Q_ReadData                   0x03
#define W25Q_ReadData_4Addr             0x13
#define W25Q_FastReadData               0x0B
#define W25Q_FastReadData_4Addr         0x0C
#define W25Q_FastReadDual               0x3B
#define W25Q_FastReadDual_4Addr         0x3C
#define W25Q_FastReadQuad               0x6B
#define W25Q_FastReadQuad_4Addr         0x6C
#define W25Q_PageProgram                0x02
#define W25Q_PageProgram_4Addr          0x12
#define W25Q_PageProgramQuad            0x32
#define W25Q_PageProgramQuad_4Addr      0x34
#define W25Q_BlockErase                 0xD8
#define W25Q_BlockErase_4Addr           0xDC
#define W25Q_SectorErase                0x20
#define W25Q_SectorErase_4Addr          0x21
#define W25Q_ChipErase                  0xC7
#define W25Q_PowerDown                  0xB9
#define W25Q_ReleasePowerDown           0xAB
#define W25Q_DeviceID                   0xAB
#define W25Q_ManufactDeviceID           0x90
#define W25Q_JedecDeviceID              0x9F
/*其它*/
#define FLASH_ID                        0X18
#define FLASH_JEDECDEVICE_ID            0XEF4018
#define FLASH_LUT_SIZE	60
#define FLASH_LUT_CONTENT	{	/* 普通读指令，Normal read mode -SDR */\
								FLEXSPI_LUT_SEQ(Driver::FlexSpi::LUTCommand::kSDR, Driver::FlexSpi::Pad::k1PAD, W25Q_ReadData_4Addr, Driver::FlexSpi::LUTCommand::kRADDR_SDR, Driver::FlexSpi::Pad::k1PAD, 32),\
								FLEXSPI_LUT_SEQ(Driver::FlexSpi::LUTCommand::kREAD_SDR, Driver::FlexSpi::Pad::k1PAD, 0x04, Driver::FlexSpi::LUTCommand::kSTOP, Driver::FlexSpi::Pad::k1PAD, 0),0,0,\
								/* 快速读指令，Fast read mode - SDR */\
								FLEXSPI_LUT_SEQ(Driver::FlexSpi::LUTCommand::kSDR, Driver::FlexSpi::Pad::k1PAD, W25Q_FastReadData_4Addr, Driver::FlexSpi::LUTCommand::kRADDR_SDR, Driver::FlexSpi::Pad::k1PAD, 32),\
								FLEXSPI_LUT_SEQ(Driver::FlexSpi::LUTCommand::kDUMMY_SDR, Driver::FlexSpi::Pad::k1PAD, 0x08, Driver::FlexSpi::LUTCommand::kREAD_SDR, Driver::FlexSpi::Pad::k1PAD, 0x04),0,0,\
								/* QUAD模式快速读指令，Fast read quad mode - SDR */\
								FLEXSPI_LUT_SEQ(Driver::FlexSpi::LUTCommand::kSDR, Driver::FlexSpi::Pad::k1PAD, 0xEB, Driver::FlexSpi::LUTCommand::kRADDR_SDR, Driver::FlexSpi::Pad::k4PAD, 0x18),\
								FLEXSPI_LUT_SEQ(Driver::FlexSpi::LUTCommand::kDUMMY_SDR, Driver::FlexSpi::Pad::k4PAD, 0x06, Driver::FlexSpi::LUTCommand::kREAD_SDR, Driver::FlexSpi::Pad::k4PAD, 0x04),0,0,\
								/* 读取扩展参数，Read extend parameters */\
								FLEXSPI_LUT_SEQ(Driver::FlexSpi::LUTCommand::kSDR, Driver::FlexSpi::Pad::k1PAD, W25Q_ReadStatusReg, Driver::FlexSpi::LUTCommand::kREAD_SDR, Driver::FlexSpi::Pad::k1PAD, 0x04),0,0,0,\
								/* 写使能，Write Enable */\
								FLEXSPI_LUT_SEQ(Driver::FlexSpi::LUTCommand::kSDR, Driver::FlexSpi::Pad::k1PAD, W25Q_WriteEnable, Driver::FlexSpi::LUTCommand::kSTOP, Driver::FlexSpi::Pad::k1PAD, 0),0,0,0,\
								/* 擦除扇区，Erase Sector  */\
								FLEXSPI_LUT_SEQ(Driver::FlexSpi::LUTCommand::kSDR, Driver::FlexSpi::Pad::k1PAD, 0x20, Driver::FlexSpi::LUTCommand::kRADDR_SDR, Driver::FlexSpi::Pad::k1PAD, 24),0,0,0,\
								/* SINGLE模式页写入，Page Program - single mode */\
								FLEXSPI_LUT_SEQ(Driver::FlexSpi::LUTCommand::kSDR, Driver::FlexSpi::Pad::k1PAD, W25Q_PageProgram_4Addr, Driver::FlexSpi::LUTCommand::kRADDR_SDR, Driver::FlexSpi::Pad::k1PAD, 32),\
								FLEXSPI_LUT_SEQ(Driver::FlexSpi::LUTCommand::kWRITE_SDR, Driver::FlexSpi::Pad::k1PAD, 0x04, Driver::FlexSpi::LUTCommand::kSTOP, Driver::FlexSpi::Pad::k1PAD, 0),0,0,\
								/* QUAD模式页写入，Page Program - quad mode */\
								FLEXSPI_LUT_SEQ(Driver::FlexSpi::LUTCommand::kSDR, Driver::FlexSpi::Pad::k1PAD, 0x32, Driver::FlexSpi::LUTCommand::kRADDR_SDR, Driver::FlexSpi::Pad::k1PAD, 24),\
								FLEXSPI_LUT_SEQ(Driver::FlexSpi::LUTCommand::kWRITE_SDR, Driver::FlexSpi::Pad::k4PAD, 0x04, Driver::FlexSpi::LUTCommand::kSTOP, Driver::FlexSpi::Pad::k1PAD, 0),0,0,\
								/* 读ID，Read ID ，ID7-ID0*/\
								FLEXSPI_LUT_SEQ(Driver::FlexSpi::LUTCommand::kSDR, Driver::FlexSpi::Pad::k1PAD, W25Q_DeviceID, Driver::FlexSpi::LUTCommand::kDUMMY_SDR, Driver::FlexSpi::Pad::k1PAD, 32),\
								FLEXSPI_LUT_SEQ(Driver::FlexSpi::LUTCommand::kREAD_SDR, Driver::FlexSpi::Pad::k1PAD, 0x04, Driver::FlexSpi::LUTCommand::kSTOP, Driver::FlexSpi::Pad::k1PAD, 0),0,0,\
								/* 读JedecDeviceID,MF7-MF0+ID15-ID0 */\
								FLEXSPI_LUT_SEQ(Driver::FlexSpi::LUTCommand::kSDR, Driver::FlexSpi::Pad::k1PAD, W25Q_JedecDeviceID, Driver::FlexSpi::LUTCommand::kREAD_SDR, Driver::FlexSpi::Pad::k1PAD, 0x04),0,0,0,\
								/* 写状态寄存器，Enable Quad mode */\
								FLEXSPI_LUT_SEQ(Driver::FlexSpi::LUTCommand::kSDR, Driver::FlexSpi::Pad::k1PAD, W25Q_WriteStatusReg, Driver::FlexSpi::LUTCommand::kWRITE_SDR, Driver::FlexSpi::Pad::k1PAD, 0x04),0,0,0,\
								/* 读状态寄存器，Read status register */\
								FLEXSPI_LUT_SEQ(Driver::FlexSpi::LUTCommand::kSDR, Driver::FlexSpi::Pad::k1PAD, W25Q_ReadStatusReg, Driver::FlexSpi::LUTCommand::kREAD_SDR, Driver::FlexSpi::Pad::k1PAD, 0x04),0,0,0,\
								/* 擦除整片FLASH，Erase Chip */\
								FLEXSPI_LUT_SEQ(Driver::FlexSpi::LUTCommand::kSDR, Driver::FlexSpi::Pad::k1PAD, W25Q_ChipErase, Driver::FlexSpi::LUTCommand::kSTOP, Driver::FlexSpi::Pad::k1PAD, 0),0,0,0,\
								/* 给AHB命令访问的 QUAD模式页写入 序列，包含写使能和页写入两条序列 */\
								/* 写使能，Write Enable */\
								FLEXSPI_LUT_SEQ(Driver::FlexSpi::LUTCommand::kSDR, Driver::FlexSpi::Pad::k1PAD, W25Q_WriteEnable, Driver::FlexSpi::LUTCommand::kSTOP, Driver::FlexSpi::Pad::k1PAD, 0),0,0,0,\
								/* QUAD模式页写入，Page Program - quad mode */\
								FLEXSPI_LUT_SEQ(Driver::FlexSpi::LUTCommand::kSDR, Driver::FlexSpi::Pad::k1PAD, W25Q_PageProgramQuad_4Addr, Driver::FlexSpi::LUTCommand::kRADDR_SDR, Driver::FlexSpi::Pad::k1PAD, 32),\
								FLEXSPI_LUT_SEQ(Driver::FlexSpi::LUTCommand::kWRITE_SDR, Driver::FlexSpi::Pad::k4PAD, 0x04, Driver::FlexSpi::LUTCommand::kSTOP, Driver::FlexSpi::Pad::k1PAD, 0)}

#define ST7735R_USED 1
#define MT9V034_USED 1
#define MPU6050_USED 1
#define LED_USED 3
#define USE_LVGL 1

#define ST7735R_SCL(id) id==0?System::Pinout::Name::kGPIO_AD_B0_00:System::Pinout::Name::kDisable
#define ST7735R_SDA(id) id==0?System::Pinout::Name::kGPIO_AD_B0_01:System::Pinout::Name::kDisable
#define ST7735R_CS(id) id==0?System::Pinout::Name::kGPIO_AD_B0_03:System::Pinout::Name::kDisable
#define ST7735R_DC(id) id==0?System::Pinout::Name::kGPIO_B0_01:System::Pinout::Name::kDisable
#define ST7735R_RESET(id) id==0?System::Pinout::Name::kGPIO_B0_00:System::Pinout::Name::kDisable
#define ST7735R_BL(id) id==0?System::Pinout::Name::kGPIO_B0_02:System::Pinout::Name::kDisable

#define MT9V034_DATABUS 8
#define MT9V034_SDA(id) id==0?System::Pinout::Name::kGPIO_AD_B1_01:System::Pinout::Name::kDisable
#define MT9V034_SCK(id) id==0?System::Pinout::Name::kGPIO_AD_B1_00:System::Pinout::Name::kDisable
#define MT9V034_VSYNC(id) id==0?System::Pinout::Name::kGPIO_AD_B1_06:System::Pinout::Name::kDisable
#define MT9V034_HSYNC(id) id==0?System::Pinout::Name::kGPIO_AD_B1_07:System::Pinout::Name::kDisable
#define MT9V034_PCLK(id) id==0?System::Pinout::Name::kGPIO_AD_B1_04:System::Pinout::Name::kDisable
#define MT9V034_D2(id) id==0?System::Pinout::Name::kGPIO_AD_B1_15:System::Pinout::Name::kDisable
#define MT9V034_D3(id) id==0?System::Pinout::Name::kGPIO_AD_B1_14:System::Pinout::Name::kDisable
#define MT9V034_D4(id) id==0?System::Pinout::Name::kGPIO_AD_B1_13:System::Pinout::Name::kDisable
#define MT9V034_D5(id) id==0?System::Pinout::Name::kGPIO_AD_B1_12:System::Pinout::Name::kDisable
#define MT9V034_D6(id) id==0?System::Pinout::Name::kGPIO_AD_B1_11:System::Pinout::Name::kDisable
#define MT9V034_D7(id) id==0?System::Pinout::Name::kGPIO_AD_B1_10:System::Pinout::Name::kDisable
#define MT9V034_D8(id) id==0?System::Pinout::Name::kGPIO_AD_B1_09:System::Pinout::Name::kDisable
#define MT9V034_D9(id) id==0?System::Pinout::Name::kGPIO_AD_B1_08:System::Pinout::Name::kDisable

#define MPU6050_SDA(id) id==0?System::Pinout::Name::kGPIO_AD_B1_01:System::Pinout::Name::kDisable
#define MPU6050_SCK(id) id==0?System::Pinout::Name::kGPIO_AD_B1_00:System::Pinout::Name::kDisable

#define FIREGE5INCHTOUCHSCREEN_SDA(id)	id==0?System::Pinout::Name::kGPIO_SD_B1_05:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_SCL(id) 	id==0?System::Pinout::Name::kGPIO_SD_B1_04:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_INT(id) 	id==0?System::Pinout::Name::kGPIO_AD_B0_11:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_RST(id) 	id==0?System::Pinout::Name::kGPIO_AD_B0_12:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_BK(id) 	id==0?System::Pinout::Name::kGPIO_AD_B0_13:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_DISP(id) id==0?System::Pinout::Name::kGPIO_AD_B0_14:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_D0(id)	id==0?System::Pinout::Name::kGPIO_B0_04:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_D1(id)	id==0?System::Pinout::Name::kGPIO_B0_05:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_D2(id)	id==0?System::Pinout::Name::kGPIO_B0_06:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_D3(id)	id==0?System::Pinout::Name::kGPIO_B0_07:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_D4(id)	id==0?System::Pinout::Name::kGPIO_B0_08:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_D5(id)	id==0?System::Pinout::Name::kGPIO_B0_09:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_D6(id)	id==0?System::Pinout::Name::kGPIO_B0_10:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_D7(id)	id==0?System::Pinout::Name::kGPIO_B0_11:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_D8(id)	id==0?System::Pinout::Name::kGPIO_B0_12:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_D9(id)	id==0?System::Pinout::Name::kGPIO_B0_13:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_D10(id)	id==0?System::Pinout::Name::kGPIO_B0_14:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_D11(id)	id==0?System::Pinout::Name::kGPIO_B0_15:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_D12(id)	id==0?System::Pinout::Name::kGPIO_B1_00:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_D13(id)	id==0?System::Pinout::Name::kGPIO_B1_01:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_D14(id)	id==0?System::Pinout::Name::kGPIO_B1_02:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_D15(id)	id==0?System::Pinout::Name::kGPIO_B1_03:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_D16(id)	id==0?System::Pinout::Name::kGPIO_B1_04:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_D17(id)	id==0?System::Pinout::Name::kGPIO_B1_05:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_D18(id)	id==0?System::Pinout::Name::kGPIO_B1_06:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_D19(id)	id==0?System::Pinout::Name::kGPIO_B1_07:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_D20(id)	id==0?System::Pinout::Name::kGPIO_B1_08:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_D21(id)	id==0?System::Pinout::Name::kGPIO_B1_09:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_D22(id)	id==0?System::Pinout::Name::kGPIO_B1_10:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_D23(id)	id==0?System::Pinout::Name::kGPIO_B1_11:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_PCLK(id)	id==0?System::Pinout::Name::kGPIO_B0_00:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_ENABLE(id)	id==0?System::Pinout::Name::kGPIO_B0_01:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_HSYNC(id)	id==0?System::Pinout::Name::kGPIO_B0_02:System::Pinout::Name::kDisable
#define FIREGE5INCHTOUCHSCREEN_VSYNC(id)	id==0?System::Pinout::Name::kGPIO_B0_03:System::Pinout::Name::kDisable

#define LED_PIN		{	System::Pinout::Name::kGPIO_EMC_40,\
						System::Pinout::Name::kGPIO_EMC_41,\
						System::Pinout::Name::kGPIO_B1_07	}


#endif /* CONFIG_FOUNDER_H_ */
