/*
 * spi_master.cpp
 *
 *  Created on: Sep 17, 2018
 *      Author: LeeChunHei
 */

#include "driver/spi_master.h"
#include "system/clock.h"
#include "system/system.h"

namespace Driver {

using namespace System;
const clock_ip_name_t s_lpspiClocks[] = LPSPI_CLOCKS;
const uint8_t baud_rate_prescaler[] = { 1, 2, 4, 8, 16, 32, 64, 128 };
uint32_t spi_clock = 0;

SpiMaster::SpiMaster(const Config& config) {
	uint8_t module, prev_module, chip_select;

	Pinout::Config pin_config;

	pin_config.pin = config.sck;
	if (Pinout::GetSpiSckPinConfig(pin_config, prev_module)) {
		Pinout::InitPin(pin_config);
	} else {
		return;
	}
	pin_config.pin = config.sdo;
	if (pin_config.pin != System::Pinout::Name::kDisable) {
		if (Pinout::GetSpiSdoPinConfig(pin_config, module) && module == prev_module) {
			Pinout::InitPin(pin_config);
		} else {
			return;
		}
	}
	pin_config.pin = config.sdi;
	if (pin_config.pin != System::Pinout::Name::kDisable) {
		if (Pinout::GetSpiSdiPinConfig(pin_config, module) && module == prev_module) {
			Pinout::InitPin(pin_config);
		} else {
			return;
		}
	}
	pin_config.pin = config.cs;
	if (Pinout::GetSpiPCSPinConfig(pin_config, module, chip_select) && module == prev_module) {
		Pinout::InitPin(pin_config);
	} else {
		return;
	}

	/* Enable LPSPI clock */
	CLOCK_EnableClock(s_lpspiClocks[module]);

	spi_base = (LPSPI_Type*) (0x40394000u + (module - 1) * 0x4000);

	if (!spi_clock) {
		spi_clock = (usb1_pll_pfd0_clock / (CLOCK_GetDiv(kCLOCK_LpspiDiv) + 1U));
	}

	/* Reset to known status */
	Reset();

	/* Set LPSPI to master */
	SetMaster();

	/* Set specific PCS to active high or low */
	SetOnePcsPolarity(chip_select, config.chip_select_active_high);

	/* Set Configuration Register 1 related setting.*/
	spi_base->CFGR1 = (spi_base->CFGR1 & ~(LPSPI_CFGR1_OUTCFG_MASK | LPSPI_CFGR1_PINCFG_MASK | LPSPI_CFGR1_NOSTALL_MASK)) | LPSPI_CFGR1_OUTCFG(config.data_out_config) | LPSPI_CFGR1_PINCFG(config.pin_config) | LPSPI_CFGR1_NOSTALL(0);

	/* Set baudrate and delay times*/
	uint32_t tcr_prescale_value = SetBaudRate(config.baud_rate);

	/* Set default watermarks */
	SetFifoWatermarks(0, 0);

	/* Set Transmit Command Register*/
	spi_base->TCR = LPSPI_TCR_CPOL(config.clock_polarity_active_low) | LPSPI_TCR_CPHA(config.clock_phase) | LPSPI_TCR_LSBF(config.transfer_LSB_first) | LPSPI_TCR_FRAMESZ(config.bits_per_frame - 1) | LPSPI_TCR_PRESCALE(tcr_prescale_value) | LPSPI_TCR_PCS(chip_select);

	Enable(true);

	SetDelayTimes(config.pcs_to_sck_delay_ns, 1);
	SetDelayTimes(config.last_sck_to_pcs_delay_ns, 2);
	SetDelayTimes(config.between_transfer_delay_ns, 3);

	SetDummyData(0x00);

	rx_fifo_size = (1U << ((spi_base->PARAM & LPSPI_PARAM_RXFIFO_MASK) >> LPSPI_PARAM_RXFIFO_SHIFT));
	tx_fifo_size = (1U << ((spi_base->PARAM & LPSPI_PARAM_TXFIFO_MASK) >> LPSPI_PARAM_TXFIFO_SHIFT));

	Enable(false);
	spi_base->CFGR1 &= (~LPSPI_CFGR1_NOSTALL_MASK);
	Enable(true);

	/*Flush FIFO , clear status , disable all the inerrupts.*/
	spi_base->CR |= ((uint32_t) 1 << LPSPI_CR_RTF_SHIFT) | ((uint32_t) 1 << LPSPI_CR_RRF_SHIFT | 1 << 3);
	spi_base->SR = 16793347;
	spi_base->IER &= ~16131;
	spi_base->TCR = (spi_base->TCR & ~(LPSPI_TCR_CONT_MASK | LPSPI_TCR_CONTC_MASK | LPSPI_TCR_RXMSK_MASK | LPSPI_TCR_PCS_MASK)) | LPSPI_TCR_CONT(0) | LPSPI_TCR_CONTC(0) | LPSPI_TCR_RXMSK(0) | LPSPI_TCR_TXMSK(0) | LPSPI_TCR_PCS(chip_select);
	if (config.pin_config == Config::PinConfig::kSdiInSdiOut || config.pin_config == Config::PinConfig::kSdoInSdoOut) {
		spi_base->TCR |= LPSPI_TCR_RXMSK(1);
	}
	while (GetTxFifoCount())
		;
}

void SpiMaster::Reset() {
	/* Reset all internal logic and registers, except the Control Register. Remains set until cleared by software.*/
	spi_base->CR |= LPSPI_CR_RST_MASK;

	/* Software reset doesn't reset the CR, so manual reset the FIFOs */
	spi_base->CR |= LPSPI_CR_RRF_MASK | LPSPI_CR_RTF_MASK;

	/* Master logic is not reset and module is disabled.*/
	spi_base->CR = 0x00U;
}

void SpiMaster::SetMaster() {
	spi_base->CFGR1 = (spi_base->CFGR1 & (~LPSPI_CFGR1_MASTER_MASK)) | LPSPI_CFGR1_MASTER(1);
}

void SpiMaster::SetOnePcsPolarity(uint8_t chip_select, bool chip_select_active_high) {
	/* Clear the PCS polarity bit */
	uint32_t cfgr1 = spi_base->CFGR1 & ~(1U << (LPSPI_CFGR1_PCSPOL_SHIFT + chip_select));

	/* Configure the PCS polarity bit according to the activeLowOrHigh setting */
	spi_base->CFGR1 = cfgr1 | ((uint32_t) chip_select_active_high << (LPSPI_CFGR1_PCSPOL_SHIFT + chip_select));
}

uint32_t SpiMaster::SetBaudRate(uint32_t baud_rate) {
	/* find combination of prescaler and scaler resulting in baudrate closest to the
	 * requested value
	 */
	uint32_t min_diff = 0xFFFFFFFFU;

	/* Set to maximum divisor value bit settings so that if baud rate passed in is less
	 * than the minimum possible baud rate, then the SPI will be configured to the lowest
	 * possible baud rate
	 */
	uint32_t best_prescaler = 7;
	uint32_t best_scaler = 255;

	uint32_t best_baudrate = 0; /* required to avoid compilation warning */

	/* In all for loops, if min_diff = 0, the exit for loop*/
	for (uint32_t prescaler = 0; (prescaler < 8) && min_diff; prescaler++) {
		for (uint32_t scaler = 0; (scaler < 256) && min_diff; scaler++) {
			uint32_t real_baudrate = (spi_clock / (baud_rate_prescaler[prescaler] * (scaler + 2U)));

			/* calculate the baud rate difference based on the conditional statement
			 * that states that the calculated baud rate must not exceed the desired baud rate
			 */
			if (baud_rate >= real_baudrate) {
				uint32_t diff = baud_rate - real_baudrate;
				if (min_diff > diff) {
					/* a better match found */
					min_diff = diff;
					best_prescaler = prescaler;
					best_scaler = scaler;
					best_baudrate = real_baudrate;
				}
			}
		}
	}

	/* Write the best baud rate scalar to the CCR.
	 * Note, no need to check for error since we've already checked to make sure the module is
	 * disabled and in master mode. Also, there is a limit on the maximum divider so we will not
	 * exceed this.
	 */
	spi_base->CCR = (spi_base->CCR & ~LPSPI_CCR_SCKDIV_MASK) | LPSPI_CCR_SCKDIV(best_scaler);

	/* return the best prescaler value for user to use later */
	return best_prescaler;
}

void SpiMaster::SetFifoWatermarks(uint8_t tx_watermark, uint8_t rx_watermark) {
	spi_base->FCR = LPSPI_FCR_TXWATER(tx_watermark) | LPSPI_FCR_RXWATER(rx_watermark);
}

void SpiMaster::Enable(bool enable) {
	if (enable) {
		spi_base->CR |= LPSPI_CR_MEN_MASK;
	} else {
		spi_base->CR &= ~LPSPI_CR_MEN_MASK;
	}
}

void SpiMaster::SetDelayTimes(uint32_t delay_ns, uint8_t type) {
	/* For delay between transfer, an additional scaler value is needed */
	uint32_t additional_scaler = 0;

	/*As the RM note, the LPSPI baud rate clock is itself divided by the PRESCALE setting, which can vary between
	 * transfers.*/
	uint32_t clock_divided_prescaler = spi_clock / baud_rate_prescaler[(spi_base->TCR & LPSPI_TCR_PRESCALE_MASK) >> LPSPI_TCR_PRESCALE_SHIFT];

	/* Find combination of prescaler and scaler resulting in the delay closest to the requested value.*/
	uint32_t min_diff = 0xFFFFFFFFU;

	/* Initialize scaler to max value to generate the max delay */
	uint32_t best_scaler = 0xFFU;

	uint64_t initial_delay_ns;
	/* Calculate the initial (min) delay and maximum possible delay based on the specific delay as
	 * the delay divisors are slightly different based on which delay we are configuring.
	 */
	if (type == 3) {
		/* First calculate the initial, default delay, note min delay is 2 clock cycles. Due to large size of
		 calculated values (uint64_t), we need to break up the calculation into several steps to ensure
		 accurate calculated results
		 */
		initial_delay_ns = 1000000000U;
		initial_delay_ns *= 2U;
		initial_delay_ns /= clock_divided_prescaler;

		additional_scaler = 1U;
	} else {
		/* First calculate the initial, default delay, min delay is 1 clock cycle. Due to large size of calculated
		 values (uint64_t), we need to break up the calculation into several steps to ensure accurate calculated
		 results.
		 */
		initial_delay_ns = 1000000000U;
		initial_delay_ns /= clock_divided_prescaler;

		additional_scaler = 0;
	}

	/* If the initial, default delay is already greater than the desired delay, then
	 * set the delay to their initial value (0) and return the delay. In other words,
	 * there is no way to decrease the delay value further.
	 */
	if (initial_delay_ns >= delay_ns) {
		SetDelayScaler(0, type);
		return;
	}

	/* If min_diff = 0, the exit for loop */
	for (uint32_t scaler = 0; (scaler < 256U) && min_diff; scaler++) {
		/* Calculate the real delay value as we cycle through the scaler values.
		 Due to large size of calculated values (uint64_t), we need to break up the
		 calculation into several steps to ensure accurate calculated results
		 */
		uint64_t real_delay = 1000000000U;
		real_delay *= (scaler + 1 + additional_scaler);
		real_delay /= clock_divided_prescaler;

		/* calculate the delay difference based on the conditional statement
		 * that states that the calculated delay must not be less then the desired delay
		 */
		if (real_delay >= delay_ns) {
			uint32_t diff = real_delay - delay_ns;
			if (min_diff > diff) {
				/* a better match found */
				min_diff = diff;
				best_scaler = scaler;
			}
		}
	}

	/* write the best scaler value for the delay */
	SetDelayScaler(best_scaler, type);
}

void SpiMaster::SetDelayScaler(uint32_t scaler, uint8_t type) {
	/*These settings are only relevant in master mode */
	switch (type) {
	case 1:
		spi_base->CCR = (spi_base->CCR & (~LPSPI_CCR_PCSSCK_MASK)) | LPSPI_CCR_PCSSCK(scaler);
		break;
	case 2:
		spi_base->CCR = (spi_base->CCR & (~LPSPI_CCR_SCKPCS_MASK)) | LPSPI_CCR_SCKPCS(scaler);
		break;
	case 3:
		spi_base->CCR = (spi_base->CCR & (~LPSPI_CCR_DBT_MASK)) | LPSPI_CCR_DBT(scaler);
		break;
	default:
		assert(false);
		break;
	}
}

void SpiMaster::SetDummyData(uint8_t dummy_data) {

}

}
