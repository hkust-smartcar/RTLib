/*
 * uart.cpp
 *
 *  Created on: Aug 26, 2018
 *      Author: LeeChunHei
 */

#include "driver/uart.h"
#include "system/clock.h"
#include "system/system.h"

namespace Driver {

/* Array of LPUART clock name. */
static const System::clock_ip_name_t s_lpuart_clock[] = { System::kCLOCK_IpInvalid, System::kCLOCK_Lpuart1, System::kCLOCK_Lpuart2, System::kCLOCK_Lpuart3, System::kCLOCK_Lpuart4, System::kCLOCK_Lpuart5, System::kCLOCK_Lpuart6, System::kCLOCK_Lpuart7, System::kCLOCK_Lpuart8 };

uint32_t uart_clock = 0;

uint32_t GetUartClock() {

	/* To make it simple, we assume default PLL and divider settings, and the only variable
	 from application is use PLL3 source or OSC source */
	if (System::CLOCK_GetMux(System::kCLOCK_UartMux) == 0) /* PLL3 div6 80M */
	{
		return (System::usb1_pll_clock / 6u) / (System::CLOCK_GetDiv(System::kCLOCK_UartDiv) + 1u);
	} else {
		return System::CLOCK_GetOscFreq() / (System::CLOCK_GetDiv(System::kCLOCK_UartDiv) + 1u);
	}
}

Uart::Uart(Config& config) :
		m_data_bits(config.data_bits) {
	assert(config.tx_fifo_watermark < 5);
	assert(config.rx_fifo_watermark < 5);

	/* This LPUART instantiation uses a slightly different baud rate calculation
	 * The idea is to use the best OSR (over-sampling rate) possible
	 * Note, OSR is typically hard-set to 16 in other LPUART instantiations
	 * loop to find the best OSR value possible, one that generates minimum baudDiff
	 * iterate through the rest of the supported values of OSR */

	uint32_t baud_diff = config.baud_rate;
	uint32_t osr = 0;
	uint32_t sbr = 0;
	if (!uart_clock) {
		uart_clock = GetUartClock();
	}
	for (uint8_t osr_temp = 4; osr_temp < 33; osr_temp++) {
		/* calculate the temporary sbr value   */
		uint32_t sbr_temp = (uart_clock / (config.baud_rate * osr_temp));
		/*set sbrTemp to 1 if the sourceClockInHz can not satisfy the desired baud rate*/
		if (sbr_temp == 0) {
			sbr_temp = 1;
		}

		uint32_t temp_diff = (uart_clock / (osr_temp * sbr_temp)) - config.baud_rate;

		/* Select the better value between srb and (sbr + 1) */
		if (temp_diff > (config.baud_rate - (uart_clock / (osr_temp * (sbr_temp + 1))))) {
			temp_diff = config.baud_rate - (uart_clock / (osr_temp * (sbr_temp + 1)));
			sbr_temp++;
		}

		if (temp_diff <= baud_diff) {
			baud_diff = temp_diff;
			osr = osr_temp; /* update and store the best OSR value calculated */
			sbr = sbr_temp; /* update store the best SBR value calculated */
		}
	}

	/* Check to see if actual baud rate is within 3% of desired baud rate
	 * based on the best calculate OSR value */
	assert(baud_diff < ((config.baud_rate / 100) * 3));

	uart_base = (LPUART_Type*) (0x40180000u + ((uint8_t) config.uart_module * 0x4000u));

	/* Enable lpuart clock */
	System::CLOCK_EnableClock(s_lpuart_clock[(uint8_t) config.uart_module]);

	uart_base->GLOBAL |= LPUART_GLOBAL_RST_MASK;
	uart_base->GLOBAL &= ~LPUART_GLOBAL_RST_MASK;

	uint32_t reg = uart_base->BAUD;

	/* Acceptable baud rate, check if OSR is between 4x and 7x oversampling.
	 * If so, then "BOTHEDGE" sampling must be turned on */
	if ((osr > 3) && (osr < 8)) {
		reg |= LPUART_BAUD_BOTHEDGE_MASK;
	}

	/* program the osr value (bit value is one less than actual value) */
	reg &= ~LPUART_BAUD_OSR_MASK;
	reg |= LPUART_BAUD_OSR(osr - 1);

	/* write the sbr value to the BAUD registers */
	reg &= ~LPUART_BAUD_SBR_MASK;
	uart_base->BAUD = reg | LPUART_BAUD_SBR(sbr);

	/* Set bit count and parity mode. */
	uart_base->BAUD &= ~LPUART_BAUD_M10_MASK;

	reg = uart_base->CTRL & ~(LPUART_CTRL_PE_MASK | LPUART_CTRL_PT_MASK | LPUART_CTRL_M_MASK | LPUART_CTRL_ILT_MASK | LPUART_CTRL_IDLECFG_MASK);

	reg |= (uint8_t) config.parity_mode | LPUART_CTRL_IDLECFG(config.rx_idle_config) | LPUART_CTRL_ILT(config.rx_idle_type);

	if ((bool) config.data_bits) {
		if ((bool) config.parity_mode) {
			reg &= ~LPUART_CTRL_M7_MASK; /* Seven data bits and one parity bit */
		} else {
			reg |= LPUART_CTRL_M7_MASK;
		}
	} else {
		if ((bool) config.parity_mode) {
			reg |= LPUART_CTRL_M_MASK; /* Eight data bits and one parity bit */
		}
	}

	uart_base->CTRL = reg;

	/* set stop bit per char */
	reg = uart_base->BAUD & ~LPUART_BAUD_SBNS_MASK;
	uart_base->BAUD = reg | LPUART_BAUD_SBNS((uint8_t) config.stop_bit);

	/* Set tx/rx WATER watermark
	 Note:
	 Take care of the RX FIFO, RX interrupt request only assert when received bytes
	 equal or more than RX water mark, there is potential issue if RX water
	 mark larger than 1.
	 For example, if RX FIFO water mark is 2, upper layer needs 5 bytes and
	 5 bytes are received. the last byte will be saved in FIFO but not trigger
	 RX interrupt because the water mark is 2.
	 */
	uart_base->WATER = (((uint32_t)(config.rx_fifo_watermark) << 16) | config.tx_fifo_watermark);

	/* Enable tx/rx FIFO */
	uart_base->FIFO |= (LPUART_FIFO_TXFE_MASK | LPUART_FIFO_RXFE_MASK);

	/* Flush FIFO */
	uart_base->FIFO |= (LPUART_FIFO_TXFLUSH_MASK | LPUART_FIFO_RXFLUSH_MASK);

	/* Clear all status flags */
	reg = (LPUART_STAT_RXEDGIF_MASK | LPUART_STAT_IDLE_MASK | LPUART_STAT_OR_MASK | LPUART_STAT_NF_MASK | LPUART_STAT_FE_MASK | LPUART_STAT_PF_MASK);

	reg |= LPUART_STAT_LBKDIF_MASK;

	reg |= (LPUART_STAT_MA1F_MASK | LPUART_STAT_MA2F_MASK);

	/* Set the CTS configuration/TX CTS source. */
	uart_base->MODIR |= LPUART_MODIR_TXCTSC(config.tx_cts_config) | LPUART_MODIR_TXCTSSRC(config.tx_cts_source);
	if (config.enable_rx_rts) {
		/* Enable the receiver RTS(request-to-send) function. */
		uart_base->MODIR |= LPUART_MODIR_RXRTSE_MASK;
	}
	if (config.enable_tx_cts) {
		/* Enable the CTS(clear-to-send) function. */
		uart_base->MODIR |= LPUART_MODIR_TXCTSE_MASK;
	}

	/* Set data bits order. */
	if (config.is_MSB) {
		reg |= LPUART_STAT_MSBF_MASK;
	} else {
		reg &= ~LPUART_STAT_MSBF_MASK;
	}

	uart_base->STAT |= reg;

	/* Enable TX/RX base on configure structure. */
	reg = uart_base->CTRL;
	if (config.enableTx) {
		reg |= LPUART_CTRL_TE_MASK;
	}

	if (config.enableRx) {
		reg |= LPUART_CTRL_RE_MASK;
	}

	uart_base->CTRL = reg;
}

void Uart::SendByteBuffer(const uint8_t* data, uint32_t length) {
    while (length--)
    {
        while (!(uart_base->STAT & LPUART_STAT_TDRE_MASK));
        uart_base->DATA = *(data++);
    }
}

bool Uart::OpenTXPin(System::Pinout::Config& config) {
	if (System::Pinout::GetUartTXPinConfig(config, uart_base)) {
		System::Pinout::InitPin(config);
		return true;
	} else {
		return false;
	}
}

bool Uart::OpenRXPin(System::Pinout::Config& config) {
	if (System::Pinout::GetUartRXPinConfig(config, uart_base)) {
		System::Pinout::InitPin(config);
		return true;
	} else {
		return false;
	}
}

}
