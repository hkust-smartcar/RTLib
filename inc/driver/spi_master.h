/*
 * spi_master.h
 *
 *  Created on: Sep 17, 2018
 *      Author: LeeChunHei
 */

#ifndef INC_DRIVER_SPI_MASTER_H_
#define INC_DRIVER_SPI_MASTER_H_

#include "system/pinout/pinout.h"
#include "system/cmsis/access_layer/access_layer.h"

namespace Driver {

class SpiMaster {
public:
	struct Config {
		enum struct ClockPhase {
			kFirstEdge = 0U, /*!< CPHA=0. Data is captured on the leading edge of the SCK and changed on the
			 following edge.*/
			kSecondEdge = 1U /*!< CPHA=1. Data is changed on the leading edge of the SCK and captured on the
			 following edge.*/
		};
		enum struct PinConfig {
			kSdiInSdoOut, kSdiInSdiOut, kSdoInSdoOut, kSdoInSdiOut
		};
		enum struct DataOutConfig {
			kDataOutRetained = 0U, /*!< Data out retains last value when chip select is de-asserted */
			kDataOutTristate = 1U /*!< Data out is tristated when chip select is de-asserted */
		};
		System::Pinout::Name sck;
		System::Pinout::Name sdo = System::Pinout::Name::kDisable;
		System::Pinout::Name sdi = System::Pinout::Name::kDisable;
		System::Pinout::Name cs;
		uint32_t baud_rate = 500000; /*!< Baud Rate for LPSPI. */
		uint32_t bits_per_frame = 8; /*!< Bits per frame, minimum 8, maximum 4096.*/
		bool clock_polarity_active_low = false; /*!< Clock polarity. */
		ClockPhase clock_phase = ClockPhase::kFirstEdge; /*!< Clock phase. */
		bool transfer_LSB_first = false; /*!< MSB or LSB data shift direction. */

		uint32_t pcs_to_sck_delay_ns = 1000000000 / baud_rate * 2; /*!< PCS to SCK delay time in nanoseconds, setting to 0 sets the minimum delay.
		 It sets the boundary value if out of range.*/
		uint32_t last_sck_to_pcs_delay_ns = 1000000000 / baud_rate * 2; /*!< Last SCK to PCS delay time in nanoseconds, setting to 0 sets the minimum
		 delay. It sets the boundary value if out of range.*/
		uint32_t between_transfer_delay_ns = 1000000000 / baud_rate * 2; /*!< After the SCK delay time with nanoseconds, setting to 0 sets the minimum
		 delay. It sets the boundary value if out of range.*/
		bool chip_select_active_high = false; /*!< Desired PCS active high or low */
		PinConfig pin_config = PinConfig::kSdiInSdoOut; /*!< Configures which pins are used for input and output data during single bit transfers.*/
		DataOutConfig data_out_config = DataOutConfig::kDataOutRetained; /*!< Configures if the output data is tristated between accesses (LPSPI_PCS is negated). */
	};
	SpiMaster(const Config& config);
	inline void SendData(uint32_t data) {
		while (GetTxFifoCount() >= tx_fifo_size)
			;
		spi_base->TDR = data;
	}
	inline void GetData(uint32_t& data) {
		while (!GetRxFifoCount())
			;
		data = spi_base->RDR;
	}
	inline bool GetTransferState() {
		return (((spi_base->SR) >> 9) & 1);
	}
	inline void ClearTransferState() {
		spi_base->SR |= 1 << 9;
	}
	inline uint32_t GetTxFifoCount() {
		return ((spi_base->FSR & LPSPI_FSR_TXCOUNT_MASK) >> LPSPI_FSR_TXCOUNT_SHIFT);
	}
	inline uint32_t GetRxFifoCount() {
		return ((spi_base->FSR & LPSPI_FSR_RXCOUNT_MASK) >> LPSPI_FSR_RXCOUNT_SHIFT);
	}

private:
	void Reset();
	void SetMaster();
	void SetOnePcsPolarity(uint8_t chip_select, bool chip_select_active_high);
	uint32_t SetBaudRate(uint32_t baud_rate);
	void SetFifoWatermarks(uint8_t tx_watermark, uint8_t rx_watermark);
	void Enable(bool enable);
	void SetDelayTimes(uint32_t delay_ns, uint8_t type);
	void SetDelayScaler(uint32_t scaler, uint8_t type);
	void SetDummyData(uint8_t dummy_data);


	LPSPI_Type* spi_base;
	uint8_t rx_fifo_size;
	uint8_t tx_fifo_size;
};

}

#endif /* INC_DRIVER_SPI_MASTER_H_ */
