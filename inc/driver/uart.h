/*
 * uart.h
 *
 *  Created on: Aug 26, 2018
 *      Author: LeeChunHei
 */

#ifndef INC_DRIVER_UART_H_
#define INC_DRIVER_UART_H_

#include <stdint.h>
#include "system/cmsis/access_layer/access_layer.h"
#include "system/pinout/pinout.h"

namespace Driver {

class Uart {
public:
	struct Config {
		enum struct UartModule {
			kUart1 = 1, kUart2, kUart3, kUart4, kUart5, kUart6, kUart7
		};
		enum struct ParityMode {
			kDisableParity = 0x0U, /*!< Parity disabled */
			kEvenParity = 0x2U, /*!< Parity enabled, type even, bit setting: PE|PT = 10 */
			kOddParity = 0x3U, /*!< Parity enabled, type odd,  bit setting: PE|PT = 11 */
		};
		/*! @brief LPUART data bits count. */
		enum struct DataBits {
			kEightDataBits, kSevenDataBits
		};

		enum struct StopBit {
			kOneStopBit, kTwoStopBit
		};

		/*! @brief LPUART transmit CTS source. */
		enum struct TransmitCTSSource {
			kCtsSourcePin, kCtsSourceMatchResult
		};

		/*! @brief LPUART transmit CTS configure. */
		enum struct TransmitCTSConfig {
			kCtsSampleAtStart, kCtsSampleAtIdle
		};

		/*! @brief LPUART idle flag type defines when the receiver starts counting. */
		enum struct IdleType {
			kIdleTypeStartBit, kIdleTypeStopBit
		};

		/*! @brief LPUART idle detected configuration.
		 *  This structure defines the number of idle characters that must be received before
		 *  the IDLE flag is set.
		 */
		enum struct IdleConfig {
			kLPUART_IdleCharacter1, kLPUART_IdleCharacter2, kLPUART_IdleCharacter4, kLPUART_IdleCharacter8, kLPUART_IdleCharacter16, kLPUART_IdleCharacter32, kLPUART_IdleCharacter64, kLPUART_IdleCharacter128
		};

		UartModule uart_module;
		uint32_t baud_rate = 115200; /*!< LPUART baud rate  */
		ParityMode parity_mode = ParityMode::kDisableParity; /*!< Parity mode, disabled (default), even, odd */
		DataBits data_bits = DataBits::kEightDataBits; /*!< Data bits count, eight (default), seven */
		bool is_MSB = false; /*!< Data bits order, LSB (default), MSB */
		StopBit stop_bit = StopBit::kOneStopBit; /*!< Number of stop bits, 1 stop bit (default) or 2 stop bits  */
		uint8_t tx_fifo_watermark = 0; /*!< TX FIFO watermark, must not larger than 4*/
		uint8_t rx_fifo_watermark = 0; /*!< RX FIFO watermark, must not larger than 4*/
		bool enable_rx_rts = false; /*!< RX RTS enable */
		bool enable_tx_cts = false; /*!< TX CTS enable */
		TransmitCTSSource tx_cts_source = TransmitCTSSource::kCtsSourcePin; /*!< TX CTS source */
		TransmitCTSConfig tx_cts_config = TransmitCTSConfig::kCtsSampleAtStart; /*!< TX CTS configure */
		IdleType rx_idle_type = IdleType::kIdleTypeStartBit; /*!< RX IDLE type. */
		IdleConfig rx_idle_config = IdleConfig::kLPUART_IdleCharacter1; /*!< RX IDLE configuration. */
		bool enableTx = false; /*!< Enable TX */
		bool enableRx = false; /*!< Enable RX */
	};
	Uart(Config& config);
	inline void SendByte(const uint8_t& data) {
		uart_base->DATA = data;
	}
	inline uint8_t ReadByte() {
		return (bool) m_data_bits ? (uart_base->DATA & 0x7F) : uart_base->DATA;
	}
	void SendByteBuffer(const uint8_t* data, uint32_t length);
	uint8_t ReadByteBuffer(uint8_t* data);
	bool OpenTXPin(System::Pinout::Config& config);
	bool OpenRXPin(System::Pinout::Config& config);
private:
	LPUART_Type* uart_base;
	Config::DataBits m_data_bits;
};

}

#endif /* INC_DRIVER_UART_H_ */
