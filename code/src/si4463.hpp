/*
 *  AVR/Si4463 Wireless Doorbell
 *  Copyright (C) 2022  Andreas Stöckel
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <stddef.h>
#include <stdint.h>

#include <utils/buffer.hpp>
#include <utils/byte_suffix.hpp>

#include "platform.hpp"

namespace Si4463 {

static constexpr uint8_t POWER_UP = 0x02_B;
static constexpr uint8_t PART_INFO = 0x01_B;
static constexpr uint8_t SET_PROPERTY = 0x11_B;
static constexpr uint8_t GET_PROPERTY = 0x12_B;
static constexpr uint8_t GPIO_PIN_CFG = 0x13_B;
static constexpr uint8_t FIFO_INFO = 0x15_B;
static constexpr uint8_t GET_INT_STATUS = 0x20_B;
static constexpr uint8_t GET_MODEM_STATUS = 0x22_B;
static constexpr uint8_t START_TX = 0x31_B;
static constexpr uint8_t START_RX = 0x32_B;
static constexpr uint8_t REQUEST_DEVICE_STATE = 0x33_B;
static constexpr uint8_t CHANGE_STATE = 0x34_B;
static constexpr uint8_t READ_CMD_BUFF = 0x44_B;
static constexpr uint8_t WRITE_TX_FIFO = 0x66_B;
static constexpr uint8_t READ_RX_FIFO = 0x77_B;

static constexpr uint8_t INFINITE_LEN = 0x06_B;

enum class GPIOMode : uint8_t {
	Unchanged = 0x00_B,
	Tristate = 0x01_B,
	Drive_Low = 0x02_B,
	Drive_High = 0x03_B,
	Input = 0x04_B,
	Tx_State = 0x20_B,
	Rx_State = 0x21_B,
};

enum class State : uint8_t {
	Sleep = 0x01_B,
	Spi_Active = 0x02_B,
	Ready = 0x03_B,
	Ready2 = 0x04_B,
	Tx_Tune = 0x05_B,
	Rx_Tune = 0x06_B,
	Tx = 0x07_B,
	Rx = 0x08_B,
};

struct Transaction {
	Transaction(uint8_t cmd)
	{
		Platform::radio_select(false);
		Platform::radio_spi_send(cmd);
	}

	~Transaction() { Platform::radio_select(true); }

	template <typename T>
	Transaction &write(T x);

	template <typename T, typename... Ts>
	Transaction &write(T x, Ts... xs)
	{
		return write(x).write(xs...);
	}

	Transaction &wait_for_response()
	{
		// Try to read the response by issuing command 0x44
		while (true) {
			// Finalise the SPI transaction by toggling nselect and waiting for
			// a few cycles
			Platform::radio_select(true);
			Platform::delay_us(1);
			Platform::radio_select(false);

			// Send the READ_CMD_BUFF command to determine CTS
			Platform::radio_spi_send(READ_CMD_BUFF);
			if (Platform::radio_spi_send(0x00_B) == 0xFF_B) {
				break;  // We got CTS! Break, so we can handle the read()
				        // section
			}

			// Note that this loop might hang indefinetely. A watchdog should
			// be configured to reset the µC if not response is received.
		}
		return *this;
	}

	template <typename T>
	Transaction &read(T x);

	template <typename T, typename... Ts>
	Transaction &read(T x, Ts... xs)
	{
		return read(x).read(xs...);
	}
};

struct GPIOConfig {
	GPIOMode _gpio0 = GPIOMode::Unchanged;
	GPIOMode _gpio1 = GPIOMode::Unchanged;
	GPIOMode _gpio2 = GPIOMode::Unchanged;
	GPIOMode _gpio3 = GPIOMode::Unchanged;

	GPIOConfig &gpio0(bool on)
	{
		_gpio0 = on ? GPIOMode::Drive_High : GPIOMode::Drive_Low;
		return *this;
	}

	GPIOConfig &gpio1(bool on)
	{
		_gpio1 = on ? GPIOMode::Drive_High : GPIOMode::Drive_Low;
		return *this;
	}

	GPIOConfig &gpio2(bool on)
	{
		_gpio2 = on ? GPIOMode::Drive_High : GPIOMode::Drive_Low;
		return *this;
	}

	GPIOConfig &gpio3(bool on)
	{
		_gpio3 = on ? GPIOMode::Drive_High : GPIOMode::Drive_Low;
		return *this;
	}

	GPIOConfig &gpio0(GPIOMode mode)
	{
		_gpio0 = mode;
		return *this;
	}

	GPIOConfig &gpio1(GPIOMode mode)
	{
		_gpio1 = mode;
		return *this;
	}

	GPIOConfig &gpio2(GPIOMode mode)
	{
		_gpio2 = mode;
		return *this;
	}

	GPIOConfig &gpio3(GPIOMode mode)
	{
		_gpio3 = mode;
		return *this;
	}
};

void reset();

void init();

bool verify();

uint16_t part_number();

void clear_interrupts();

void clear_fifos(bool rx = true, bool tx = true);

uint8_t rx_fifo_count();

template <typename... Ts>
static void read_rx_fifo(Ts... xs)
{
	Transaction(READ_RX_FIFO).read(xs...);
}

uint8_t tx_fifo_space();

template <typename... Ts>
static void write_tx_fifo(Ts... xs)
{
	Transaction(WRITE_TX_FIFO).write(xs...);
}

void gpio_pin_cfg(const GPIOConfig &cfg);

void start_tx(uint16_t packet_size, uint8_t channel = 0);

void start_rx(uint16_t packet_size = 0, uint8_t channel = 0);

void set_state(State state);

State get_state();

uint8_t get_rssi();

void set_pkt_len(uint16_t len);

void set_pwr_lvl(uint8_t lvl = 0x7F);

void set_preamble_length(uint8_t len = 0x7F);

};  // namespace Si4463
