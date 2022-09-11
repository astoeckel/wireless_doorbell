#include <avr/pgmspace.h>

#include "si4463.hpp"
#include "si4463_config.h"

namespace Si4463 {

/******************************************************************************
 * Class Transaction template specialisations                                 *
 ******************************************************************************/

template <>
Transaction &Transaction::write(char x)
{
	Platform::radio_spi_send(x);
	return *this;
}

template <>
Transaction &Transaction::write(uint8_t x)
{
	Platform::radio_spi_send(x);
	return *this;
}

template <>
Transaction &Transaction::write(State x)
{
	Platform::radio_spi_send(static_cast<uint8_t>(x));
	return *this;
}


template <>
Transaction &Transaction::write(uint16_t x)
{
	Platform::radio_spi_send((x & 0xFF00) >> 8);
	Platform::radio_spi_send((x & 0x00FF) >> 0);
	return *this;
}

template <>
Transaction &Transaction::write(Buffer buf)
{
	for (size_t i = 0; i < buf.size(); i++) {
		Platform::radio_spi_send(buf[i]);
	}
	return *this;
}

template <>
Transaction &Transaction::write(ProgramSpaceBuffer buf)
{
	for (size_t i = 0; i < buf.size(); i++) {
		Platform::radio_spi_send(buf[i]);
	}
	return *this;
}

template <>
Transaction &Transaction::write(const char *s)
{
	while (*s) {
		Platform::radio_spi_send(*(s++));
	}
	return *this;
}

template <>
Transaction &Transaction::read(uint8_t *x)
{
	*x = Platform::radio_spi_send(0x00);
	return *this;
}

template <>
Transaction &Transaction::read(State *x)
{
	*x = static_cast<State>(Platform::radio_spi_send(0x00));
	return *this;
}

template <>
Transaction &Transaction::read(uint16_t *x)
{
	*x = (Platform::radio_spi_send(0x00) << 8);
	*x |= (Platform::radio_spi_send(0x00) << 0);
	return *this;
}

template <>
Transaction &Transaction::read(Buffer buf)
{
	for (size_t i = 0; i < buf.size(); i++) {
		buf[i] = Platform::radio_spi_send(0x00);
	}
	return *this;
}

template <>
Transaction &Transaction::read(Ignore ignore)
{
	for (size_t i = 0; i < ignore.size(); i++) {
		Platform::radio_spi_send(0x00);
	}
	return *this;
}

/******************************************************************************
 * Public API                                                                 *
 ******************************************************************************/

const uint8_t radio_config[] PROGMEM = RADIO_CONFIGURATION_DATA_ARRAY;

void reset()
{
	Platform::radio_reset(true);
	Platform::delay_us(100);
	Platform::radio_reset(false);
	Platform::delay_ms(10);
}

void init()
{
	Platform::radio_spi_init();

	size_t ptr = 0;
	while ((ptr + 1) < sizeof(radio_config)) {
		uint8_t len = pgm_read_byte(&radio_config[ptr + 0]);
		uint8_t cmd = pgm_read_byte(&radio_config[ptr + 1]);

		Transaction(cmd)
		    .write(ProgramSpaceBuffer{&radio_config[ptr + 2], uint8_t(len - 1)})
		    .wait_for_response();
		ptr += len + 1;
		_delay_ms(1);
	}
}

bool verify()
{
	bool valid = true;
	size_t ptr = 0;
	while ((ptr + 4) < sizeof(radio_config)) {
		uint8_t len = pgm_read_byte(&radio_config[ptr + 0]);
		uint8_t cmd = pgm_read_byte(&radio_config[ptr + 1]);
		if (cmd == SET_PROPERTY) {
			uint8_t group = pgm_read_byte(&radio_config[ptr + 2]);
			uint8_t num_props = pgm_read_byte(&radio_config[ptr + 3]);
			uint8_t start_prop = pgm_read_byte(&radio_config[ptr + 4]);

			uint8_t buf[16];
			Transaction(GET_PROPERTY)
			    .write(group, num_props, start_prop)
			    .wait_for_response()
			    .read(Buffer{&buf[0], num_props});

			for (size_t i = 0; i < num_props; i++) {
				uint8_t expected = pgm_read_byte(&radio_config[ptr + 5 + i]);
				if (buf[i] != expected) {
					valid = false;
				}
			}
		}
		ptr += len + 1;
	}
	return valid;
}

uint16_t part_number()
{
	uint16_t part;
	Transaction(PART_INFO).wait_for_response().read(Ignore(1), &part,
	                                                Ignore(5));
	return part;
}

void clear_interrupts()
{
	Transaction(GET_INT_STATUS)
	    .write(0xFF_B, 0xFF_B, 0x7F_B)
	    .wait_for_response()
	    .read(Ignore(8));
}

void clear_fifos(bool rx, bool tx)
{
	Transaction(FIFO_INFO)
	    .write(uint8_t((rx ? 2 : 0) | (tx ? 1 : 0)))
	    .wait_for_response()
	    .read(Ignore(2));
}

uint8_t rx_fifo_count()
{
	uint8_t res;
	Transaction(FIFO_INFO).write(0x00_B).wait_for_response().read(&res,
	                                                              Ignore(1));
	return res;
}

uint8_t tx_fifo_space()
{
	uint8_t res;
	Transaction(FIFO_INFO).write(0x00_B).wait_for_response().read(Ignore(1),
	                                                              &res);
	return res;
}

void gpio_pin_cfg(const GPIOConfig &cfg)
{
	Transaction(GPIO_PIN_CFG)
	    .write(
	        static_cast<uint8_t>(cfg._gpio0), static_cast<uint8_t>(cfg._gpio1),
	        static_cast<uint8_t>(cfg._gpio2), static_cast<uint8_t>(cfg._gpio3),
	        0_B,  // do not change nIRQ
	        0_B,  // do not change SDO
	        0_B   // maximum drive strength
	        )
	    .wait_for_response()
	    .read(Ignore(6));
}

void start_tx(uint16_t packet_size, uint8_t channel)
{
	Transaction(START_TX)
	    .write(channel, 0x30_B, packet_size, 0x00_B, 0x00_B)
	    .wait_for_response();
}

void start_rx(uint16_t packet_size, uint8_t channel)
{
	Transaction(START_RX)
	    .write(channel, 0x00_B, packet_size, State::Rx, State::Rx, State::Rx)
	    .wait_for_response();
}

void set_state(State state)
{
	Transaction(CHANGE_STATE).write(state).wait_for_response();
}

State get_state()
{
	State state;
	Transaction(REQUEST_DEVICE_STATE)
	    .wait_for_response()
	    .read(&state, Ignore(1));
	return state;
}

uint8_t get_rssi()
{
	uint8_t rssi;
	Transaction(GET_MODEM_STATUS)
	    .write(0x00_B)
	    .wait_for_response()
	    .read(Ignore(2), &rssi, Ignore(4));
	return rssi;
}

void set_pkt_len(uint16_t len)
{
	Transaction(SET_PROPERTY)
	    .write(0x12_B, 0x02_B, 0x11_B, len)
	    .wait_for_response();
}

void set_pwr_lvl(uint8_t lvl)
{
	Transaction(SET_PROPERTY)
	    .write(0x22_B, 0x01_B, 0x01_B, lvl)
	    .wait_for_response();
}

void set_preamble_length(uint8_t len)
{
	Transaction(SET_PROPERTY)
	    .write(0x10_B, 0x01_B, 0x00_B, len)
	    .wait_for_response();
}

}  // namespace Si4463
