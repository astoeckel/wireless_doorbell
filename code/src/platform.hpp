#pragma once

#define F_CPU 14745600ULL

#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>


namespace Platform {

static inline void delay_us(double us) {
	_delay_us(us);
}

static inline void delay_ms(double ms) {
	_delay_ms(ms);
}

static inline void set_bit(volatile uint8_t &port, uint8_t pin, bool x)
{
	port = x ? (port | (1 << pin)) : (port & ~(1 << pin));
}

static inline bool get_bit(volatile uint8_t &pinreg, uint8_t pin)
{
	return (pinreg & (1 << pin)) == (1 << pin);
}

static inline bool sample_radio_irq_pin() {
	return get_bit(PIND, 2);
}

static inline bool sample_pulse_pin() {
	return get_bit(PIND, 3);
}

static inline void init_gpio() {
	// Port A: N/A

	// Port B:
	// Pin 0: CLKO
	// Pin 1: NC
	// Pin 2: Si4463 SS    O
	// Pin 3: Si4463 MOSI  O
	// Pin 4: Si4463 MISO  I
	// Pin 5: Si4463 SCK   O
	// Pin 6: N/A (XTAL1)
	// Pin 7: N/A (XTAL2)
	DDRB = 0b0010'1101;
	PORTB = 0b1101'0000;

	// Port C:
	// Pin 0: NC
	// Pin 1: NC
	// Pin 2: NC
	// Pin 3: NC
	// Pin 4: NC
	// Pin 5: NC
	// Pin 6: N/A (RESET)
	// Pin 7: NC
	DDRC = 0b0000'0000;
	PORTC = 0b1111'1111;

	// Port D:
	// Pin 0: RXD I
	// Pin 1: TXD O
	// Pin 2: Si4463 IRQ   I
	// Pin 3: PULSE I (already has a hw pull-up)
	// Pin 4: Si4463 SDN   O
	// Pin 5: BUTTON I
	// Pin 6: NC
	// Pin 7: DBG O
	DDRD  = 0b1001'0010;
	PORTD = 0b0110'0101;

	// Setup IRQs on PIND2 (INT0) and PIND3 (INT1)
	EICRA = (1 << ISC11) | (1 << ISC01);
	EIMSK = (1 << INT1) | (1 << INT0);
}

static inline void radio_reset(bool value) { set_bit(PORTD, 4, value); }

static inline void radio_select(bool value) { set_bit(PORTB, 2, value); }

static inline void radio_spi_init() {
	// Enable SPI with CHPA = 0; use F_CPU / 2
	SPCR = (1 << SPE) | (1 << MSTR);
	//SPSR = (1 << SPI2X);
}

static inline uint8_t radio_spi_send(uint8_t x)
{
	SPDR = x;
	while (!(SPSR & (1 << SPIF))) {}
	return SPDR;
}

}
