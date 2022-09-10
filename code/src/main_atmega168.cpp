#include <platform.hpp>
#include <si4463.hpp>
#include <uart.hpp>

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stddef.h>
#include <stdint.h>
#include <util/atomic.h>
#include <util/delay.h>

#include <utils/buffer.hpp>
#include <utils/byte_suffix.hpp>

#include "platform.hpp"

/******************************************************************************
 * Watchdog timer                                                             *
 ******************************************************************************/

namespace Watchdog {
static inline void disable()
{
	MCUSR = 0;
	wdt_disable();
}

static inline void reset() { wdt_reset(); }

static inline void enable() { wdt_enable(WDTO_2S); }

}  // namespace Watchdog

/******************************************************************************
 * 100µs clock                                                                *
 ******************************************************************************/

namespace Clock {

static uint8_t _increment;
static uint16_t _ticks;

ISR(TIMER1_COMPA_vect)
{
	_ticks += _increment;  // Increment the 100µs tick counter
}

static inline void init()
{
	// Reset the timer configuration; disable PWM output and waveform generation
	TCCR1A = 0_B;
	TCCR1B = 0_B;
	TCCR1C = 0_B;
	TIMSK1 = 0_B;
	TCNT1 = 0;
	_ticks = 0;
	_increment = 1;

	// Generate an interrupt approximately every 100µs; operate the
	// counter in CTC mode
	OCR1A = (F_CPU / 10000);              // compare value
	TIMSK1 = (1 << OCIE1A);               // enable compare interrupt
	TCCR1B = (1 << WGM12) | (1 << CS10);  // CTC mode, highest possible speed
}

static inline void go_slow()
{
	// Generate an interrupt approximately every 1ms
	_increment = 10;
	TCNT1 = 0;
	OCR1A = (F_CPU / 1000);  // compare value
}

static inline void go_fast()
{
	// Generate an interrupt approximately every 100µs
	_increment = 1;
	TCNT1 = 0;
	OCR1A = (F_CPU / 10000);  // compare value
}

static inline uint16_t ticks()
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { return _ticks; }
	__builtin_unreachable();
}

};  // namespace Clock

/******************************************************************************
 * Interrupts                                                                 *
 ******************************************************************************/

namespace Events {
struct Event {
	bool valid;
	uint16_t time;

	operator bool() const { return valid; }
};

Event pulse, radio_irq;

void init()
{
	pulse = Event{.valid = false, .time = 0};
	radio_irq = Event{.valid = false, .time = 0};
}

Event get_and_reset_pulse_event()
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		Event res = pulse;
		pulse.valid = false;
		return res;
	}
	__builtin_unreachable();
}

Event get_and_reset_radio_irq_event()
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		Event res = radio_irq;
		radio_irq.valid = false;
		return res;
	}
	__builtin_unreachable();
}

ISR(INT0_vect)
{
	if (!Platform::sample_radio_irq_pin()) {
		radio_irq = Event{.valid = true, .time = Clock::ticks()};
	}
}

ISR(INT1_vect)
{
	if (!Platform::sample_pulse_pin()) {
		pulse = Event{.valid = true, .time = Clock::ticks()};
	}
}
};  // namespace Events

namespace Sleep {
void init()
{
	set_sleep_mode(SLEEP_MODE_IDLE);  // Lowest possible sleep mode where the
	                                  // timer still runs
}

void enter()
{
	sleep_enable();
	sei();
	sleep_cpu();
	sleep_disable();
}

}  // namespace Sleep

/******************************************************************************
 * MAIN PROGRAM                                                               *
 ******************************************************************************/

/**
 * The `DoorbellButtonStateMachine` is responsible for detecting doorbell button
 * presses.
 *
 * If the doorbell button is depressed, there should be a periodic 50 Hz signal
 * at PIND3; the timing of this signal is precisely captured by the interrupt
 * code above.
 *
 * If the doorbell button is pressed, this periodic signal will go away. This
 * code makes sure that a doorbell button-press event is only triggered if
 *
 *    1. There was a periodic 50-60 Hz signal before the signal went away.
 *    2. The signal goes away for a minimum delay.
 *    3. The corresponding hardware pin is actually high when after the minimum
 *       delay.
 *
 * This is meant to minimize false-positive doorbell triggers.
 */
struct DoorbellButton {
	enum State {
		UNKNOWN,
		PRESSED,
		DEPRESSED,
	};

	static constexpr uint16_t MIN_DELTA = 156;      // 1 / 60 Hz - 1ms
	static constexpr uint16_t MAX_DELTA = 210;      // 1 / 50 Hz + 1ms
	static constexpr uint16_t DEPRESS_DELAY = 800;  // 80 ms (4 cycles @ 50 Hz)
	static constexpr uint8_t AC_CARRIER_SYNC = 5;   // 5 cycles to synchronise

	uint8_t ac_carrier_pulse_count = 0;
	uint16_t last_falling_edge_ticks = 0;

	State update(uint16_t cur_ticks, const Events::Event &pulse_event,
	             bool pin_state)
	{
		// Check whether the delay between the last detected pulse and the
		// current pulse is larger than 65ms (> 3 cycles). If we are locked
		// (i.e., we detected many valid 50/60 Hz pulses previously), then this
		// should mean that the doorbell button was pressed.
		const uint16_t delta = cur_ticks - last_falling_edge_ticks;
		if ((delta > DEPRESS_DELAY) &&
		    (ac_carrier_pulse_count >= AC_CARRIER_SYNC)) {
			// We were synchronised. Someone must have rang the doorbell!
			ac_carrier_pulse_count = 0;

			// Reset last_falling_edge_ticks to prevent overflows of delta
			last_falling_edge_ticks = cur_ticks;

			// Is the pulse pin still high? If not, we might just be running
			// out of power.
			if (pin_state) {
				return PRESSED;
			}
		}
		else if (pulse_event) {
			const uint16_t delta = pulse_event.time - last_falling_edge_ticks;
			last_falling_edge_ticks = pulse_event.time;

			// Were the falling edges 50 or 60 Hz apart? If we get at least 10
			// such pulses, we count that as synchronised.
			if (delta >= MIN_DELTA && delta <= MAX_DELTA) {
				if (ac_carrier_pulse_count < AC_CARRIER_SYNC) {
					ac_carrier_pulse_count++;
				}
			}
			else {
				// Something weird happened, we are no longer synchronised
				ac_carrier_pulse_count = 0;
			}
		}
		return (ac_carrier_pulse_count >= AC_CARRIER_SYNC) ? DEPRESSED
		                                                   : UNKNOWN;
	}

	bool can_sleep()
	{
		return ac_carrier_pulse_count == AC_CARRIER_SYNC;
	}
};

struct DoorbellButtonTxStateMachine {
	static constexpr uint8_t FORCE_IVAL_SEC = 60;
	static constexpr uint8_t FORCE_IVAL_SEC_UNKNOWN = 5;
	static constexpr uint16_t REPEAT = 5;
	static constexpr uint16_t IVAL_MS = 500;  // 50 ms

	DoorbellButton::State last_doorbell_state = DoorbellButton::UNKNOWN;
	uint8_t tx_doorbell_seq_idx = 0;
	bool tx_doorbell_forced = false;
	bool had_good_state_once = false;
	uint16_t tx_doorbell_event_ticks = 0;
	uint8_t last_tx_seconds = FORCE_IVAL_SEC;
	uint16_t last_seconds_ticks = 0;

	template <typename T>
	void update(uint16_t cur_ticks, DoorbellButton::State button_state,
	            T &radio_tx_buf)
	{
		// Second counter; force an update every FORCE_IVAL_SEC seconds.
		if ((cur_ticks - last_seconds_ticks > 10000) && had_good_state_once) {
			last_tx_seconds++;
			last_seconds_ticks = cur_ticks;
			if (((button_state == DoorbellButton::UNKNOWN) &&
			     (last_tx_seconds > FORCE_IVAL_SEC_UNKNOWN)) ||
			    (last_tx_seconds > FORCE_IVAL_SEC)) {
				tx_doorbell_seq_idx = REPEAT;
				tx_doorbell_forced = true;
				last_tx_seconds = 0;
			}
		}

		// If there was a transition in button state, print a message and
		// update the output
		if ((button_state != DoorbellButton::UNKNOWN) &&
		    (button_state != last_doorbell_state)) {
			had_good_state_once = true;
			last_doorbell_state = button_state;
			switch (button_state) {
				case DoorbellButton::PRESSED:
					UART::print_json_msg("info", "doorbell button pressed");
					tx_doorbell_seq_idx = 2 * REPEAT;
					tx_doorbell_event_ticks = cur_ticks - IVAL_MS;
					tx_doorbell_forced = false;
					break;
				case DoorbellButton::DEPRESSED:
					UART::print_json_msg("info", "doorbell button depressed");
					if (tx_doorbell_seq_idx < REPEAT) {
						tx_doorbell_seq_idx = REPEAT;
						tx_doorbell_forced = false;
					}
					break;
				default:
					break;
			}
		}

		// Transmit any outstanding the doorbell events
		if (tx_doorbell_seq_idx &&
		    (cur_ticks - tx_doorbell_event_ticks) >= IVAL_MS) {
			tx_doorbell_event_ticks = cur_ticks;

			bool is_on = (tx_doorbell_seq_idx > REPEAT);
			bool is_off = (last_doorbell_state == DoorbellButton::DEPRESSED) ||
			              (tx_doorbell_forced);
			if (is_on || is_off) {
				uint8_t seq = (is_on ? 2 : 1) * REPEAT - tx_doorbell_seq_idx;
				radio_tx_buf.push("DBL,");
				if (is_on) {
					radio_tx_buf.push('1');
				}
				else {
					if (tx_doorbell_forced &&
					    (button_state == DoorbellButton::UNKNOWN)) {
						radio_tx_buf.push('?');
					}
					else {
						radio_tx_buf.push('0');
					}
				}
				radio_tx_buf.push(',');
				radio_tx_buf.push('0' + seq);
				radio_tx_buf.push("\r\n");
				tx_doorbell_seq_idx--;
				last_tx_seconds = 0;
			}
		}
	}

	bool can_sleep()
	{
		return tx_doorbell_seq_idx == 0;
	}
};

struct RadioStateMachine {
	uint16_t first_radio_tx_ticks = 0;
	bool has_radio_tx = false;
	Si4463::State known_state = Si4463::State::Sleep;
	Si4463::State target_state = Si4463::State::Ready;

	template <typename T1, typename T2>
	void update(uint16_t cur_ticks, T1 &radio_rx_buf, T2 &radio_tx_buf,
	            bool can_sleep)
	{
		// Is there data in the transmit buffer? If yes, either wait for
		// 20ms or until the buffer is half full
		const uint8_t radio_tx_level = radio_tx_buf.level();
		if (radio_tx_level && !has_radio_tx) {
			first_radio_tx_ticks = cur_ticks;
			has_radio_tx = true;
		}
		bool need_tx =
		    radio_tx_level && ((radio_tx_level >= radio_tx_buf.size() / 2) ||
		                       ((cur_ticks - first_radio_tx_ticks) > 400));

		if (known_state != target_state) {
			known_state = Si4463::get_state();
		}
		if (known_state == Si4463::State::Ready) {
			if (need_tx) {
				const uint8_t n_send =
				    (radio_tx_level > 63) ? 63 : radio_tx_level;
				const uint8_t p1 = radio_tx_buf.read_ptr;
				const uint8_t p2 = (radio_tx_buf.read_ptr + n_send) &
				                   (radio_tx_buf.size() - 1);
				if (p1 < p2) {
					Si4463::write_tx_fifo(
					    n_send, Buffer(radio_tx_buf.data + p1, n_send));
				}
				else {
					Si4463::write_tx_fifo(n_send,
					                      Buffer(radio_tx_buf.data + p1,
					                             radio_tx_buf.size() - p1),
					                      Buffer(radio_tx_buf.data, p2));
				}
				radio_tx_buf.read_ptr = p2;

				// Set the packet length
				Si4463::set_pkt_len(n_send);

				// Start the transmission; wait until we're in the ready state
				Si4463::start_tx(0);
				known_state = Si4463::State::Tx;
				target_state = Si4463::State::Ready;

				// Reset the maximum packet length for the receiver
				Si4463::set_pkt_len(63);

				// Determine whether we still have data in the send buffer
				has_radio_tx = (n_send != radio_tx_level);
			}
			else if (!can_sleep) {
				// If we don't need to transmit something, go to the RX mode
				Si4463::start_rx(0);
				target_state = Si4463::State::Rx;
			}
		}
		else if (known_state == Si4463::State::Rx) {
			// Put all the received data into the RX buffer
			const uint8_t radio_rx_level = radio_rx_buf.level();
			const uint8_t rx_count = Si4463::rx_fifo_count();
			const uint8_t rx_space = radio_rx_buf.size() - radio_rx_level;
			const uint8_t n_recv = (rx_count < rx_space) ? rx_count : rx_space;
			if (n_recv) {
				const uint8_t p1 = radio_rx_buf.write_ptr;
				const uint8_t p2 = (radio_rx_buf.write_ptr + n_recv) &
				                   (radio_rx_buf.size() - 1);
				if (p1 < p2) {
					Si4463::read_rx_fifo(
					    Buffer(radio_rx_buf.data + p1, n_recv));
				}
				else {
					Si4463::read_rx_fifo(Buffer(radio_rx_buf.data + p1,
					                            radio_rx_buf.size() - p1),
					                     Buffer(radio_rx_buf.data, p2));
				}
				radio_rx_buf.write_ptr = p2;
			}

			if (!rx_count && (need_tx || can_sleep) &&
			    (Si4463::get_rssi() < 0x30)) {
				Si4463::set_state(Si4463::State::Ready);
				target_state = Si4463::State::Ready;
				known_state = Si4463::State::Ready;
			}
		}
	}

	bool can_sleep() { return !has_radio_tx && (known_state == target_state); }
};

int main()
{
	// Reset the watchdog
	Watchdog::disable();
	Sleep::init();

	Platform::init_gpio();
	Watchdog::enable();
	Events::init();
	Clock::init();
	UART::init();

	Si4463::reset();
	Si4463::init();
	if (Si4463::part_number() != 0x4463) {
		UART::print_json_msg("error", "unknown part");
		while (true) {}
	}
	if (!Si4463::verify()) {
		UART::print_json_msg("error", "config verification");
		while (true) {}
	}
	UART::print_json_msg("info", "initialized radio");

	// Clear interrupts and fifos
	Si4463::set_state(Si4463::State::Ready);
	Si4463::clear_interrupts();
	Si4463::clear_fifos();

	// Configure GPIO1 and 2 to drive the T/R switch
	Si4463::gpio_pin_cfg(Si4463::GPIOConfig()
	                         .gpio0(Si4463::GPIOMode::Tx_State)
	                         .gpio1(Si4463::GPIOMode::Rx_State)
	                         .gpio2(Si4463::GPIOMode::Tx_State)
	                         .gpio3(Si4463::GPIOMode::Rx_State));

	// Enable µC interrupts
	sei();

	uint8_t flash_leds = 0;
	uint16_t button_press_delay = 0;
	uint16_t button_press_ticks = 0;
	uint8_t seq_no = 0;
	uint8_t ping_state = 0;
	uint8_t pong_state = 0;
	uint16_t flash_leds_ticks = 0;

	bool last_can_sleep = false;
	uint8_t active_led_phase = 0;
	uint16_t last_active_led_ticks = 0;

	DoorbellButton doorbell;
	DoorbellButtonTxStateMachine doorbell_tx;

	RadioStateMachine radio;
	Ringbuffer<64> radio_tx_buf;
	Ringbuffer<64> radio_rx_buf;

	while (true) {
		// Yes, we are alive!
		wdt_reset();

		// Fetch the current time in ticks (one tick = 100µs)
		uint16_t cur_ticks = Clock::ticks();

		// Update the doorbell state
		auto button_state =
		    doorbell.update(cur_ticks, Events::get_and_reset_pulse_event(),
		                    Platform::sample_pulse_pin());
		doorbell_tx.update(cur_ticks, button_state, radio_tx_buf);

		auto radio_irq_event = Events::get_and_reset_radio_irq_event();
		if (radio_irq_event) {
			UART::print_json_msg("info", "irq event");
		}

		if (!button_press_delay) {
			if (cur_ticks - button_press_ticks > 2500) {
				button_press_delay = 1;
			}
		}

		if (!(PIND & (1 << 5)) && button_press_delay) {
			button_press_delay = 0;
			button_press_ticks = cur_ticks;
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				radio_tx_buf.push('P');
				radio_tx_buf.push('I');
				radio_tx_buf.push('N');
				radio_tx_buf.push('G');
				radio_tx_buf.push(' ');
				radio_tx_buf.push(UART::hex_digit((seq_no & 0xF0) >> 4));
				radio_tx_buf.push(UART::hex_digit((seq_no & 0x0F) >> 0));
				radio_tx_buf.push('\n');
				radio_tx_buf.push('\r');
			}
			seq_no++;
		}

		if (flash_leds > 0) {
			if (cur_ticks - flash_leds_ticks > 1000) {
				flash_leds_ticks = cur_ticks;
				flash_leds--;

				if (flash_leds != 0) {
					Si4463::GPIOMode mode = (flash_leds & 1)
					                            ? Si4463::GPIOMode::Drive_High
					                            : Si4463::GPIOMode::Drive_Low;
					Si4463::gpio_pin_cfg(Si4463::GPIOConfig()
					                         .gpio0(mode)
					                         .gpio1(Si4463::GPIOMode::Unchanged)
					                         .gpio2(Si4463::GPIOMode::Unchanged)
					                         .gpio3(mode));
				}
				else {
					Si4463::gpio_pin_cfg(
					    Si4463::GPIOConfig()
					        .gpio0(Si4463::GPIOMode::Tx_State)
					        .gpio1(Si4463::GPIOMode::Unchanged)
					        .gpio2(Si4463::GPIOMode::Unchanged)
					        .gpio3(Si4463::GPIOMode::Rx_State));
				}
			}
		}
		else {
			if ((cur_ticks - last_active_led_ticks) > 1000) {
				last_active_led_ticks = cur_ticks;
				if (active_led_phase == 10) {
					active_led_phase = 0;
					Si4463::gpio_pin_cfg(
					    Si4463::GPIOConfig()
					        .gpio0(Si4463::GPIOMode::Unchanged)
					        .gpio1(Si4463::GPIOMode::Unchanged)
					        .gpio2(Si4463::GPIOMode::Unchanged)
					        .gpio3(Si4463::GPIOMode::Drive_High));
				}
				else if (active_led_phase == 1) {
					Si4463::gpio_pin_cfg(
					    Si4463::GPIOConfig()
					        .gpio0(Si4463::GPIOMode::Unchanged)
					        .gpio1(Si4463::GPIOMode::Unchanged)
					        .gpio2(Si4463::GPIOMode::Unchanged)
					        .gpio3(Si4463::GPIOMode::Rx_State));
				}
				active_led_phase++;
			}
		}

		// Send/receive data
		radio.update(cur_ticks, radio_rx_buf, radio_tx_buf, last_can_sleep);

		while (radio_rx_buf.level()) {
			uint8_t buf = radio_rx_buf.pop();
			UART::putc(buf);
			const char *cmp_ping = "PING ";
			const char *cmp_pong = "PONG ";
			if (buf == '\r' || buf == '\n') {
				if (ping_state >= 5) {
					radio_tx_buf.push('\n');
					radio_tx_buf.push('\r');
				}
				ping_state = 0;
			}
			else if (ping_state < 5 && buf == cmp_ping[ping_state]) {
				ping_state++;
			}
			else if (ping_state >= 5) {
				if (ping_state == 5) {
					radio_tx_buf.push('P');
					radio_tx_buf.push('O');
					radio_tx_buf.push('N');
					radio_tx_buf.push('G');
					radio_tx_buf.push(' ');
				}
				radio_tx_buf.push(buf);
				ping_state++;
			}

			if (buf == '\r' || buf == '\n') {
				if (pong_state >= 5) {
					flash_leds = 10;
					flash_leds_ticks = cur_ticks;
				}
				pong_state = 0;
			}
			else if (pong_state < 5 && buf == cmp_pong[pong_state]) {
				pong_state++;
			}
		}

		bool can_sleep = doorbell.can_sleep() && doorbell_tx.can_sleep() &&
		                 radio.can_sleep() && !flash_leds;
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			if (!can_sleep && last_can_sleep) {
				Clock::go_fast();
			}
			else if (can_sleep) {
				Clock::go_slow();
				Sleep::enter();
			}
		}
		last_can_sleep = can_sleep;
	}
}
