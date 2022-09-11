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

static inline void enable() { wdt_enable(WDTO_120MS); }

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
	// Generate an interrupt approximately every 2ms
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
		Unknown,
		Pressed,
		Depressed,
	};

	static constexpr uint16_t MIN_DELTA = 126;      // 1 / 60 Hz - 4ms
	static constexpr uint16_t MAX_DELTA = 240;      // 1 / 50 Hz + 4ms
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
				return Pressed;
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
		return (ac_carrier_pulse_count >= AC_CARRIER_SYNC) ? Depressed
		                                                   : Unknown;
	}

	bool can_sleep() { return ac_carrier_pulse_count == AC_CARRIER_SYNC; }
};

struct DebouncedIO {
	/**
	 * Time for which the I/O pin needs to be stable.
	 */
	static constexpr uint16_t DEBOUNCE_TICKS = 500;  // 50ms

	bool cur_state = true;
	bool last_raw_in = false;
	uint16_t last_edge_ticks = 0;

	bool update(uint16_t cur_ticks, bool raw_in)
	{
		if (raw_in != last_raw_in) {
			last_raw_in = raw_in;
			last_edge_ticks = cur_ticks;
		}
		if (cur_ticks - last_edge_ticks >= DEBOUNCE_TICKS) {
			last_edge_ticks = cur_ticks - DEBOUNCE_TICKS;
			cur_state = raw_in;
		}
		return cur_state;
	}

	bool can_sleep(uint16_t cur_ticks)
	{
		return (cur_ticks - last_edge_ticks) >= DEBOUNCE_TICKS;
	}
};

struct OnboardButton {
	/**
	 * Number of seconds the button has to be pushed down for the board to
	 * reset.
	 */
	static constexpr uint8_t RESET_DUR_SEC = 3;  // 3s

	/**
	 * Number of seconds for which pressing the button forces non-power-save
	 * mode.
	 */
	static constexpr uint8_t NO_SLEEP_DUR_SEC = 60;  // 60s

	enum Action {
		DoNothing,
		SendPing,
		Reset,
	};

	uint16_t last_button_press_sec_ticks = 0;
	uint8_t no_sleep_secs = 0;
	uint8_t reset_secs = 0;
	bool last_is_pressed = false;

	Action update(uint16_t cur_ticks, bool pin_state)
	{
		// Fetch the actual button state (high = depressed, low = pressed)
		const bool is_pressed = !pin_state;

		if (is_pressed != last_is_pressed) {
			last_is_pressed = is_pressed;
			last_button_press_sec_ticks = cur_ticks;
			if (is_pressed) {
				no_sleep_secs = NO_SLEEP_DUR_SEC;
				reset_secs = RESET_DUR_SEC;
				return Action::SendPing;
			}
			else {
				reset_secs = 0;
			}
		}

		// Count the number of seconds since the last button press
		uint16_t delta_sec = cur_ticks - last_button_press_sec_ticks;
		if (delta_sec > 10000) {
			last_button_press_sec_ticks += 10000;
			if (no_sleep_secs > 0) {
				no_sleep_secs--;
			}
			if (last_is_pressed && reset_secs > 0) {
				reset_secs--;
				if (reset_secs == 0) {
					return Action::Reset;
				}
			}
		}

		return Action::DoNothing;
	}

	bool can_sleep() { return !last_is_pressed && (no_sleep_secs == 0); }
};

struct DoorbellButtonTxStateMachine {
	static constexpr uint8_t FORCE_IVAL_SEC = 120;
	static constexpr uint8_t FORCE_IVAL_SEC_UNKNOWN = 10;
	static constexpr uint16_t REPEAT = 5;
	static constexpr uint16_t IVAL_MS = 500;  // 50 ms

	DoorbellButton::State last_doorbell_state = DoorbellButton::Unknown;
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
			last_seconds_ticks += 10000;
			if (((button_state == DoorbellButton::Unknown) &&
			     (last_tx_seconds > FORCE_IVAL_SEC_UNKNOWN)) ||
			    (last_tx_seconds > FORCE_IVAL_SEC)) {
				tx_doorbell_seq_idx = REPEAT;
				tx_doorbell_forced = true;
				last_tx_seconds = 0;
			}
		}

		// If there was a transition in button state, print a message and
		// update the output
		if ((button_state != DoorbellButton::Unknown) &&
		    (button_state != last_doorbell_state)) {
			had_good_state_once = true;
			last_doorbell_state = button_state;
			switch (button_state) {
				case DoorbellButton::Pressed:
					UART::print_json_msg("info", "doorbell button pressed");
					tx_doorbell_seq_idx = 2 * REPEAT;
					tx_doorbell_event_ticks = cur_ticks - IVAL_MS;
					tx_doorbell_forced = false;
					break;
				case DoorbellButton::Depressed:
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
			bool is_off = (last_doorbell_state == DoorbellButton::Depressed) ||
			              (tx_doorbell_forced);
			if (is_on || is_off) {
				uint8_t seq = (is_on ? 2 : 1) * REPEAT - tx_doorbell_seq_idx;
				radio_tx_buf.push("$DBL,");
				if (is_on) {
					radio_tx_buf.push('1');
				}
				else {
					if (tx_doorbell_forced &&
					    (button_state == DoorbellButton::Unknown)) {
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

	bool can_sleep() { return tx_doorbell_seq_idx == 0; }
};

/**
 * The `RadioStateMachine` class is responsible for transitioning the
 * wireless transceiver IC into the right state, sending the bytes stored
 * in the transmit buffer, and writing received bytes to a receive buffer.
 */
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

struct LEDStateMachine {
	enum State {
		Off,
		On,
		BlinkSlowlyOn,
		BlinkSlowlyOff,
		BlinkQuickly,
	};

	uint16_t last_led_ticks = 0;
	State event_state = State::Off;
	State base_state = State::Off;
	uint8_t led_phase = 0;
	uint8_t event_len = 0;
	bool is_on = false;

	bool update(uint16_t cur_ticks)
	{
		if (cur_ticks - last_led_ticks > 1000) {
			last_led_ticks += 1000;
			led_phase++;
			if (led_phase == 10) {
				led_phase = 0;
			}
			if (event_len > 0) {
				event_len--;
			}
			State state = event_len ? event_state : base_state;
			switch (state) {
				case State::Off:
					is_on = false;
					break;
				case State::On:
					is_on = true;
					break;
				case State::BlinkSlowlyOff:
					is_on = (led_phase == 9);
					break;
				case State::BlinkSlowlyOn:
					is_on = (led_phase != 9);
					break;
				case State::BlinkQuickly:
					is_on = (led_phase & 1) == 0;
					break;
			}
		}
		return is_on;
	}

	void set_state(State new_state, uint8_t new_event_len = 0)
	{
		if (new_event_len) {
			if ((new_event_len > event_len) || (new_state != event_state)) {
				event_state = new_state;
				event_len = new_event_len;
			}
		}
		else {
			base_state = new_state;
		}
	}

	bool can_sleep() { return event_len == 0; }
};

struct RadioLEDStateMachine {
	LEDStateMachine left_led;
	LEDStateMachine right_led;
	bool last_left_on = false;
	bool last_right_on = false;

	void update(uint16_t cur_ticks)
	{
		bool left_on = left_led.update(cur_ticks);
		bool right_on = right_led.update(cur_ticks);
		if ((left_on != last_left_on) || (right_on != last_right_on)) {
			last_left_on = left_on;
			last_right_on = right_on;
			auto gpio0 = right_on ? Si4463::GPIOMode::Drive_High
			                      : Si4463::GPIOMode::Drive_Low;
			auto gpio3 = left_on ? Si4463::GPIOMode::Drive_High
			                     : Si4463::GPIOMode::Drive_Low;
			Si4463::gpio_pin_cfg(Si4463::GPIOConfig()
			                         .gpio0(gpio0)
			                         .gpio1(Si4463::GPIOMode::Unchanged)
			                         .gpio2(Si4463::GPIOMode::Unchanged)
			                         .gpio3(gpio3));
		}
	}

	bool can_sleep() { return left_led.can_sleep() && right_led.can_sleep(); }

	void update_blink_pattern(bool has_tx_data, bool has_rx_data,
	                          bool last_can_sleep, bool synced, bool in_rx)
	{
		if (has_tx_data) {
			right_led.set_state(LEDStateMachine::BlinkQuickly, 10);
		}
		if (has_rx_data) {
			left_led.set_state(LEDStateMachine::BlinkQuickly, 10);
		}
		if (in_rx) {
			if (synced) {
				left_led.set_state(LEDStateMachine::BlinkSlowlyOn);
			}
			else {
				left_led.set_state(LEDStateMachine::On);
			}
		}
		else {
			if (last_can_sleep) {
				left_led.set_state(LEDStateMachine::BlinkSlowlyOff);
			}
			else {
				left_led.set_state(LEDStateMachine::Off);
			}
		}
	}
};

static bool is_valid_char(uint8_t c)
{
	return ((c >= 'A') && (c <= 'Z')) || (c == ',') || (c == '?') ||
	       ((c >= '0') && (c <= '9'));
}

static bool strcmp(const char *s1, const char *s2)
{
	do {
		if (*(s1++) != *(s2++)) {
			return false;
		}
	} while (*s1 && *s2);
	return *s1 == *s2;
}

int main()
{
	bool last_can_sleep = false;

	DebouncedIO btn_onboard_debounce;
	OnboardButton btn_onboard;
	DoorbellButton btn_doorbell;
	DoorbellButtonTxStateMachine doorbell_tx;
	RadioLEDStateMachine radio_leds;

	char pkt_msg[16];
	uint8_t pkt_ptr = 0;

	RadioStateMachine radio;
	Ringbuffer<32> radio_tx_buf;
	Ringbuffer<32> radio_rx_buf;

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
	UART::print_json_msg("info", "power on reset");
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

	// For the first 500ms, and while the button is held, turn both LEDs on;
	// this is to give a visual indication of the reset happening.
	Si4463::gpio_pin_cfg(Si4463::GPIOConfig()
	                         .gpio0(Si4463::GPIOMode::Drive_High)
	                         .gpio1(Si4463::GPIOMode::Unchanged)
	                         .gpio2(Si4463::GPIOMode::Unchanged)
	                         .gpio3(Si4463::GPIOMode::Drive_High));
	Watchdog::disable();
	_delay_ms(500);
	while (!Platform::sample_button_pin()) {
		_delay_ms(10);
	};
	Watchdog::enable();
	Si4463::gpio_pin_cfg(Si4463::GPIOConfig()
	                         .gpio0(Si4463::GPIOMode::Drive_Low)
	                         .gpio1(Si4463::GPIOMode::Unchanged)
	                         .gpio2(Si4463::GPIOMode::Unchanged)
	                         .gpio3(Si4463::GPIOMode::Drive_Low));
	Watchdog::disable();
	_delay_ms(500);
	Watchdog::enable();

	// Configure GPIO1 and GPIO2 to drive the Tx/Rx switch
	Si4463::gpio_pin_cfg(Si4463::GPIOConfig()
	                         .gpio0(Si4463::GPIOMode::Unchanged)
	                         .gpio1(Si4463::GPIOMode::Rx_State)
	                         .gpio2(Si4463::GPIOMode::Tx_State)
	                         .gpio3(Si4463::GPIOMode::Unchanged));

	// Enable µC interrupts
	sei();

	while (true) {
		// Yes, we are alive!
		wdt_reset();

		// Fetch the current time in ticks (one tick = 100µs)
		uint16_t cur_ticks = Clock::ticks();

		// Update the onboard button state
		const bool onboard_btn_debounced = btn_onboard_debounce.update(
		    cur_ticks, Platform::sample_button_pin());
		switch (btn_onboard.update(cur_ticks, onboard_btn_debounced)) {
			case OnboardButton::DoNothing:
				break;
			case OnboardButton::SendPing:
				radio_tx_buf.push("$PING\n\r");
				break;
			case OnboardButton::Reset:
				UART::print_json_msg("info", "pending reset");
				while (true) {}
				break;
		}

		// Update the doorbell state
		auto button_state =
		    btn_doorbell.update(cur_ticks, Events::get_and_reset_pulse_event(),
		                        Platform::sample_pulse_pin());
		doorbell_tx.update(cur_ticks, button_state, radio_tx_buf);

		// Send/receive data and update the LEDs
		bool has_tx_data = radio_tx_buf.level() > 0;
		radio.update(cur_ticks, radio_rx_buf, radio_tx_buf, last_can_sleep);
		bool has_rx_data = radio_rx_buf.level() > 0;
		bool in_rx = radio.known_state == Si4463::State::Rx;
		bool synced = button_state != DoorbellButton::Unknown;
		radio_leds.update_blink_pattern(has_tx_data, has_rx_data,
		                                last_can_sleep, synced, in_rx);
		radio_leds.update(cur_ticks);

		// Parse the buffer data into a message
		while (radio_rx_buf.level()) {
			uint8_t buf = radio_rx_buf.pop();
			if (buf == '$') {
				pkt_ptr = 0;
				pkt_msg[pkt_ptr++] = buf;
			}
			else if (pkt_ptr < sizeof(pkt_msg)) {
				if (buf == '\r' || buf == '\n') {
					pkt_msg[pkt_ptr] = 0;
					if (pkt_msg[0] == '$') {
						btn_onboard.no_sleep_secs =
							btn_onboard.NO_SLEEP_DUR_SEC;
						if (strcmp(pkt_msg, "$PING")) {
							radio_tx_buf.push("$PONG\n\r");
						}
						else if (strcmp(pkt_msg, "$PONG")) {
							radio_leds.left_led.set_state(
							    LEDStateMachine::BlinkQuickly, 20);
							radio_leds.right_led.set_state(
							    LEDStateMachine::BlinkQuickly, 20);
						}
						UART::print_json_msg("radio", pkt_msg);
					}
					pkt_ptr = 0;
				}
				else if (is_valid_char(buf)) {
					pkt_msg[pkt_ptr++] = buf;
				}
				else {
					pkt_ptr = 0;
				}
			}
		}

		// Handle entering sleep mode
		bool can_sleep = btn_doorbell.can_sleep() &&
		                 btn_onboard_debounce.can_sleep(cur_ticks) &&
		                 btn_onboard.can_sleep() && doorbell_tx.can_sleep() &&
		                 radio.can_sleep() && radio_leds.can_sleep();
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
