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

#include "uart.hpp"

#include <avr/interrupt.h>

namespace UART {

ISR(USART_RX_vect)
{
	// rx_buf.push(UDR0);
}

void init()
{
	UBRR0H = 0x00;
	UBRR0L = 0x07; // 115.2 kbaud
	//UBRR0L = 0x0F;  // 57.6 kbaud
	//	UBRR0L = 0x2F; // 19.2 kbaud
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
}

void puts(const char *s)
{
	while (*s) {
		putc(*(s++));
	}
}

void put_hex_digit(uint8_t n) { putc(hex_digit(n)); }

void put_hex(uint32_t x)
{
	put_hex_digit((x & 0xF0'00'00'00) >> 28);
	put_hex_digit((x & 0x0F'00'00'00) >> 24);
	put_hex_digit((x & 0x00'F0'00'00) >> 20);
	put_hex_digit((x & 0x00'0F'00'00) >> 16);
	put_hex_digit((x & 0x00'00'F0'00) >> 12);
	put_hex_digit((x & 0x00'00'0F'00) >> 8);
	put_hex_digit((x & 0x00'00'00'F0) >> 4);
	put_hex_digit((x & 0x00'00'00'0F) >> 0);
}

void put_hex(uint16_t x)
{
	put_hex_digit((x & 0xF0'00) >> 12);
	put_hex_digit((x & 0x0F'00) >> 8);
	put_hex_digit((x & 0x00'F0) >> 4);
	put_hex_digit((x & 0x00'0F) >> 0);
}

void put_hex(uint8_t x)
{
	put_hex_digit((x & 0xF0) >> 4);
	put_hex_digit((x & 0x0F) >> 0);
}

template <>
void print(const char *s)
{
	puts(s);
}

template <>
void print(uint8_t x)
{
	put_hex(x);
}

template <>
void print(uint16_t x)
{
	put_hex(x);
}

template <>
void print(uint32_t x)
{
	put_hex(x);
}

template <>
void print(Buffer buf)
{
	for (size_t i = 0; i < buf.size(); i++) {
		putc(buf[i]);
	}
}

template <>
void print(ProgramSpaceBuffer buf)
{
	for (size_t i = 0; i < buf.size(); i++) {
		putc(buf[i]);
	}
}

void print_json_str(const char *s)
{
	print("\"");
	while (*s) {
		char c = *(s++);
		if (c == '"') {
			putc('\\');
			putc('"');
		}
		else if (c == '\\') {
			putc('\\');
			putc('\\');
		}
		else if (c == '\n') {
			putc('\\');
			putc('n');
		}
		else if (c == '\r') {
			putc('\\');
			putc('r');
		}
		else if (c == '\b') {
			putc('\\');
			putc('b');
		}
		else if (c == '\t') {
			putc('\\');
			putc('t');
		}
		else if (c >= 0x20 && c < 0x7F) {
			putc(c);
		}
		else {
			print("\\u");
			print(uint16_t(c));
		}
	}
	print("\"");
}

void print_json_msg(const char *type, const char *data)
{
	print("{\"type\": ");
	print_json_str(type);
	print(", \"data\": ");
	print_json_str(data);
	print("}\n\r");
}

}  // namespace UART