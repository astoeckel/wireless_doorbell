#pragma once

#include <stddef.h>
#include <stdint.h>

#include <utils/buffer.hpp>

namespace UART {
void init();

static inline void putc(uint8_t c)
{
	while (!(UCSR0A & (1 << UDRE0))) {}
	UDR0 = c;
}

void puts(const char *s);

static inline uint8_t hex_digit(uint8_t n)
{
	return (n > 9) ? ('A' + (n - 10)) : ('0' + n);
}

void put_hex_digit(uint8_t n);
void put_hex(uint32_t x);
void put_hex(uint16_t x);
void put_hex(uint8_t x);

template <typename T>
void print(T x);

template <typename T, typename... Ts>
void print(T x, Ts... xs)
{
	print(x);
	print(xs...);
}

void print_json_str(const char *s);
void print_json_msg(const char *type, const char *data);

}  // namespace UART