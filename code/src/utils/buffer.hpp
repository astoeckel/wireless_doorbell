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

#pragma once

#include <avr/pgmspace.h>
#include <stddef.h>
#include <stdint.h>

class Buffer {
private:
	uint8_t *m_buf;
	size_t m_size;

public:
	template <typename T>
	explicit Buffer(T &buf)
	    : m_buf(reinterpret_cast<uint8_t *>(&buf)), m_size(sizeof(buf))
	{
	}

	template <typename T>
	Buffer(T *buf, size_t size)
	    : m_buf(reinterpret_cast<uint8_t *>(buf)), m_size(size)
	{
	}

	size_t size() const { return m_size; }
	uint8_t &operator[](size_t i) { return m_buf[i]; }
	uint8_t operator[](size_t i) const { return m_buf[i]; }
};

class ProgramSpaceBuffer {
private:
	const uint8_t *m_buf;
	size_t m_size;

public:
	template <typename T>
	explicit ProgramSpaceBuffer(T &buf)
	    : m_buf(reinterpret_cast<const uint8_t *>(&buf)), m_size(sizeof(buf))
	{
	}

	template <typename T>
	ProgramSpaceBuffer(const T *buf, size_t size)
	    : m_buf(reinterpret_cast<const uint8_t *>(buf)), m_size(size)
	{
	}

	size_t size() const { return m_size; }
	uint8_t operator[](size_t i) const { return pgm_read_byte(&(m_buf[i])); }
};

class Ignore {
private:
	size_t m_size;

public:
	Ignore(size_t size) : m_size(size) {}

	size_t size() const { return m_size; }
};

template <size_t Size>
struct Ringbuffer {
	static_assert(Size <= 256, "Size must be smaller than 256");
	static_assert((Size & (Size - 1)) == 0, "Size must be a power of two");

	uint8_t data[Size];
	uint8_t read_ptr;
	uint8_t write_ptr;

	Ringbuffer() : read_ptr(0), write_ptr(0) {}

	uint8_t level() const { return (write_ptr - read_ptr) & (Size - 1); }

	void push(uint8_t x)
	{
		data[write_ptr] = x;
		write_ptr = (write_ptr + 1) & (Size - 1);
	}

	void push(const char *s) {
		while (*s) {
			push(*(s++));
		}
	}

	uint8_t pop()
	{
		uint8_t res = data[read_ptr];
		read_ptr = (read_ptr + 1) & (Size - 1);
		return res;
	}

	static constexpr size_t size() { return Size; }
};