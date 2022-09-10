#pragma once

#include <stdint.h>

inline constexpr uint8_t operator"" _B(unsigned long long value)
{
	return static_cast<uint8_t>(value);
}
