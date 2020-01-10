#include "crc16.hpp"

static constexpr uint16_t CRC16_INITIAL = 0xFFFFu;
static constexpr uint16_t CRC16_OUTPUT_XOR = 0x0000u;

uint16_t crc16_add(uint16_t crc, uint8_t value)
{
	uint32_t i;
	const uint16_t poly = 0x1021u;
	crc ^= (uint16_t)((uint16_t) value << 8u);

	for (i = 0; i < 8; i++) {
		if (crc & (1u << 15u)) {
			crc = (uint16_t)((crc << 1u) ^ poly);

		} else {
			crc = (uint16_t)(crc << 1u);
		}
	}

	return crc;
}

uint16_t crc16_signature(uint16_t initial, size_t length, const uint8_t *bytes)
{
	size_t i;

	for (i = 0u; i < length; i++) {
		initial = crc16_add(initial, bytes[i]);
	}

	return initial ^ CRC16_OUTPUT_XOR;
}
