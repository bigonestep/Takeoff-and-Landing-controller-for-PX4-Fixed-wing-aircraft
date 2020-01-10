


uint64_t crc64_add_word(uint64_t crc, uint32_t value)
{
	uint32_t i, j;
	uint8_t byte;
	const uint64_t poly = 0x42F0E1EBA9EA3693ull;

	for (j = 0; j < 4; j++) {
		byte = ((uint8_t *) &value)[j];
		crc ^= (uint64_t) byte << 56u;

		for (i = 0; i < 8; i++) {
			if (crc & (1ull << 63u)) {
				crc = (uint64_t)(crc << 1u) ^ poly;

			} else {
				crc = (uint64_t)(crc << 1u);
			}
		}
	}

	return crc;
}
