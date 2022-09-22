#pragma once

#include <string>


namespace maytag
{
	struct tag_family_t
	{
		std::string name;          // Unique name.
		uint32_t ncodes;           // How many codes are there in this tag family?
		const uint64_t* codes;     // The codes in the family.
		uint32_t width_at_border;
		uint32_t total_width;
		uint32_t nbits;
		const uint8_t* bit_x;      // The bit locations.
		const uint8_t* bit_y;      // The bit locations.
		uint8_t h;                 // Minimum hamming distance between any two codes (e.g. 36h11 => 11).
		bool black = true;         // Tag color (black or white).
		uint8_t hamming = 0;       // How many errors corrected?
	};
}