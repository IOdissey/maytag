#pragma once

#include <string>

#include "pt.h"


namespace maytag
{
	struct tag_t
	{
		pt_t p[4];        // Corners.
		std::string name; // Tag family.
		uint16_t id;      // 
		bool black;       // Tag color (black or white).
		uint8_t hamming;  // Hamming distance.
		double score;     //
	};
}