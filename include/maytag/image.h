#pragma once

#include <cstdint>

#include "math.h"


namespace maytag
{
	struct image_t
	{
		uint32_t w; // Width.
		uint32_t h; // Height.
		uint8_t* d; // Data.

		image_t():
			w(0), h(0), d(nullptr)
		{
		}

		image_t(uint32_t width, uint32_t height, uint8_t* data):
			w(width), h(height), d(data)
		{
		}
	};
}