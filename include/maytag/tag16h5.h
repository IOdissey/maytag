#pragma once

#include <cstdint>

#include "tag_family.h"


namespace maytag
{
	tag_family_t tag16h5(bool black = true, uint8_t hamming = 0)
	{		
		static const uint64_t codes[30] = 
		{
			0x00000000000027c8UL,
			0x00000000000031b6UL,
			0x0000000000003859UL,
			0x000000000000569cUL,
			0x0000000000006c76UL,
			0x0000000000007ddbUL,
			0x000000000000af09UL,
			0x000000000000f5a1UL,
			0x000000000000fb8bUL,
			0x0000000000001cb9UL,
			0x00000000000028caUL,
			0x000000000000e8dcUL,
			0x0000000000001426UL,
			0x0000000000005770UL,
			0x0000000000009253UL,
			0x000000000000b702UL,
			0x000000000000063aUL,
			0x0000000000008f34UL,
			0x000000000000b4c0UL,
			0x00000000000051ecUL,
			0x000000000000e6f0UL,
			0x0000000000005fa4UL,
			0x000000000000dd43UL,
			0x0000000000001aaaUL,
			0x000000000000e62fUL,
			0x0000000000006dbcUL,
			0x000000000000b6ebUL,
			0x000000000000de10UL,
			0x000000000000154dUL,
			0x000000000000b57aUL,
		};
		static const uint8_t bit_x[16] = {1, 2, 3, 2, 4, 4, 4, 3, 4, 3, 2, 3, 1, 1, 1, 2};
		static const uint8_t bit_y[16] = {1, 1, 1, 2, 1, 2, 3, 2, 4, 4, 4, 3, 4, 3, 2, 3};

		tag_family_t tf;
		tf.name = "tag16h5";
		tf.h = 5;
		tf.ncodes = 30;
		tf.codes = codes;
		tf.nbits = 16;
		tf.bit_x = bit_x;
		tf.bit_y = bit_y;
		tf.width_at_border = 6;
		tf.total_width = 8;
		tf.black = black;
		tf.hamming = hamming;
		return tf;
	}
}