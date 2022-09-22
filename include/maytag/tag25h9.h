#pragma once

#include <cstdint>

#include "tag_family.h"


namespace maytag
{
	tag_family_t tag25h9(bool black = true, uint8_t hamming = 0)
	{
		static const uint64_t codes[35] = 
		{
			0x000000000156f1f4UL,
			0x0000000001f28cd5UL,
			0x00000000016ce32cUL,
			0x0000000001ea379cUL,
			0x0000000001390f89UL,
			0x000000000034fad0UL,
			0x00000000007dcdb5UL,
			0x000000000119ba95UL,
			0x0000000001ae9daaUL,
			0x0000000000df02aaUL,
			0x000000000082fc15UL,
			0x0000000000465123UL,
			0x0000000000ceee98UL,
			0x0000000001f17260UL,
			0x00000000014429cdUL,
			0x00000000017248a8UL,
			0x00000000016ad452UL,
			0x00000000009670adUL,
			0x00000000016f65b2UL,
			0x0000000000b8322bUL,
			0x00000000005d715bUL,
			0x0000000001a1c7e7UL,
			0x0000000000d7890dUL,
			0x0000000001813522UL,
			0x0000000001c9c611UL,
			0x000000000099e4a4UL,
			0x0000000000855234UL,
			0x00000000017b81c0UL,
			0x0000000000c294bbUL,
			0x000000000089fae3UL,
			0x000000000044df5fUL,
			0x0000000001360159UL,
			0x0000000000ec31e8UL,
			0x0000000001bcc0f6UL,
			0x0000000000a64f8dUL,
		};
		static const uint8_t bit_x[25] = {1, 2, 3, 4, 2, 3, 5, 5, 5, 5, 4, 4, 5, 4, 3, 2, 4, 3, 1, 1, 1, 1, 2, 2, 3};
		static const uint8_t bit_y[25] = {1, 1, 1, 1, 2, 2, 1, 2, 3, 4, 2, 3, 5, 5, 5, 5, 4, 4, 5, 4, 3, 2, 4, 3, 3};

		tag_family_t tf;
		tf.name = "tag25h9";
		tf.h = 9;
		tf.ncodes = 35;
		tf.codes = codes;
		tf.nbits = 25;
		tf.bit_x = bit_x;
		tf.bit_y = bit_y;
		tf.width_at_border = 7;
		tf.total_width = 9;
		tf.black = black;
		tf.hamming = hamming;
		return tf;
	}
}