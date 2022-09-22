#pragma once

#include <cstdint>

#include "cfg.h"
#include "image.h"


namespace maytag::_
{
	class Decimate
	{
	private:
		const cfg_t* const _cfg;
		uint32_t _size = 0;
		uint8_t* _ptr = nullptr;

		void _init_ptr(uint32_t size)
		{
			if (size <= _size)
				return;
			_size = size;
			if (_ptr)
				delete[] _ptr;
			_ptr = new uint8_t[_size];
		}

	public:
		Decimate(const cfg_t* cfg) :
			_cfg(cfg)
		{
		}

		~Decimate()
		{
			if (_ptr)
				delete[] _ptr;
		}

		image_t calc(const image_t& gray_img)
		{
			const uint32_t quad_decimate_type = _cfg->quad_decimate_type;
			if (quad_decimate_type == 1)
				return gray_img;
			const uint8_t* const img = gray_img.d;
			const uint32_t w = gray_img.w;
			const uint32_t h = gray_img.h;
			const uint32_t s = w * h;
			// 1.5
			if (quad_decimate_type == 0)
			{
				const uint32_t dw = (w / 3) * 2;
				const uint32_t dh = (h / 3) * 2;
				if (dw < 3 || dh < 3)
					return gray_img;
				_init_ptr(dw * dh);
				for (uint32_t dy = 0, y = 0; dy < dh; dy += 2, y += 3)
				{
					for (uint32_t dx = 0, p4 = y * w + w; dx < dw; dx += 2, p4 += 3)
					{
						// 1 2 3
						// 4 5 6
						// 7 8 9
						const uint8_t v1 = img[p4 - w];
						const uint8_t v2 = img[p4 - w + 1];
						const uint8_t v3 = img[p4 - w + 2];
						const uint8_t v4 = img[p4];
						const uint8_t v5 = img[p4 + 1];
						const uint8_t v6 = img[p4 + 2];
						const uint8_t v7 = img[p4 + w];
						const uint8_t v8 = img[p4 + w + 1];
						const uint8_t v9 = img[p4 + w + 2];
						//
						const uint32_t p = dy * dw + dx;
						_ptr[p]          = (4 * v1 + 2 * v2 + 2 * v4 + v5) / 9;
						_ptr[p + 1]      = (4 * v3 + 2 * v2 + 2 * v6 + v5) / 9;
						_ptr[p + dw]     = (4 * v7 + 2 * v8 + 2 * v4 + v5) / 9;
						_ptr[p + dw + 1] = (4 * v9 + 2 * v8 + 2 * v6 + v5) / 9;
					}
				}
				return image_t(dw, dh, _ptr);
			}
			// 2, 3, ...
			else
			{
				const uint32_t dw = 1 + (w - 1) / quad_decimate_type;
				const uint32_t dh = 1 + (h - 1) / quad_decimate_type;
				if (dw < 3 || dh < 3)
					return gray_img;
				_init_ptr(dw * dh);
				for (uint32_t dy = 0, y = 0; dy < dh; ++dy, y += quad_decimate_type)
				{
					for (uint32_t dx = 0, p = y * w; dx < dw; ++dx, p += quad_decimate_type)
						_ptr[dy * dw + dx] = img[p];
				}
				return image_t(dw, dh, _ptr);
			}
		}
	};
}