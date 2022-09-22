#pragma once

#include <cstring>

#include "cfg.h"
#include "image.h"


namespace maytag::_
{
	class Threshold
	{
	private:
		const cfg_t* const _cfg;
		uint32_t _size = 0;
		uint8_t* _ptr = nullptr;
		uint8_t* _img_min;
		uint8_t* _img_max;
		uint8_t* _img_min_tmp;
		uint8_t* _img_max_tmp;

	public:
		Threshold(const cfg_t* cfg) :
			_cfg(cfg)
		{
		}

		~Threshold()
		{
			if (_ptr)
				delete[] _ptr;
		}

		image_t calc(const image_t& gray_img)
		{
			const uint32_t tile_size = _cfg->tile_size;
			const uint8_t* const img = gray_img.d;
			const uint32_t w = gray_img.w;
			const uint32_t h = gray_img.h;
			const uint32_t s = w * h;
			const uint32_t tw = w / tile_size;
			const uint32_t th = h / tile_size;
			const uint32_t tx_end = tw - 1;
			const uint32_t ty_end = th - 1;
			if (s > _size)
			{
				_size = s;
				if (_ptr)
					delete[] _ptr;
				const uint32_t ts = tw * th;
				_ptr = new uint8_t[s + 2 * ts];
				_img_min = _ptr + s;
				_img_max = _img_min + ts;
				_img_max_tmp = _img_min - ts;
				_img_min_tmp = _img_max_tmp - ts;
			}
			// Collect min/max statistics for each tile.
			for (uint32_t ty = 0, t = 0; ty < th; ++ty)
			{
				for (uint32_t tx = 0; tx < tw; ++tx, ++t)
				{
					const uint32_t p = (ty * w + tx) * tile_size;
					uint8_t min = 255;
					uint8_t max = 0;
					for (uint32_t dy = 0; dy < tile_size; ++dy)
					{
						for (uint32_t dx = 0, i = p + dy * w; dx < tile_size; ++dx, ++i)
						{
							const uint8_t v = img[i];
							if (v < min)
								min = v;
							if (v > max)
								max = v;
						}
					}
					_img_min[t] = min;
					_img_max[t] = max;
				}
			}
			// Apply 3x3 min/max convolution to "blur" these values over larger areas.
			// This reduces artifacts due to abrupt changes in the threshold value.
			// Apply 1x3 min/max convolution.
			for (uint32_t ty = 0, t = 0; ty < th; ++ty)
			{
				for (uint32_t tx = 0; tx < tw; ++tx, ++t)
				{
					uint8_t min = _img_min[t];
					uint8_t max = _img_max[t];
					if (tx > 0)
					{
						const uint8_t v_min = _img_min[t - 1];
						if (v_min < min)
							min = v_min;
						const uint8_t v_max = _img_max[t - 1];
						if (v_max > max)
							max = v_max;
					}
					if (tx < tx_end)
					{
						const uint8_t v_min = _img_min[t + 1];
						if (v_min < min)
							min = v_min;
						const uint8_t v_max = _img_max[t + 1];
						if (v_max > max)
							max = v_max;
					}
					_img_min_tmp[t] = min;
					_img_max_tmp[t] = max;
				}
			}
			// Apply 3x1 min/max convolution.
			for (uint32_t ty = 0, t = 0; ty < th; ++ty)
			{
				for (uint32_t tx = 0; tx < tw; ++tx, ++t)
				{
					uint8_t min = _img_min_tmp[t];
					uint8_t max = _img_max_tmp[t];
					if (ty > 0)
					{
						const uint8_t v_min = _img_min_tmp[t - tw];
						if (v_min < min)
							min = v_min;
						const uint8_t v_max = _img_max_tmp[t - tw];
						if (v_max > max)
							max = v_max;
					}
					if (ty < ty_end)
					{
						const uint8_t v_min = _img_min_tmp[t + tw];
						if (v_min < min)
							min = v_min;
						const uint8_t v_max = _img_max_tmp[t + tw];
						if (v_max > max)
							max = v_max;
					}
					_img_min[t] = min;
					_img_max[t] = max;
				}
			}
			// Calculate binary image.
			const uint8_t min_wb_diff = _cfg->min_wb_diff;
			std::memset(_ptr, 2, s);
			for (uint32_t ty = 0, t = 0; ty < th; ++ty)
			{
				const uint32_t dy_end = (ty == ty_end) ? (tile_size + h - th * tile_size) : tile_size;
				for (uint32_t tx = 0; tx < tw; ++tx, ++t)
				{
					const uint8_t d = _img_max[t] - _img_min[t];
					if (d <= min_wb_diff)
						continue;
					const uint32_t dx_end = (tx == tx_end) ? (tile_size + w - tw * tile_size) : tile_size;
					const uint8_t thresh = _img_min[t] + d / 2;
					const uint32_t p = (ty * w + tx) * tile_size;
					for (uint32_t dy = 0; dy < dy_end; ++dy)
					{
						for (uint32_t dx = 0, i = p + dy * w; dx < dx_end; ++dx, ++i)
						{
							if (img[i] > thresh)
								_ptr[i] = 1;
							else
								_ptr[i] = 0;
						}
					}
				}
			}
			return image_t(w, h, _ptr);
		}
	};
}