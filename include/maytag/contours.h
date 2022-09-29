#pragma once

#include <limits>
#include <cstdint>

#include "cfg.h"
#include "image.h"


namespace maytag::_
{
	// Сontour point.
	struct cpt_t
	{
		uint16_t x;
		uint16_t y;
		int8_t gx;
		int8_t gy;
		uint16_t order;

		cpt_t(uint16_t x, uint16_t y, int8_t gx, int8_t gy):
			x(x), y(y), gx(gx), gy(gy)
		{
		}
	};

	class Contours
	{
	private:
		struct rn_t
		{
			uint32_t r; // Root.
			uint32_t n; // Size.

			rn_t(uint32_t r, uint32_t n):
				r(r), n(n)
			{
			}
		};

		const cfg_t* const _cfg;
		uint32_t _size = 0;
		uint32_t _root = 0;
		uint32_t _root_max = 1000;
		uint32_t _contour_max = 100;
		std::vector<uint32_t> _u;
		std::vector<rn_t> _rn;
		std::vector<std::vector<cpt_t>> _contours;

		inline uint32_t _get_root(uint32_t id) const
		{
			uint32_t root_ref = _u[id];
			if (root_ref == 0)
				return 0;
			uint32_t root = _rn[root_ref].r;
			while (root != root_ref)
			{
				// TODO. Collapse the tree?
				// _rn[root_ref].r = _rn[root].r;
				root_ref = root;
				root = _rn[root].r;
			}
			return root;
		}

		inline void _new(uint32_t id)
		{
			++_root;
			_u[id] = _root;
			_rn.emplace_back(_root, 1);
		}

		inline void _new_connect(uint32_t aid, uint32_t bid)
		{
			uint32_t broot = _get_root(bid);
			_u[aid] = broot;
			if (broot > 0)
				_rn[broot].n += 1;
		}

		inline void _new_connect(uint32_t aid, uint32_t bid, uint32_t cid)
		{
			const uint32_t broot = _get_root(bid);
			const uint32_t croot = _get_root(cid);
			if (broot > croot)
			{
				_rn[broot].r = croot;
				_u[aid] = croot;
				if (croot > 0)
					_rn[croot].n += _rn[broot].n + 1;
			}
			else if (broot < croot)
			{
				_rn[croot].r = broot;
				_u[aid] = broot;
				if (broot > 0)
					_rn[broot].n += _rn[croot].n + 1;
			}
			else
			{
				_u[aid] = broot;
				if (broot > 0)
					_rn[broot].n += 1;
			}
		}

		inline void _zero_connect(uint32_t id)
		{
			uint32_t root_ref = _u[id];
			if (root_ref == 0)
				return;
			uint32_t root = _rn[root_ref].r;
			if (root == 0)
				return;
			// Find the root and collapse the tree.
			_rn[root_ref].r = 0;
			do
			{
				root_ref = root;
				root = _rn[root_ref].r;
				_rn[root_ref].r = 0;
			}
			while (root != root_ref);
		}

	public:
		Contours(const cfg_t* cfg) :
			_cfg(cfg)
		{
		}

		std::vector<std::vector<cpt_t>>& calc(const image_t& thresh_img)
		{
			const uint8_t* const img = thresh_img.d;
			const uint32_t w = thresh_img.w;
			const uint32_t h = thresh_img.h;
			const uint32_t size = w * h;
			if (size > _size)
			{
				_size = size;
				_u.resize(size);
			}
			std::memset(_u.data(), 0, size * sizeof(uint32_t));
			_rn.clear();
			_rn.reserve(_root_max);
			_rn.emplace_back(0, 0);
			_root = 0;
			// Find connected contours.
			const uint32_t end = size - w;
			for (uint32_t p = w; p < end;)
			{
				// |d b|
				// |c a| => 0d0c'0b0a
				uint8_t mask = (img[p - w] << 2) | img[p];
				const uint32_t row_end = p + w - 1;
				++p;
				for (; p < row_end; ++p)
				{
					mask = (mask << 4) | (img[p - w] << 2) | img[p];
					switch (mask)
					{
						// |0 0|
						// |0 1| => 0000'0001 = 1
						case 1:
						// |1 1|
						// |1 0| => 0101'0100 = 84
						case 84:
							_new(p);
							break;
						// |0 0|
						// |1 0| => 0001'0000 = 16
						case 16:
						// |0 0|
						// |1 1| => 0001'0001 = 17
						case 17:
						// |1 1|
						// |0 0| => 0100'0100 = 68
						case 68:
						// |1 1|
						// |0 1| => 0100'0101 = 69
						case 69:
							_new_connect(p, p - 1);
							break;
						// |0 1|
						// |0 0| => 0000'0100 = 4
						case 4:
						// |0 1|
						// |0 1| => 0000'0101 = 5
						case 5:
						// |1 0|
						// |1 0| => 0101'0000 = 80
						case 80:
						// |1 0|
						// |1 1| => 0101'0001 = 81
						case 81:
							_new_connect(p, p - w);
							break;
						// |0 1|
						// |1 0| => 0001'0100 = 20
						case 20:
						// |0 1|
						// |1 1| => 0001'0101 = 21
						case 21:
						// |1 0|
						// |0 0| => 0100'0000 = 64
						case 64:
						// |1 0|
						// |0 1| => 0100'0001 = 65
						case 65:
							_new_connect(p, p - 1, p - w);
							break;
						// |0 1|
						// |1 -| => 0001'0110 = 22
						case 22:
						// |1 0|
						// |0 -| => 0100'0010 = 66
						case 66:
							_zero_connect(p - 1);
							_zero_connect(p - w);
						// |0 0|
						// |1 -| => 0001'0010 = 18
						case 18:
						// |0 -|
						// |1 -| => 0001'1010 = 26
						case 26:
						// |1 1|
						// |0 -| => 0100'0110 = 70
						case 70:
						// |1 -|
						// |0 -| => 0100'1010 = 74
						case 74:
							_zero_connect(p - 1);
							break;
						// |0 1|
						// |0 -| => 0000'0110 = 6
						case 6:
						// |0 1|
						// |- 0| => 0010'0100 = 36
						case 36:
						// |0 1|
						// |- 1| => 0010'0101 = 37
						case 37:
						// |0 1|
						// |- -| => 0010'0110 = 38
						case 38:
						// |1 0|
						// |1 -| => 0101'0010 = 82
						case 82:
						// |1 0|
						// |- 0| => 0110'0000 = 96
						case 96:
						// |1 0|
						// |- 1| => 0110'0001 = 97
						case 97:
						// |1 0|
						// |- -| => 0110'0010 = 98
						case 98:
							_zero_connect(p - w);
							break;
					}
				}
				// Right border.
				// We do not check for low contrast.
				{
					mask = (mask << 4) | (img[p - w] << 2) | img[p];
					switch (mask)
					{
						// |0 0|
						// |1 0| => 0001'0000 = 16
						case 16:
						// |1 1|
						// |0 1| => 0100'0101 = 69
						case 69:
							_new_connect(p, p - 1);
							break;
						// |0 1|
						// |0 1| => 0000'0101 = 5
						case 5:
						// |1 0|
						// |1 0| => 0101'0000 = 80
						case 80:
							_new_connect(p, p - w);
							break;
						// |0 1|
						// |1 1| => 0001'0101 = 21
						case 21:
						// |1 0|
						// |0 0| => 0100'0000 = 64
						case 64:
							_new_connect(p, p - 1, p - w);
							break;
						// |0 0|
						// |1 1| => 0001'0001 = 17
						case 17:
						// |1 1|
						// |0 0| => 0100'0100 = 68
						case 68:
							_zero_connect(p - 1);
							break;
						// |0 1|
						// |0 0| => 0000'0100 = 4
						case 4:
						// |1 0|
						// |1 1| => 0101'0001 = 81
						case 81:
							_zero_connect(p - w);
							break;
						// |0 1|
						// |1 0| => 0001'0100 = 20
						case 20:
						// |1 0|
						// |0 1| => 0100'0001 = 65
						case 65:
							_zero_connect(p - 1);
							_zero_connect(p - w);
							break;
					}
					++p;
				}
			}
			// Bottom border.
			// We do not check for low contrast.
			{
				uint32_t p = end;
				uint8_t mask = (img[p - w] << 2) | img[p];
				const uint32_t row_end = p + w - 1;
				++p;
				for (; p < row_end; ++p)
				{
					mask = (mask << 4) | (img[p - w] << 2) | img[p];
					switch (mask)
					{
						// |0 0|
						// |1 1| => 0001'0001 = 17
						case 17:
						// |1 1|
						// |0 0| => 0100'0100 = 68
						case 68:
							_new_connect(p, p - 1);
							break;
						// |0 1|
						// |0 0| => 0000'0100 = 4
						case 4:
						// |1 0|
						// |1 1| => 0101'0001 = 81
						case 81:
							_new_connect(p, p - w);
							break;
						// |0 1|
						// |1 1| => 0001'0101 = 21
						case 21:
						// |1 0|
						// |0 0| => 0100'0000 = 64
						case 64:
							_new_connect(p, p - 1, p - w);
							break;
						// |0 0|
						// |1 0| => 0001'0000 = 16
						case 16:
						// |1 1|
						// |0 1| => 0100'0101 = 69
						case 69:
							_zero_connect(p - 1);
							break;
						// |0 1|
						// |0 1| => 0000'0101 = 5
						case 5:
						// |1 0|
						// |1 0| => 0101'0000 = 80
						case 80:
							_zero_connect(p - w);
							break;
						// |0 1|
						// |1 0| => 0001'0100 = 20
						case 20:
						// |1 0|
						// |0 1| => 0100'0001 = 65
						case 65:
							_zero_connect(p - 1);
							_zero_connect(p - w);
							break;
					}
				}
			}
			// 
			if (_root > _root_max)
			{
				_root_max = _root + _root / 10;
				if (_root_max > size)
					_root_max = size;
			}
			// Collect contours.
			_contours.clear();
			_contours.reserve(_contour_max);
			const uint32_t min_contour_size = _cfg->min_contour_size;
			for (uint32_t p = w + 1; p < size; ++p)
			{
				const uint32_t root = _get_root(p);
				if (root == 0)
					continue;
				if (_rn[root].n < min_contour_size)
					continue;
				// Save index into size.
				uint32_t idx;
				if (_rn[root].n < size)
				{
					idx = _contours.size();
					_contours.emplace_back(std::vector<cpt_t>());
					_contours.back().reserve(_rn[root].n);
					_rn[root].n = size + idx;
				}
				else
					idx = _rn[root].n - size;
				//
				const uint16_t y = p / w;
				const uint16_t x = p - y * w;
				const uint8_t mask = (img[p - w - 1] << 4) | (img[p - 1] << 4) | (img[p - w] << 2) | img[p];
				switch (mask)
				{
					// |0 1|
					// |0 1| => 0000'0101 = 5
					case 5:
						_contours[idx].emplace_back(x, y, -1, 0);
						break;
					// |1 0|
					// |1 0| => 0101'0000 = 80
					case 80:
						_contours[idx].emplace_back(x, y, 1, 0);
						break;
					// |0 0|
					// |1 1| => 0001'0001 = 17
					case 17:
						_contours[idx].emplace_back(x, y, 0, -1);
						break;
					// |1 1|
					// |0 0| => 0100'0100 = 68
					case 68:
						_contours[idx].emplace_back(x, y, 0, 1);
						break;
					// |0 0|
					// |0 1| => 0000'0001 = 1
					case 1:
					// |0 1|
					// |1 1| => 0001'0101 = 21
					case 21:
						_contours[idx].emplace_back(x, y, -1, -1);
						break;
					// |1 0|
					// |0 0| => 0100'0000 = 64
					case 64:
					// |1 1|
					// |1 0| => 0101'0100 = 84
					case 84:
						_contours[idx].emplace_back(x, y, 1, 1);
						break;
					// |0 1|
					// |0 0| => 0000'0100 = 4
					case 4:
					// |1 1|
					// |0 1| => 0100'0101 = 69
					case 69:
						_contours[idx].emplace_back(x, y, -1, 1);
						break;
					// |0 0|
					// |1 0| => 0001'0000 = 16
					case 16:
					// |1 0|
					// |1 1| => 0101'0001 = 81
					case 81:
						_contours[idx].emplace_back(x, y, 1, -1);
						break;
					default:
						_contours[idx].emplace_back(x, y, 0, 0);
						break;
				}
			}
			// 
			if (_contours.size() > _contour_max)
				_contour_max = _contours.size() + _contours.size() / 10;

			return _contours;
		}
	};
}