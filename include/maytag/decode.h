#pragma once

#include <cstdint>
#include <cstring>

#include "cfg.h"
#include "quad.h"
#include "image.h"
#include "graymodel.h"
#include "tag_family.h"
#include "tag.h"


namespace maytag::_
{
	class Decode
	{
	private:
		const cfg_t* const _cfg;
		double _h[9];
		uint32_t _size = 0;
		double* _val = nullptr;
		double* _tmp = nullptr;
		std::vector<tag_t> _tags;

		void _init_ptr(uint32_t size)
		{
			if (size <= _size)
				return;
			_size = size;
			if (_val)
				delete[] _val;
			_val = new double[2 * _size * _size];
			_tmp = _val + _size * _size;
		}

		void _sharpen(const uint32_t size)
		{
			// Kernel:
			// | 0 -1  0|
			// |-1  4 -1|
			// | 0 -1  0|
			for (uint32_t y = 0, p = 0; y < size; ++y)
			{
				for (uint32_t x = 0; x < size; ++x, ++p)
				{
					double v = 4.0 * _val[p];
					if (y > 0)
						v -= _val[p - size];
					if (x > 0)
						v -= _val[p - 1];
					if (x < size - 1)
						v -= _val[p + 1];
					if (y < size - 1)
						v -= _val[p + size];
					_tmp[p] = v;
				}
			}
			const uint32_t size_2 = size * size;
			const double decode_sharpening = _cfg->decode_sharpening;
			for (uint32_t p = 0; p < size_2; ++p)
				_val[p] += decode_sharpening * _tmp[p];
		}

		bool _calc_homography(const quad_t& quad)
		{
			const double c[4][4] = {
				{0.0, 0.0, quad.p[0].x, quad.p[0].y},
				{0.0, 1.0, quad.p[1].x, quad.p[1].y},
				{1.0, 1.0, quad.p[2].x, quad.p[2].y},
				{1.0, 0.0, quad.p[3].x, quad.p[3].y}
			};
			double a[] = {
				c[0][0], c[0][1], 1.0,     0.0,     0.0, 0.0, -c[0][0] * c[0][2], -c[0][1] * c[0][2], c[0][2],
				    0.0,     0.0, 0.0, c[0][0], c[0][1], 1.0, -c[0][0] * c[0][3], -c[0][1] * c[0][3], c[0][3],
				c[1][0], c[1][1], 1.0,     0.0,     0.0, 0.0, -c[1][0] * c[1][2], -c[1][1] * c[1][2], c[1][2],
				    0.0,     0.0, 0.0, c[1][0], c[1][1], 1.0, -c[1][0] * c[1][3], -c[1][1] * c[1][3], c[1][3],
				c[2][0], c[2][1], 1.0,     0.0,     0.0, 0.0, -c[2][0] * c[2][2], -c[2][1] * c[2][2], c[2][2],
				    0.0,     0.0, 0.0, c[2][0], c[2][1], 1.0, -c[2][0] * c[2][3], -c[2][1] * c[2][3], c[2][3],
				c[3][0], c[3][1], 1.0,     0.0,     0.0, 0.0, -c[3][0] * c[3][2], -c[3][1] * c[3][2], c[3][2],
				    0.0,     0.0, 0.0, c[3][0], c[3][1], 1.0, -c[3][0] * c[3][3], -c[3][1] * c[3][3], c[3][3],
			};
			// Eliminate.
			for (int col = 0; col < 8; col++)
			{
				// Find best row to swap with.
				double max_val = 0;
				int max_val_idx = -1;
				for (int row = col; row < 8; row++)
				{
					double val = std::abs(a[row * 9 + col]);
					if (val > max_val)
					{
						max_val = val;
						max_val_idx = row;
					}
				}
				if (max_val < 1e-9)
					return false;
				// Swap to get best row.
				if (max_val_idx != col)
				{
					for (int i = col; i < 9; i++)
					{
						double tmp = a[col * 9 + i];
						a[col * 9 + i] = a[max_val_idx * 9 + i];
						a[max_val_idx * 9 + i] = tmp;
					}
				}
				// Do eliminate.
				for (int i = col + 1; i < 8; i++)
				{
					double f = a[i * 9 + col] / a[col * 9 + col];
					a[i * 9 + col] = 0;
					for (int j = col + 1; j < 9; j++)
						a[i * 9 + j] -= f * a[col * 9 + j];
				}
			}
			// Back solve.
			for (int col = 7; col >= 0; col--)
			{
				double sum = 0;
				for (int i = col + 1; i < 8; i++)
					sum += a[col * 9 + i] * a[i * 9 + 8];
				a[col * 9 + 8] = (a[col * 9 + 8] - sum) / a[col * 9 + col];
			}
			_h[0] = a[8];
			_h[1] = a[17];
			_h[2] = a[26];
			_h[3] = a[35];
			_h[4] = a[44];
			_h[5] = a[53];
			_h[6] = a[62];
			_h[7] = a[71];
			_h[8] = 1.0;
			return true;
		}

		// h - 3x3 dimansion.
		// x, y - coordinates in tag space [0, 1].
		// px, py - coordinates (pixels) in image space.
		void _homography_project(const double* h, double x, double y, double& px, double& py) const
		{
			double xx = h[0] * x + h[1] * y + h[2];
			double yy = h[3] * x + h[4] * y + h[5];
			double zz = h[6] * x + h[7] * y + h[8];
			px = xx / zz;
			py = yy / zz;
		}

		void _add_model(const image_t& gray_img, graymodel_t& model, double tx, double ty) const
		{
			double px, py;
			_homography_project(_h, tx, ty, px, py);
			// don't round
			int ix = static_cast<int>(px);
			int iy = static_cast<int>(py);
			if (ix >= 0 && iy >= 0 && ix < gray_img.w && iy < gray_img.h)
				model.add(tx, ty, gray_img.d[iy * gray_img.w + ix]);
		}

		// Decode the tag binary contents by sampling the pixel closest to the center of each bit cell.
		// We will compute a threshold by sampling known white/black cells around this tag.
		// This sampling is achieved by considering a set of samples along lines.
		uint64_t _quad_code(const quad_t& quad, const tag_family_t& family, const image_t& gray_img, double& score)
		{
			// Tag coordinates in range [0, 1].
			// Tag bit size.
			const double d_full = 1.0 / family.width_at_border;
			// Tag bit falf size.
			const double d_half = 0.5 * d_full;
			//
			graymodel_t white_model;
			graymodel_t black_model;
			// Left and right white columns.
			{
				const double dx_l = -d_half;
				const double dx_r = 1.0 + d_half;
				double dy = d_half;
				for (uint32_t i = 0; i < family.width_at_border; ++i, dy += d_full)
				{
					_add_model(gray_img, white_model, dx_l, dy);
					_add_model(gray_img, white_model, dx_r, dy);
				}
			}
			// Top and bottom white rows.
			{
				const double dy_t = -d_half;
				const double dy_b = 1.0 + d_half;
				double dx = d_half;
				for (uint32_t i = 0; i < family.width_at_border; ++i, dx += d_full)
				{
					_add_model(gray_img, white_model, dx, dy_t);
					_add_model(gray_img, white_model, dx, dy_b);
				}
			}
			// Left and right black columns.
			{
				const double dx_l = d_half;
				const double dx_r = 1.0 - d_half;
				double dy = d_half;
				for (uint32_t i = 0; i < family.width_at_border; ++i, dy += d_full)
				{
					_add_model(gray_img, black_model, dx_l, dy);
					_add_model(gray_img, black_model, dx_r, dy);
				}
			}
			// Top and bottom black rows.
			{
				const double dy_t = d_half;
				const double dy_b = 1.0 - d_half;
				double dx = d_half + d_full;
				// Two bits less since it were counted in columns.
				for (uint32_t i = 2; i < family.width_at_border; ++i, dx += d_full)
				{
					_add_model(gray_img, black_model, dx, dy_t);
					_add_model(gray_img, black_model, dx, dy_b);
				}
			}
			//
			white_model.solve();
			black_model.solve();
			//
			if (white_model.interpolate(0.0, 0.0) < black_model.interpolate(0.0, 0.0))
			{
				if (family.black)
					return std::numeric_limits<uint64_t>::max();
			}
			else if (!family.black)
				return std::numeric_limits<uint64_t>::max();
			//
			std::memset(_val, 0, family.total_width * family.total_width * sizeof(double));
			const uint32_t beg_coord = (family.total_width + 1) * (family.total_width - family.width_at_border) / 2;
			const uint8_t* const img = gray_img.d;
			const uint32_t w = gray_img.w;
			const uint32_t h = gray_img.h;
			for (uint32_t i = 0; i < family.nbits; ++i)
			{
				uint32_t bit_x = family.bit_x[i];
				uint32_t bit_y = family.bit_y[i];
				double tx = d_half + bit_x * d_full;
				double ty = d_half + bit_y * d_full;
				double px, py;
				_homography_project(_h, tx, ty, px, py);
				// Interpolate.
				int xi = static_cast<int>(px);
				int yi = static_cast<int>(py);
				if (xi < 0 || yi < 0 || xi >= w - 1 || yi >= h - 1)
					continue;
				double v = 0.5 * (black_model.interpolate(tx, ty) + white_model.interpolate(tx, ty));
				const uint32_t p = yi * w + xi;
				if (_cfg->interpolate)
				{
					px -= xi;
					py -= yi;
					uint8_t i00 = img[p];
					uint8_t i10 = img[p + 1];
					uint8_t i01 = img[p + w];
					uint8_t i11 = img[p + w + 1];
					double w11 = px * py;
					double w10 = px - w11;
					double w01 = py - w11;
					double w00 = 1.0 + w11 - px - py;
					v -= i00 * w00 + i10 * w10 + i01 * w01 + i11 * w11;
				}
				else
					v -= img[p];
				const uint32_t idx = beg_coord + family.total_width * bit_y + bit_x;
				if (family.black)
					_val[idx] = -v;
				else
					_val[idx] = v;
			}
			// Sharpen.
			_sharpen(family.total_width);
			//
			double black_score = 0.0;
			double white_score = 0.0;
			uint32_t black_score_count = 1;
			uint32_t white_score_count = 1;
			uint64_t code = 0;
			for (uint32_t i = 0; i < family.nbits; ++i)
			{
				code <<= 1;
				uint32_t bit_x = family.bit_x[i];
				uint32_t bit_y = family.bit_y[i];
				double v = _val[beg_coord + family.total_width * bit_y + bit_x];
				if (v > 0)
				{
					white_score += v;
					white_score_count++;
					code |= 1;
				}
				else
				{
					black_score -= v;
					black_score_count++;
				}
			}
			if (white_score_count == 0 || black_score_count == 0)
				return std::numeric_limits<uint64_t>::max();
			score = std::min(white_score / white_score_count, black_score / black_score_count);
			if (score < _cfg->min_score)
				return std::numeric_limits<uint64_t>::max();
			return code;
		}

		//
		inline void _tag_rotate(uint8_t rot, const pt_t* const p_src, pt_t* const p_dest) const
		{
			for (uint8_t i = 0; i < 4; ++i)
				p_dest[(i + rot) & 3] = p_src[i];
		}

	public:
		Decode(const cfg_t* cfg) :
			_cfg(cfg)
		{
		}

		~Decode()
		{
			if (_val)
				delete[] _val;
		}

		const std::vector<tag_t>& calc(const std::vector<quad_t>& quads, const image_t& gray_img)
		{
			_tags.clear();
			const uint32_t size = quads.size();
			if (size == 0)
				return _tags;
			const auto& tag_family = _cfg->tag_family;
			const uint32_t tag_family_size = tag_family.size();
			if (tag_family_size == 0)
				return _tags;
			_init_ptr(_cfg->max_total_width);
			_tags.reserve(size);
			tag_t tag;
			for (uint32_t i = 0; i < size; ++i)
			{
				const auto& quad = quads[i];
				if (!_calc_homography(quad))
					continue;
				for (int fi = 0; fi < tag_family_size; ++fi)
				{
					const auto& family = tag_family[fi];
					if (family.black != quad.black)
						continue;
					uint64_t code = _quad_code(quad, family, gray_img, tag.score);
					if (code == std::numeric_limits<uint64_t>::max())
						continue;
					uint8_t rot;
					if (!_cfg->tag_dict[fi]->decode(code, tag.id, tag.hamming, rot))
						continue;
					if (tag.hamming > family.hamming)
						continue;
					_tag_rotate(rot, quad.p, tag.p);
					tag.black = quad.black;
					tag.name = family.name;
					_tags.emplace_back(tag);
				}
			}
			return _tags;
		}
	};
}