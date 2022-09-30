#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

#include "cfg.h"
#include "contours.h"
#include "pt.h"


namespace maytag::_
{
	struct quad_t
	{
		pt_t p[4];  // Corners.
		bool black; //
	};

	class Quad
	{
	private:
		struct fit_data_t
		{
			double w = 0.0;
			double wx = 0.0;
			double wy = 0.0;
			double wxx = 0.0;
			double wyy = 0.0;
			double wxy = 0.0;

			inline void operator+=(const fit_data_t& v)
			{
				w += v.w;
				wx += v.wx;
				wy += v.wy;
				wxx += v.wxx;
				wyy += v.wyy;
				wxy += v.wxy;
			}

			inline void operator-=(const fit_data_t& v)
			{
				w -= v.w;
				wx -= v.wx;
				wy -= v.wy;
				wxx -= v.wxx;
				wyy -= v.wyy;
				wxy -= v.wxy;
			}
		};

		struct line_param_t
		{
			double px;  // Point on the line.
			double py;  // Point on the line.
			double dx;  // Line direction.
			double dy;  // Line direction.
			double cxx;
			double cyy;
			double cxy;
			double sq;

			double calc_point(const fit_data_t& fd)
			{
				const double inv_w = 1.0 / fd.w;
				px = fd.wx * inv_w;
				py = fd.wy * inv_w;
				cxx = fd.wxx * inv_w - px * px;
				cyy = fd.wyy * inv_w - py * py;
				cxy = fd.wxy * inv_w - px * py;
				sq = std::sqrt((cxx - cyy) * (cxx - cyy) + 4.0 * cxy * cxy);
				return 0.5 * (cxx + cyy - sq);
			}

			void calc_direction()
			{
				double eig = 0.5 * (cxx + cyy + sq);
				double nx1 = cxx - eig;
				double ny1 = cxy;
				double m1 = nx1 * nx1 + ny1 * ny1;
				double nx2 = cxy;
				double ny2 = cyy - eig;
				double m2 = nx2 * nx2 + ny2 * ny2;
				if (m1 > m2)
				{
					const double norm = 1.0 / std::sqrt(m1);
					// Rotate the normal (nx, ny) by 90 degrees.
					dx = -ny1 * norm;
					dy = nx1 * norm;
				}
				else
				{
					const double norm = 1.0 / std::sqrt(m2);
					// Rotate the normal (nx, ny) by 90 degrees.
					dx = -ny2 * norm;
					dy = nx2 * norm;
				}
			}
		};

		const cfg_t* const _cfg;
		std::vector<double> _filter;
		std::vector<fit_data_t> _fit_data;
		std::vector<quad_t> _quads;

		// Sort contour points around the center.
		// Border color check.
		bool _sort_contour(std::vector<cpt_t>& contour, quad_t& quad)
		{
			const double quad_decimate = _cfg->quad_decimate;
			const uint32_t size = contour.size();
			// Calculate the bounding box.
			uint16_t x_max = contour[0].x;
			uint16_t x_min = x_max;
			uint16_t y_max = contour[0].y;
			uint16_t y_min = y_max;
			for (uint32_t i = 1; i < size; ++i)
			{
				const auto& p = contour[i];
				if (p.x > x_max)
					x_max = p.x;
				else if (p.x < x_min)
					x_min = p.x;
				if (p.y > y_max)
					y_max = p.y;
				else if (p.y < y_min)
					y_min = p.y;
			}
			// Quick check tag size.
			{
				double dx = (x_max - x_min) * quad_decimate;
				if (dx < _cfg->min_tag_size)
					return false;
				double dy = (y_max - y_min) * quad_decimate;
				if (dy < _cfg->min_tag_size)
					return false;
			}
			// Sort contour points around the center.
			const double cx = (x_min + x_max) * 0.5 + 0.01;
			const double cy = (y_min + y_max) * 0.5 + 0.01;
			const double eps = _cfg->center_eps / quad_decimate;
			int32_t dot = 0;
			// Points must be in all quarters.
			uint8_t mask = 0;
			for (uint32_t i = 0; i < size; ++i)
			{
				auto& p = contour[i];
				const double dx = p.x - cx;
				const double dy = p.y - cy;
				//
				const double cos_a = dx * p.gx + dy * p.gy;
				if (cos_a > 0)
					++dot;
				else
					--dot;
				// Calculate the order (from 0 to 64 000) of points.
				// Right or left.
				if (std::abs(dx) > std::abs(dy))
				{
					// Right order: 8000 * [0.0, 2.0].
					if (dx > eps)
					{
						p.order = static_cast<uint16_t>(8000 * (1.0 - dy / dx));
						mask |= 1;
					}
					// Left order: 8000 * [4.0, 6.0].
					else if (dx < -eps)
					{
						p.order = static_cast<uint16_t>(8000 * (5.0 - dy / dx));
						mask |= 2;
					}
					else
						return false;
				}
				// Top or bottom.
				else
				{
					// Top order: 8000 * [2.0, 4.0].
					if (dy < -eps)
					{
						p.order = static_cast<uint16_t>(8000 * (3.0 + dx / dy));
						mask |= 4;
					}
					// Bottom order: 8000 * [6.0, 8.0].
					else if (dy > eps)
					{
						p.order = static_cast<uint16_t>(8000 * (7.0 + dx / dy));
						mask |= 8;
					}
					else
						return false;
				}
			}
			if (mask != 15)
				return false;
			if (std::abs(dot) < size * _cfg->dot_thresh)
				return false;
			bool black = (dot < 0);
			// Drop the contours of the wrong border type.
			uint8_t border_mask = black ? 1 : 2;
			if ((_cfg->border_mask & border_mask) == 0)
				return false;
			quad.black = black;
			// quad.p[0].x = x_min * quad_decimate;
			// quad.p[0].y = y_min * quad_decimate;
			// quad.p[1].x = x_max * quad_decimate;
			// quad.p[1].y = y_min * quad_decimate;
			// quad.p[2].x = x_max * quad_decimate;
			// quad.p[2].y = y_max * quad_decimate;
			// quad.p[3].x = x_min * quad_decimate;
			// quad.p[3].y = y_max * quad_decimate;
			//
			std::sort(contour.begin(), contour.end(), [](const auto& a, const auto& b) {
				return a.order < b.order;
			});
			return true;
		}

		//
		inline bool _check_max_cos(const line_param_t& lp1, const line_param_t& lp2) const
		{
			return (std::abs(lp1.dx * lp2.dx + lp1.dy * lp2.dy) < _cfg->max_cos);
		}

		// Get fit_data from range.
		inline void _get_fit_data(int32_t i0, int32_t i1, fit_data_t& fd) const
		{
			fd = _fit_data[i1];
			if (i0 > 0)
				fd -= _fit_data[i0 - 1];
			if (i0 > i1)
				fd += _fit_data[_fit_data.size() - 1];
		}

		//
		inline void _fit_line_mse(int32_t i0, int32_t i1, double& mse, line_param_t& line_parm) const
		{
			fit_data_t fd;
			_get_fit_data(i0, i1, fd);
			mse = line_parm.calc_point(fd);
		}

		//
		bool _fit_line(int32_t i0, int32_t i1, double& mse, line_param_t& line_parm) const
		{
			fit_data_t fd;
			_get_fit_data(i0, i1, fd);
			mse = line_parm.calc_point(fd);
			if (mse > _cfg->max_line_fit_mse)
				return false;
			line_parm.calc_direction();
			return true;
		}

		// Calculation of tag corners.
		// lp - array of size 4.
		bool _calc_corners(const line_param_t* const lp, quad_t& quad) const
		{
			for (uint32_t i = 0; i < 4; i++)
			{
				// Solve for the intersection of lines (i) and (i+1)&3.
				// p0 + lambda0 * d0 = p1 + lambda1 * d1, where d0 and d1 are the line directions.
				// =>
				// lambda0 * u0 - lambda1 * u1 = p1 - p0
				// =>
				// |d0_x  -d1_x| |lambda0|   |p1_x - p0_x|
				// |d0_y  -d1_y| |lambda1| = |p1_y - p0_y|
				const line_param_t& lp_0 = lp[i];
				const line_param_t& lp_1 = lp[(i + 1) & 3];
				double a00 =  lp_0.dx;
				double a01 = -lp_1.dx;
				double a10 =  lp_0.dy;
				double a11 = -lp_1.dy;
				double b0 = lp_1.px - lp_0.px;
				double b1 = lp_1.py - lp_0.py;
				//
				double det = a00 * a11 - a10 * a01;
				if (std::abs(det) < 0.001)
					return false;
				// Inverse.
				double w00 =  a11 / det;
				double w01 = -a01 / det;
				// Solve.
				double l0 = w00 * b0 + w01 * b1;
				quad.p[i].x = lp_0.px + l0 * lp_0.dx;
				quad.p[i].y = lp_0.py + l0 * lp_0.dy;
			}
			return true;
		}

		void _prepare_fit_data(const image_t& gray_img, const std::vector<cpt_t>& contour)
		{
			const double quad_decimate = _cfg->quad_decimate;
			const uint32_t size = contour.size();
			const uint8_t* const img = gray_img.d;
			const uint32_t iw = gray_img.w;
			const uint32_t ih = gray_img.h;
			_fit_data.clear();
			_fit_data.reserve(size);
			fit_data_t sum;
			for (uint32_t i = 0; i < size; ++i)
			{
				const auto& p = contour[i];
				const double x = p.x * quad_decimate;
				const double y = p.y * quad_decimate;
				double w = 1.0;
				uint32_t ix = static_cast<uint32_t>(x + 0.5);
				uint32_t iy = static_cast<uint32_t>(y + 0.5);
				if (ix > 0 && iy > 0 && ix < iw && iy < ih)
				{
					//  3 | 2
					// ---+---
					//  1 | 0
					const uint32_t p = iy * iw + ix;
					// 0
					uint8_t v = img[p];
					uint8_t i_min = v;
					uint8_t i_max = v;
					// 1
					v = img[p - 1];
					if (v < i_min)
						i_min = v;
					else if (v > i_max)
						i_max = v;
					// 2
					v = img[p - iw];
					if (v < i_min)
						i_min = v;
					else if (v > i_max)
						i_max = v;
					// 3
					v = img[p - iw - 1];
					if (v < i_min)
						i_min = v;
					else if (v > i_max)
						i_max = v;
					w += i_max - i_min;
				}
				//
				sum.w += w;
				sum.wx += w * x;
				sum.wy += w * y;
				sum.wxx += w * x * x;
				sum.wyy += w * y * y;
				sum.wxy += w * x * y;
				_fit_data.emplace_back(sum);
			}
		}

		bool _find_quad(quad_t& quad) const
		{
			const uint32_t size = _fit_data.size();
			std::vector<double> err1(size);
			std::vector<double> err2(size);
			// Collect errors.
			// err1
			{
				// min_contour_size >= 24 => ksz >= 1
				const uint32_t ksz = std::min(static_cast<uint32_t>(20), size / 24);
				line_param_t lp;
				for (uint32_t i = 0; i < size; ++i)
					_fit_line_mse((i + size - ksz) % size, (i + ksz) % size, err1[i], lp);
			}
			// Filtering errors.
			// err2 = filter(err1)
			{
				const uint32_t f_size = _filter.size();
				if (size < f_size)
					return false;
				const uint32_t beg = size - f_size / 2;
				for (uint32_t i = 0; i < size; i++)
				{
					double acc = 0.0;
					for (uint32_t f = 0, j = beg + i; f < f_size; ++f, ++j)
						acc += _filter[f] * err1[j % size];
					err2[i] = acc;
				}
			}
			// Find maxima.
			// err1 = maxima(err2)
			std::vector<uint32_t> maxima;
			{
				const double min_err = 0.01;
				err1.clear();
				maxima.reserve(size / 6);
				if (err2[0] > min_err && err2[0] > err2[size - 1] && err2[0] > err2[1])
				{
					maxima.emplace_back(0);
					err1.emplace_back(err2[0]);
				}
				for (uint32_t i = 1; i < size - 1; i++)
				{
					if (err2[i] > min_err && err2[i] > err2[i - 1] && err2[i] > err2[i + 1])
					{
						maxima.emplace_back(i);
						err1.emplace_back(err2[i]);
					}
				}
				if (err2[size - 1] > min_err && err2[size - 1] > err2[size - 2] && err2[size - 1] > err2[0])
				{
					maxima.emplace_back(size - 1);
					err1.emplace_back(err2[size - 1]);
				}
			}
			uint32_t maxima_size = maxima.size();
			if (maxima_size < 4)
				return false;
			// Get best max_nmaxima.
			if (maxima_size > _cfg->max_nmaxima)
			{
				err2 = err1;
				std::nth_element(err2.begin(), err2.begin() + _cfg->max_nmaxima, err2.end(), std::greater<double>());
				const double tresh = err2[_cfg->max_nmaxima];
				const uint32_t i_max = maxima_size;
				for (uint32_t i = 0, maxima_size = 0; i < i_max; ++i)
				{
					if (err1[i] > tresh)
						maxima[maxima_size++] = maxima[i];
				}
			}
			//
			double err[4];
			double best_err = std::numeric_limits<double>::max();
			line_param_t lp[4];
			line_param_t best_lp[4];
			for (uint32_t m0 = 0; m0 < maxima_size - 3; ++m0)
			{
				const uint32_t i0 = maxima[m0];
				for (int m1 = m0 + 1; m1 < maxima_size - 2; ++m1)
				{
					const uint32_t i1 = maxima[m1];
					if (!_fit_line(i0, i1, err[0], lp[0]))
						continue;
					for (uint32_t m2 = m1 + 1; m2 < maxima_size - 1; ++m2)
					{
						const uint32_t i2 = maxima[m2];
						if (!_fit_line(i1, i2, err[1], lp[1]))
							continue;
						if (!_check_max_cos(lp[0], lp[1]))
							continue;
						for (uint32_t m3 = m2 + 1; m3 < maxima_size; ++m3)
						{
							const uint32_t i3 = maxima[m3];
							if (!_fit_line(i2, i3, err[2], lp[2]))
								continue;
							if (!_check_max_cos(lp[1], lp[2]))
								continue;
							if (!_fit_line(i3, i0, err[3], lp[3]))
								continue;
							if (!_check_max_cos(lp[2], lp[3]))
								continue;
							if (!_check_max_cos(lp[3], lp[0]))
								continue;
							double e = err[0] + err[1] + err[2] + err[3];
							if (e < best_err)
							{
								best_err = e;
								std::memcpy(best_lp, lp, 4 * sizeof(line_param_t));
							}
						}
					}
				}
			}
			if (best_err == std::numeric_limits<double>::max())
				return false;
			//
			if (!_calc_corners(best_lp, quad))
				return false;
			// Check tag size.
			const uint32_t min_tag_size = _cfg->min_tag_size * _cfg->min_tag_size;
			for (uint32_t i = 0; i < 4; i++)
			{
				const auto& p1 = quad.p[i];
				const auto& p2 = quad.p[(i + 1) & 3];
				double dx21 = p1.x - p2.x;
				double dy21 = p1.y - p2.y;
				if (dx21 * dx21 + dy21 * dy21 < min_tag_size)
					return false;
				// Check convex.
				const auto& p3 = quad.p[(i + 2) & 3];
				double dx23 = p3.x - p2.x;
				double dy23 = p3.y - p2.y;
				if (dx21 * dy23 < dy21 * dx23)
					return false;
			}
			// Area of a convex quadrilateral.
			const auto& p = quad.p;
			double area = p[0].x * p[3].y - p[3].x * p[0].y;
			for (uint32_t i = 0; i < 3; ++i)
				area += p[i + 1].x * p[i].y - p[i].x * p[i + 1].y;
			area *= 0.5;
			// Reject quads that are too small.
			if (area < _cfg->min_tag_area)
				return false;
			return true;
		}

		// 
		bool _refine_edges(const image_t& gray_img, quad_t& quad) const
		{
			const uint8_t* const img = gray_img.d;
			const uint32_t w = gray_img.w;
			const uint32_t h = gray_img.h;
			line_param_t lp[4];
			for (uint32_t i = 0; i < 4; ++i)
			{
				const pt_t pa = quad.p[i];
				const pt_t pb = quad.p[(i + 1) & 3];
				// Compute the normal to the current line estimate.
				double nx = pb.y - pa.y;
				double ny = pa.x - pb.x;
				double mag = std::sqrt(nx * nx + ny * ny);
				nx /= mag;
				ny /= mag;
				if (quad.black)
				{
					nx = -nx;
					ny = -ny;
				}
				// We will now fit a NEW line by sampling points near our original line that have large gradients.
				// On really big tags, we're willing to sample more to get an even better estimate.
				uint32_t nsamples = static_cast<uint32_t>(mag / 8);
				if (nsamples < 16)
					nsamples = 16;
				// How far to search?
				// We want to search far enough that we find the best edge, but not so far that we hit other edges that aren't part of the tag.
				// We shouldn't ever have to search more than quad_decimate, since otherwise we would (ideally) have started our search on another pixel in the first place.
				// Likewise, for very small tags, we don't want the range to be too big.
				const double range = _cfg->quad_decimate + 1.0;
				// How far +/- to look?
				// Small values compute the gradient more precisely, but are more sensitive to noise.
				const double grange = _cfg->grange;
				// Stats for fitting a line.
				fit_data_t sum;
				for (uint32_t s = 0; s < nsamples; ++s)
				{
					// Compute a point along the line.
					// Note, we're avoiding sampling *right* at the corners, since those points are the least reliable.
					const double alpha = (1.0 + s) / (nsamples + 1);
					const double x0 = pb.x + alpha * (pa.x - pb.x);
					const double y0 = pb.y + alpha * (pa.y - pb.y);
					// Search along the normal to this line, looking at the gradients along the way.
					// We're looking for a strong response.
					// Because of the guaranteed winding order of the points in the quad, we will start inside the white portion of the quad and work our way outward.
					double wn_sum = 0.0;
					double w_sum = 0.0;
					for (double n = -range; n <= range; n += 0.25)
					{
						// Sample to points (x1,y1) and (x2,y2).
						int x1 = static_cast<int>(x0 + (n + grange) * nx);
						int y1 = static_cast<int>(y0 + (n + grange) * ny);
						if (x1 < 0 || x1 >= w || y1 < 0 || y1 >= h)
							continue;
						int x2 = static_cast<int>(x0 + (n - grange) * nx);
						int y2 = static_cast<int>(y0 + (n - grange) * ny);
						if (x2 < 0 || x2 >= w || y2 < 0 || y2 >= h)
							continue;
						uint8_t g1 = img[y1 * w + x1];
						uint8_t g2 = img[y2 * w + x2];
						// Reject points whose gradient is "backwards".
						// They can only hurt us.
						if (g1 < g2)
							continue;
						double dg = g1 - g2;
						// What shape for weight=f(g2-g1)?
						double weight = dg * dg;
						// Compute weighted average of the gradient at this point.
						wn_sum += weight * n;
						w_sum += weight;
					}
					// What was the average point along the line?
					if (w_sum < 0.1)
						continue;
					double n0 = wn_sum / w_sum;
					// Where is the point along the line?
					double bestx = x0 + n0 * nx;
					double besty = y0 + n0 * ny;
					// Update our line fit statistics.
					sum.wx += bestx;
					sum.wy += besty;
					sum.wxx += bestx * bestx;
					sum.wxy += bestx * besty;
					sum.wyy += besty * besty;
					sum.w += 1.0;
				}
				if (sum.w < 0.1)
					return false;
				lp[i].calc_point(sum);
				lp[i].calc_direction();
			}
			return _calc_corners(lp, quad);
		}

	public:
		Quad(const cfg_t* cfg) :
			_cfg(cfg)
		{
			const double sigma = 1.0;
			const double cutoff = 0.05;
			const int fsz = std::sqrt(-std::log(cutoff) * 2.0 * sigma * sigma) + 1.0;
			_filter.resize(2 * fsz + 1);
			for (int i = -fsz; i <= fsz; ++i)
				_filter[i + fsz] = std::exp(-i * i / (2.0 * sigma * sigma));
		}

		//
		const std::vector<quad_t>& calc(std::vector<std::vector<cpt_t>>& contours, const image_t& gray_img)
		{
			const uint32_t size = contours.size();
			_quads.clear();
			_quads.reserve(size);
			for (uint32_t i = 0; i < size; ++ i)
			{
				quad_t quad;
				if (!_sort_contour(contours[i], quad))
					continue;
				_prepare_fit_data(gray_img, contours[i]);
				if (!_find_quad(quad))
					continue;
				if (_cfg->refine_edges && !_refine_edges(gray_img, quad))
					continue;
				_quads.emplace_back(quad);
			}
			return _quads;
		}
	};
}