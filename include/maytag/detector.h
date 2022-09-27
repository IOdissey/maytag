#pragma once

#include <cstdint>
#include <cstring>

#include "cfg.h"
#include "image.h"
#include "decimate.h"
#include "threshold.h"
#include "contours.h"
#include "quad.h"
#include "decode.h"
#include "dictionary.h"


namespace maytag
{
	using namespace _;

	class Detector
	{
	private:
		cfg_t _cfg;
		Decimate _decimate;
		Threshold _threshold;
		Contours _contours;
		Quad _quad;
		Decode _decode;

	public:
		Detector():
			_decimate(&_cfg),
			_threshold(&_cfg),
			_contours(&_cfg),
			_quad(&_cfg),
			_decode(&_cfg)
		{
		}

		const std::vector<tag_t>& calc(const image_t& gray_img)
		{
			image_t decimate_img = _decimate.calc(gray_img);
			image_t thresh_img = _threshold.calc(decimate_img);
			auto& contours = _contours.calc(thresh_img);
			const auto& quads = _quad.calc(contours, gray_img);
			return _decode.calc(quads, gray_img);
		}

		// quad_decimate = 1, 1.5, 2, 3, ...
		void set_quad_decimate(double quad_decimate)
		{
			// 0, 1
			if (quad_decimate < 1.4)
			{
				_cfg.quad_decimate_type = 1;
				_cfg.quad_decimate = 1.0;
			}
			// 2, 3, ...
			else if (quad_decimate > 1.6)
			{
				_cfg.quad_decimate_type = static_cast<uint32_t>(quad_decimate + 0.5);
				_cfg.quad_decimate =  static_cast<double>(_cfg.quad_decimate_type);
			}
			// 1.5
			else
			{
				_cfg.quad_decimate_type = 0;
				_cfg.quad_decimate = 1.5;
			}
		}

		//
		void set_tile_size(uint32_t tile_size)
		{
			if (tile_size < 2)
				_cfg.tile_size = 2;
			else
				_cfg.tile_size = tile_size;
		}

		//
		void set_min_wb_diff(uint8_t min_wb_diff)
		{
			_cfg.min_wb_diff = min_wb_diff;
		}

		//
		void set_min_contour_size(uint32_t min_contour_size)
		{
			if (min_contour_size < 24)
				_cfg.min_contour_size = 24;
			else
				_cfg.min_contour_size = min_contour_size;
		}

		//
		void set_min_tag_size(uint32_t min_tag_size)
		{
			if (min_tag_size < 5)
				_cfg.min_tag_size = 5;
			else
				_cfg.min_tag_size = min_tag_size;
		}

		//
		void set_min_tag_area(double min_tag_area)
		{
			_cfg.min_tag_area = min_tag_area;
		}

		//
		void set_max_line_fit_mse(double max_line_fit_mse)
		{
			_cfg.max_line_fit_mse = max_line_fit_mse;
		}

		//
		void set_max_cos(double max_cos)
		{
			_cfg.max_cos = max_cos;
		}

		//
		void set_refine_edges(bool refine_edges)
		{
			_cfg.refine_edges = refine_edges;
		}

		//
		void set_min_score(double min_score)
		{
			_cfg.min_score = min_score;
		}

		//
		void set_interpolate(bool interpolate)
		{
			_cfg.interpolate = interpolate;
		}

		// dict_size_scale - specifies the size of the dictionary.
		// The larger the value, the larger the size but the faster the search.
		// For the change to take effect, you must install before call add_family.
		void add_family(const tag_family_t& tf, double dict_size_scale = 3.0)
		{
			if (tf.width_at_border < 2)
				return;
			if (tf.total_width < tf.width_at_border + 2)
				return;
			if (tf.total_width > _cfg.max_total_width)
				_cfg.max_total_width = tf.total_width;
			if (tf.black)
				_cfg.border_mask |= 1;
			else
				_cfg.border_mask |= 2;
			// Checking for duplicates.
			uint32_t size = _cfg.tag_family.size();
			for (uint32_t i = 0; i < size; ++i)
			{
				if (tf.name == _cfg.tag_family[i].name)
				{
					// Expand tag variability if needed.
					_cfg.tag_dict[i]->update_hamming(tf.hamming, dict_size_scale);
					// We use the same dictionaries.
					if (tf.black != _cfg.tag_family[i].black)
					{
						_cfg.tag_family.emplace_back(tf);
						_cfg.tag_dict.emplace_back(_cfg.tag_dict[i]);
					}
					else
						_cfg.tag_family[i].hamming = tf.hamming;
					return;
				}
			}
			_cfg.tag_family.emplace_back(tf);
			_cfg.tag_dict.emplace_back(std::make_shared<Dictionary>(tf, dict_size_scale));
		}

		//
		void clear_family()
		{
			_cfg.border_mask = 0;
			_cfg.max_total_width = 0;
			_cfg.tag_family.clear();
			_cfg.tag_dict.clear();
		}
	};
}