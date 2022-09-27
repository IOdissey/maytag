#pragma once

#include <cstdint>
#include <memory>

#include "pt.h"
#include "tag_family.h"
#include "dictionary.h"


namespace maytag::_
{
	struct cfg_t
	{
		//
		uint32_t quad_decimate_type = 1;
		double quad_decimate = 1.0;
		// 
		uint32_t tile_size = 4;         // min value = 2
		uint8_t min_wb_diff = 40;
		//
		uint32_t min_contour_size = 24;
		//
		uint32_t min_tag_size = 6;
		double min_tag_area = 36.0;
		double max_cos = 0.99;          // cos (25 grad) = 0.9
		double dot_thresh = 0.2;
		double center_eps = 0.5;
		double max_line_fit_mse = 2.0;
		uint32_t max_nmaxima = 10;
		bool refine_edges = true;

		// How much sharpening should be done to decoded images?
		// This can help decode small tags but may or may not help in odd lighting conditions or low light conditions.
		double decode_sharpening = 0.25;
		double min_score = 20.0;
		bool interpolate = false;

		uint8_t border_mask = 0;
		uint32_t max_total_width = 0;
		std::vector<tag_family_t> tag_family;
		std::vector<std::shared_ptr<Dictionary>> tag_dict;
	};
}