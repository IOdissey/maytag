#include <iostream>
#include <string>
#include <experimental/filesystem>

#include <opencv2/opencv.hpp>

#include <maytag/tag_family.h>
#include <maytag/tag16h5.h>
#include <maytag/tag25h9.h>
#include <maytag/tag36h10.h>
#include <maytag/tag36h11.h>

namespace fs = std::experimental::filesystem;

int main(int argc, char* argv[])
{
	const std::string keys =
		"{f family | tag16h5 | tag family (tag16h5, tag25h9, tag36h10, tag36h11)}"
		"{s size   | 1       | bit size in pixel (> 0)}"
		"{b black  | 1       | tag color (1 - black, 0 - white)}"
		"{h help   |         | help}";

	cv::CommandLineParser parser(argc, argv, keys);

	if (parser.has("help"))
	{
		std::cout << "OpenCV: " << CV_VERSION << std::endl;
		parser.printMessage();
		return 0;
	}

	const std::string family = parser.get<std::string>("family");
	const bool black = parser.get<bool>("black");
	int size = parser.get<int>("size");
	if (size < 1)
		size = 1;

	maytag::tag_family_t tag_family;
	if (family == "tag16h5")
		tag_family = maytag::tag16h5(black);
	else if (family == "tag25h9")
		tag_family = maytag::tag25h9(black);
	else if (family == "tag36h10")
		tag_family = maytag::tag36h10(black);
	else if (family == "tag36h11")
		tag_family = maytag::tag36h11(black);
	else
	{
		std::cout << "Unrecognized tag family name '" << family << "'." << std::endl;
		return -1;
	}

	uint8_t val_b;
	uint8_t val_w;
	std::string path = "tags/";
	if (!fs::exists(path))
		fs::create_directory(path);
	path += tag_family.name;
	if (black)
	{
		val_w = 255;
		val_b = 0;
		path += "-b-";
	}
	else
	{
		val_w = 0;
		val_b = 255;
		path += "-w-";
	}
	path += std::to_string(size) + "/";
	if (!fs::exists(path))
		fs::create_directory(path);

	std::vector<int> params;
	params.push_back(cv::IMWRITE_PNG_COMPRESSION);
	params.push_back(9);
	params.push_back(cv::IMWRITE_PNG_BILEVEL);
	params.push_back(1);

	const uint32_t ncodes = tag_family.ncodes;
	const uint32_t nbits = tag_family.nbits;
	const uint32_t tw = tag_family.total_width;
	const uint32_t wb = tag_family.width_at_border;
	const uint32_t offset = (tw - wb) / 2;
	const uint32_t beg = offset * tw + offset;
	const uint32_t nz = 1 + static_cast<uint32_t>(std::log10(ncodes));

	cv::Mat big;
	cv::Mat img(tw, tw, CV_8UC1, cv::Scalar(val_w));
	uint8_t* ptr = img.data;

	// Border.
	for (uint32_t i = 0; i < wb; ++i)
	{
		uint32_t idx = beg + i;
		ptr[idx] = val_b;
	}
	for (uint32_t i = 0; i < wb; ++i)
	{
		uint32_t idx = beg + (wb - 1) * tw + i;
		ptr[idx] = val_b;
	}
	for (uint32_t i = 0; i < wb; ++i)
	{
		uint32_t idx = beg + i * tw;
		ptr[idx] = val_b;
	}
	for (uint32_t i = 0; i < wb; ++i)
	{
		uint32_t idx = beg + wb - 1 + i * tw;
		ptr[idx] = val_b;
	}

	//
	for (uint32_t n = 0; n < ncodes; ++n)
	{
		uint64_t code = tag_family.codes[n];
		for (uint32_t b = 0; b < nbits; ++b)
		{
			uint8_t bx = tag_family.bit_x[b];
			uint8_t by = tag_family.bit_y[b];
			uint32_t idx = beg + by * tw + bx;
			if ((code >> (nbits - b - 1) & 1) == 1)
				ptr[idx] = val_w;
			else
				ptr[idx] = val_b;
		}

		if (size > 1)
			cv::resize(img, big, cv::Size(), size, size, cv::INTER_NEAREST);
		else
			big = img;
		std::ostringstream str;
		str << path << std::setw(nz) << std::setfill('0') << n << ".png";
		std::cout << str.str() << std::endl;
		cv::imwrite(str.str(), big, params);
	}

	return 0;
}