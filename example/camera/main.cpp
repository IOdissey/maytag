#include <array>
#include <chrono>
#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

#include <maytag/maytag.h>


void draw_tag(cv::Mat& img, std::array<cv::Point2d, 4>& pt, uint16_t id)
{
	const cv::Scalar color_top(0, 255, 0);
	const cv::Scalar color(255, 0, 0);
	const int line_w = 1;
	const int text_w = 2;
	cv::line(img, pt[0], pt[1], color_top, line_w);
	for (int j = 1; j < 4; ++j)
		cv::line(img, pt[j], pt[(j + 1) & 3], color, line_w);
	//
	cv::circle(img, pt[0], line_w + 2, color_top, -1);
	for (int j = 1; j < 4; ++j)
		cv::circle(img, pt[j], line_w + 2, color, -1);
	//
	int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
	double fontscale = 0.7;
	int baseline;
	std::string text = std::to_string(id);
	cv::Point2d c = (pt[0] + pt[1]+ pt[2] + pt[3]) / 4;
	cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2, &baseline);
	cv::putText(img, text, cv::Point(c.x - textsize.width / 2, c.y + textsize.height / 2), fontface, fontscale, cv::Scalar(0, 0, 255), text_w);
}

int main(int argc, char* argv[])
{
	const std::string keys =
		"{h help     |         | help}"
		"{d device   | 0       | camera device number}"
		"{iw width   | 0       | image width (optionaly)}"
		"{ih height  | 0       | image height (optionaly)}"
		"{f family   | tag16h5 | tag family: tag16h5, tag25h9, tag36h10, tag36h11}"
		"{b black    | 1       | tag color: 1 - black, 0 - white}"
		"{ha hamming | 1       | number of error correction bits (hamming distance)}"
		"{x decimate | 1.0     | decimate input image by this factor (supported 1, 1.5, 2, 3, ...)}"
		"{r refine   | 1       | spend more time trying to align edges of tags: 1 - on, 0 - off}";

	cv::CommandLineParser parser(argc, argv, keys);

	if (parser.has("help"))
	{
		std::cout << "OpenCV: " << CV_VERSION << std::endl;
		parser.printMessage();
		return 0;
	}

	const int arg_device = parser.get<int>("device");
	cv::VideoCapture cap(arg_device);
	if (!cap.isOpened())
	{
		std::cout << "Couldn't open video capture device (" << arg_device << ")." << std::endl;
		return -1;
	}
	const int arg_width = parser.get<int>("width");
	if (arg_width > 0)
		cap.set(cv::CAP_PROP_FRAME_WIDTH, arg_width);
	const int arg_height = parser.get<int>("height");
	if (arg_height > 0)
		cap.set(cv::CAP_PROP_FRAME_HEIGHT, arg_height);

	const std::string family = parser.get<std::string>("family");
	const bool black = parser.get<bool>("black");
	const int hamming = parser.get<int>("hamming");
	const double decimate = parser.get<float>("decimate");
	const bool refine = parser.get<bool>("refine");

	maytag::Detector detector;
	detector.set_quad_decimate(decimate);
	detector.set_refine_edges(refine);
	detector.set_dict_stat(true);
	if (family == "tag16h5")
		detector.add_family(maytag::tag16h5(black, hamming));
	else if (family == "tag25h9")
		detector.add_family(maytag::tag25h9(black, hamming));
	else if (family == "tag36h10")
		detector.add_family(maytag::tag36h10(black, hamming));
	else if (family == "tag36h11")
		detector.add_family(maytag::tag36h11(black, hamming));
	else
	{
		std::cout << "Unrecognized tag family name (" << family << ")." << std::endl;
		return -1;
	}

	cv::Mat frame, gray;
	while (true)
	{
		cap >> frame;
		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
		maytag::image_t gray_img(gray.cols, gray.rows, gray.data);

		auto beg = std::chrono::steady_clock::now();
		const auto& tags = detector.calc(gray_img);
		auto end = std::chrono::steady_clock::now();
		auto dt = std::chrono::duration_cast<std::chrono::nanoseconds>(end - beg).count() * 1e-9;
		std::cout << "maytag dt = " << dt << std::endl;

		// Draw all tags.
		std::array<cv::Point2d, 4> pt;
		for (size_t i = 0; i < tags.size(); ++i)
		{
			const auto& tag = tags[i];
			for (int j = 0; j < 4; ++j)
			{
				pt[j].x = tag.p[j].x;
				pt[j].y = tag.p[j].y;
			}
			draw_tag(frame, pt, tag.id);
		}

		cv::imshow("maytag", frame);
		int key = cv::waitKey(1);
		if (key == 27)
			break;
	}
	return 0;
}