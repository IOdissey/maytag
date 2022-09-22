#include <chrono>
#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

#include <maytag/detector.h>
#include <maytag/image.h>
#include <maytag/tag.h>
#include <maytag/tag16h5.h>
#include <maytag/tag25h9.h>
#include <maytag/tag36h10.h>
#include <maytag/tag36h11.h>


void draw(cv::Mat& img, const std::vector<maytag::tag_t>& tags)
{
	const cv::Scalar color_top(0, 255, 0);
	const cv::Scalar color(255, 0, 0);
	const int line_w = 1;
	const int text_w = 1;
	for (size_t i = 0; i < tags.size(); ++i)
	{
		const auto& tag = tags[i];
		cv::Point2d pt[4];
		for (int j = 0; j < 4; ++j)
		{
			pt[j].x = tag.p[j].x;
			pt[j].y = tag.p[j].y;
		}
		//
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
		std::string text = std::to_string(tag.id);
		cv::Point2d c = (pt[0] + pt[1]+ pt[2] + pt[3]) / 4;
		cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2, &baseline);
		cv::putText(img, text, cv::Point(c.x - textsize.width / 2, c.y + textsize.height / 2), fontface, fontscale, cv::Scalar(0, 0, 255), 2);
	}
}

int main(int argc, char* argv[])
{
	const std::string keys =
		"{h help     |         | print this message}"
		"{d device   | 0       | camera device number}"
		"{iw width   | 0       | image width}"
		"{ih height  | 0       | image height}"
		"{f family   | tag16h5 | tag family to use (tag16h5, tag25h9, tag36h10, tag36h11)}"
		"{b black    | 1       | tag color (1 - black, 0 - white)}"
		"{ha hamming | 1       | How many errors corrected?}"
		"{x decimate | 1.0     | decimate input image by this factor}"
		"{r refine   | 1       | spend more time trying to align edges of tags}";
	cv::CommandLineParser parser(argc, argv, keys);
	if (parser.has("help"))
	{
		parser.printMessage();
		return 0;
	}

	std::cout << "OpenCV: " << CV_VERSION << std::endl;

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

	maytag::Detector detector;
	detector.set_quad_decimate(parser.get<float>("decimate"));
	detector.set_refine_edges(parser.get<bool>("refine"));

	const std::string arg_family = parser.get<std::string>("family");
	const bool black = parser.get<bool>("black");
	const int hamming = parser.get<int>("hamming");
	if (arg_family == "tag16h5")
		detector.add_family(maytag::tag16h5(black, hamming));
	else if (arg_family == "tag25h9")
		detector.add_family(maytag::tag25h9(black, hamming));
	else if (arg_family == "tag36h10")
		detector.add_family(maytag::tag36h10(black, hamming));
	else if (arg_family == "tag36h11")
		detector.add_family(maytag::tag36h11(black, hamming));
	else
	{
		std::cout << "Unrecognized tag family name (" << arg_family << ")." << std::endl;
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

		draw(frame, tags);
		cv::imshow("maytag", frame);
		int key = cv::waitKey(1);
		if (key == 27)
			break;
	}
	return 0;
}