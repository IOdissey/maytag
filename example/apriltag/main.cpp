#include <array>
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

#include <apriltag/apriltag.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag36h10.h>
#include <apriltag/tag36h11.h>


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

class MayTag
{
private:
	maytag::Detector _detector;

public:
	MayTag(const std::string& family, bool black, int hamming, double decimate, bool refine)
	{
		_detector.set_quad_decimate(decimate);
		_detector.set_refine_edges(refine);
		if (family == "tag16h5")
			_detector.add_family(maytag::tag16h5(black, hamming));
		else if (family == "tag25h9")
			_detector.add_family(maytag::tag25h9(black, hamming));
		else if (family == "tag36h10")
			_detector.add_family(maytag::tag36h10(black, hamming));
		else if (family == "tag36h11")
			_detector.add_family(maytag::tag36h11(black, hamming));
		else
			std::cout << "Unrecognized tag family name (" << family << ")." << std::endl;
	}

	void detect(const cv::Mat& frame)
	{
		cv::Mat gray;
		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
		maytag::image_t gray_img(gray.cols, gray.rows, gray.data);

		auto beg = std::chrono::steady_clock::now();
		const auto& tags = _detector.calc(gray_img);
		auto end = std::chrono::steady_clock::now();
		auto dt = std::chrono::duration_cast<std::chrono::nanoseconds>(end - beg).count() * 1e-9;
		std::cout << "MayTag   dt = " << dt << std::endl;

		cv::Mat img = frame.clone();
		std::array<cv::Point2d, 4> pt;
		for (size_t i = 0; i < tags.size(); ++i)
		{
			const auto& tag = tags[i];
			for (int j = 0; j < 4; ++j)
			{
				pt[j].x = tag.p[j].x;
				pt[j].y = tag.p[j].y;
			}
			draw_tag(img, pt, tag.id);
		}
		cv::imshow("MayTag", img);
	}
};

class AprilTag
{
private:
	apriltag_detector_t* _td = nullptr;
	apriltag_family_t* _tf = nullptr;
	const std::string _family;

public:
	AprilTag(const std::string& family, bool black, int hamming, double decimate, bool refine):
		_family(family)
	{
		_td = apriltag_detector_create();
		_td->quad_decimate = decimate;
		_td->refine_edges = refine;
		_td->qtp.max_line_fit_mse = 2.0;
		_td->qtp.min_white_black_diff = 40;
		_td->qtp.min_cluster_pixels = 24;
		_td->qtp.cos_critical_rad = std::cos(25 * M_PI / 180);
		if (family == "tag16h5")
			_tf = tag16h5_create();
		else if (family == "tag25h9")
			_tf = tag25h9_create();
		else if (family == "tag36h10")
			_tf = tag36h10_create();
		else if (family == "tag36h11")
			_tf = tag36h11_create();
		else
		{
			std::cout << "Unrecognized tag family name (" << family << ")." << std::endl;
			return;
		}
		_tf->reversed_border = !black;
		apriltag_detector_add_family_bits(_td, _tf, hamming);
	}

	~AprilTag()
	{
		if (_td)
			apriltag_detector_destroy(_td);
		if (_tf)
		{
			if (_family == "tag16h5")
				tag16h5_destroy(_tf);
			else if (_family == "tag25h9")
				tag25h9_destroy(_tf);
			else if (_family == "tag36h10")
				tag36h10_destroy(_tf);
			else if (_family == "tag36h11")
				tag36h11_destroy(_tf);
		}
	}

	void detect(const cv::Mat& frane)
	{
		cv::Mat gray;
		cv::cvtColor(frane, gray, cv::COLOR_BGR2GRAY);
		image_u8_t im = {
			.width = gray.cols,
			.height = gray.rows,
			.stride = gray.cols,
			.buf = gray.data
		};

		auto beg = std::chrono::steady_clock::now();
		zarray_t *detections = apriltag_detector_detect(_td, &im);
		auto end = std::chrono::steady_clock::now();
		auto dt = std::chrono::duration_cast<std::chrono::nanoseconds>(end - beg).count() * 1e-9;
		std::cout << "AprilTag dt = " << dt << std::endl;

		cv::Mat img = frane.clone();
		std::array<cv::Point2d, 4> pt;
		for (int i = 0; i < zarray_size(detections); i++)
		{
			apriltag_detection_t *det;
			zarray_get(detections, i, &det);
			for (int j = 0; j < 4; ++j)
			{
				pt[j].x = det->p[j][0];
				pt[j].y = det->p[j][1];
			}
			draw_tag(img, pt, det->id);
		}
		cv::imshow("AprilTag", img);
		apriltag_detections_destroy(detections);
	}
};

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

	const std::string family = parser.get<std::string>("family");
	const bool black = parser.get<bool>("black");
	const int hamming = parser.get<int>("hamming");
	const double decimate = parser.get<float>("decimate");
	const bool refine = parser.get<bool>("refine");

	MayTag maytag(family, black, hamming, decimate, refine);
	AprilTag apriltag(family, black, hamming, decimate, refine);

	cv::Mat frame, gray;
	while (true)
	{
		cap >> frame;

		std::cout << std::endl;
		maytag.detect(frame);
		apriltag.detect(frame);

		int key = cv::waitKey(1);
		if (key == 27)
			break;
	}

	return 0;
}