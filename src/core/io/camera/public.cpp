#include <stdexcept>
#include <opencv2/opencv.hpp>
#include "core/io/camera/public.hpp"

namespace io {

	cv::Mat get_flightgear_image(int width, int height) {
		cv::VideoCapture cap("http://127.0.0.1:8080/screenshot");

		if (!cap.isOpened()) {
			throw std::runtime_error("Failed to open FlightGear camera stream");
		}

		cv::Mat frame;
		cap >> frame;

		if (frame.empty()) {
			throw std::runtime_error("Received empty frame");
		}

		cv::Mat resized;
		cv::resize(frame, resized, cv::Size(width, height), 0, 0, cv::INTER_AREA);

		return resized;
	}

}
