#include <stdexcept>
#include <opencv2/opencv.hpp>
#include "core/io/camera/public.hpp"

namespace io {

	cv::Mat get_flightgear_camera() {
		cv::VideoCapture cap("http://127.0.0.1:8080/screenshot");

		if (!cap.isOpened()) {
			throw std::runtime_error("Failed to open FlightGear camera stream");
		}

		cv::Mat frame;
		cap >> frame;

		if (frame.empty()) {
			throw std::runtime_error("Received empty frame");
		}

		return frame;
	}

}
