#pragma once
#include "core/io/rerun/public/data/types.hpp"
#include "simulation/constants/public/scalars.hpp"
#include "simulation/runner/public/data/types.hpp"
#include "simulation/transforms/public/detail/s3.hpp"

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <rerun.hpp>
#include <thread>
#include <vector>

namespace io
{

	struct RerunManager {
		runner::JSONFlags json_flags;

		double log_hz;
		std::size_t max_traj_size;
		std::size_t max_queue_size;

		// in pixels
		int image_width = 500;
		int image_height = 500;

		rerun::RecordingStream rec;

		Eigen::Quaterniond q_model_to_body =
			transforms::eul_to_quatR(-constants::pi / 2, constants::pi / 2, 0.0, transforms::EulerOrder::ZYX);

		std::vector<rerun::Vec3D> trajectory;
		std::vector<rerun::Vec3D> estimated_trajectory;

		std::deque<RerunContext> context_queue;
		std::mutex queue_mutex;
		std::condition_variable queue_cv;
		std::thread worker;
		bool stop_worker = false;

		std::thread camera_worker;
		std::atomic_bool stop_camera_worker = false;
		std::mutex camera_mutex;
		cv::Mat latest_image;

		void step(const RerunManagerInput& input);
		void stream_context(const RerunContext& context);
		void run_worker();
		void run_camera_worker();

		RerunManager(const runner::JSONFlags& json_flags, double log_hz);
		~RerunManager();
	};

} // namespace io
