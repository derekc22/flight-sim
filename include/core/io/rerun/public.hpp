#pragma once
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <rerun.hpp>
#include <thread>
#include <vector>
#include "core/io/data/public.hpp"
#include "simulation/transforms/public.hpp"
#include "simulation/constants/public.hpp"

namespace io {

    struct RerunContext {
        int t;
        DataContext data_context;
        cv::Mat image;
    };

    struct RerunManager {
        bool rerun_flag;
        bool control_flag;
        bool avionics_flag;
        bool estimation_flag;
        bool wind_flag;

        double log_hz;
        std::size_t max_traj_size = static_cast<std::size_t>(30.0 * log_hz);  // 30 seconds
        std::size_t max_queue_size = static_cast<std::size_t>(2.0 * log_hz);  // 2 seconds

        // in pixels
        int image_width = 500;
        int image_height = 500;

        rerun::RecordingStream rec;

        Eigen::Quaterniond q_model_to_body = transforms::eul_to_quatR(
            -constants::pi/2, constants::pi/2, 0.0, "ZYX"
        );

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

        void step(int t, const DataContext& data_context);
        void stream_context(const RerunContext& sample);
        void run_worker();
        void run_camera_worker();

        RerunManager(bool rerun_flag, bool control_flag, bool avionics_flag, bool estimation_flag, bool wind_flag, double log_hz);
        ~RerunManager();
    };
}
