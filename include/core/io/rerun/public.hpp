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

namespace io {

    struct RerunFrame {
        int t;
        DataContext context;
        cv::Mat camera_frame;
    };

    struct RerunManager {
        bool rerun_bool;
        bool control_bool;
        bool avionics_bool;
        bool estimation_bool;
        bool wind_bool;

        double logging_hz;
        std::size_t max_traj_size = static_cast<std::size_t>(30.0 * logging_hz);  // 30 seconds
        std::size_t max_queue_size = static_cast<std::size_t>(2.0 * logging_hz);    // 2 seconds

        // in pixels
        int img_width = 300;
        int img_height = 300;

        rerun::RecordingStream rec;
        std::vector<rerun::Vec3D> trajectory;

        std::deque<RerunFrame> frame_queue;
        std::mutex queue_mutex;
        std::condition_variable queue_cv;
        std::thread worker;
        bool stop_worker = false;

        std::thread camera_worker;
        std::atomic_bool stop_camera_worker = false;
        std::mutex camera_mutex;
        cv::Mat latest_camera_frame;

        void step(int t, const DataContext& context);
        void stream_frame(const RerunFrame& sample);
        void run_worker();
        void run_camera_worker();

        RerunManager(bool rerun_bool, bool control_bool, bool avionics_bool, bool estimation_bool, bool wind_bool, double logging_hz);
        ~RerunManager();
    };
}
