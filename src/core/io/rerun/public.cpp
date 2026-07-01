#include <cstdlib>
#include <exception>
#include <chrono>
#include <thread>
#include <utility>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include "core/io/camera/public.hpp"
#include "core/io/rerun/public.hpp"
#include "core/io/rerun/private.hpp"
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"

namespace io {

    RerunManager::RerunManager(bool rerun_bool, bool control_bool, bool avionics_bool, bool estimation_bool, bool wind_bool, double logging_hz)
        : rerun_bool(rerun_bool), control_bool(control_bool), avionics_bool(avionics_bool), estimation_bool(estimation_bool), wind_bool(wind_bool), logging_hz(logging_hz), rec("flight_sim")
    {
        if (rerun_bool) {
            auto server_uri = rec.serve_grpc("0.0.0.0", 9876, "1GiB").value;
            std::system("rerun assets/default.rbl --connect rerun+http://127.0.0.1:9876/proxy &");

            rec.log_static("/", rerun::ViewCoordinates::FRD);
            stream_vehicle_model(rec, q_model_to_body, "world/vehicle/frame");
            if (estimation_bool) {
                stream_vehicle_model(rec, q_model_to_body, "world/estimated_vehicle/frame");
            }

            camera_worker = std::thread(&RerunManager::run_camera_worker, this);
            worker = std::thread(&RerunManager::run_worker, this);
        }
    }

    RerunManager::~RerunManager() {
        stop_camera_worker = true;
        if (camera_worker.joinable()) {
            camera_worker.join();
        }

        if (!worker.joinable()) {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            stop_worker = true;
        }

        queue_cv.notify_one();
        worker.join();
    }

    void RerunManager::step(int t, const DataContext& data_context) {
        if (!rerun_bool) {
            return;
        }

        cv::Mat image;
        {
            std::lock_guard<std::mutex> lock(camera_mutex);
            image = latest_image;
        }

        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            if (context_queue.size() >= max_queue_size) {
                context_queue.pop_front();
            }
            context_queue.push_back(RerunContext{.t=t, .data_context=data_context, .image=image});
        }

        queue_cv.notify_one();
    }

    void RerunManager::stream_context(const RerunContext& context) {
        rec.set_time_sequence("step", context.t);
        rec.set_time_duration_secs("sim_time", context.t * constants::dt);

        stream_vehicle_transform(rec, context.data_context.Xt);

        if (estimation_bool) {
            stream_estimated_vehicle_transform(rec, context.data_context.Zt);
        }

        trajectory.emplace_back(
            static_cast<float>(context.data_context.Xt.p.data.x()),
            static_cast<float>(context.data_context.Xt.p.data.y()),
            static_cast<float>(context.data_context.Xt.p.data.z())
        );
        clip_trajectory(trajectory, max_traj_size);
        stream_vehicle_trajectory(rec, trajectory);

        if (estimation_bool) {
            estimated_trajectory.emplace_back(
                static_cast<float>(context.data_context.Zt.p.data.x()),
                static_cast<float>(context.data_context.Zt.p.data.y()),
                static_cast<float>(context.data_context.Zt.p.data.z())
            );
            clip_trajectory(estimated_trajectory, max_traj_size);
            stream_estimated_vehicle_trajectory(rec, estimated_trajectory);
        }

        stream_body_arrow(rec, "world/vehicle/vectors/v", context.data_context.Xt.v.data, 1.0, rerun::Color(0, 180, 255), "v");
        stream_body_arrow(rec, "world/vehicle/vectors/w", context.data_context.Xt.w.data, 50.0, rerun::Color(255, 80, 200), "w");

        if (estimation_bool) {
            stream_body_arrow(rec, "world/estimated_vehicle/vectors/v", context.data_context.Zt.v.data, 1.0, rerun::Color(0, 180, 255), "v");
            stream_body_arrow(rec, "world/estimated_vehicle/vectors/w", context.data_context.Zt.w.data, 50.0, rerun::Color(255, 80, 200), "w");
        }

        stream_body_arrow(rec, "world/vehicle/vectors/F_net", context.data_context.WB_net.F.data, 0.01, rerun::Color(255, 80, 80), "F_net");
        stream_body_arrow(rec, "world/vehicle/vectors/F_aero", context.data_context.WB_aero.F.data, 0.01, rerun::Color(255, 160, 40), "F_aero");
        stream_body_arrow(rec, "world/vehicle/vectors/F_propulsive", context.data_context.WB_propulsive.F.data, 0.001, rerun::Color(80, 220, 120), "F_propulsive");

        stream_body_arrow(rec, "world/vehicle/vectors/M_net", context.data_context.WB_net.M.data, 0.01, rerun::Color(210, 120, 255), "M_net");
        stream_body_arrow(rec, "world/vehicle/vectors/M_aero", context.data_context.WB_aero.M.data, 0.01, rerun::Color(255, 220, 80), "M_aero");
        stream_body_arrow(rec, "world/vehicle/vectors/M_propulsive", context.data_context.WB_propulsive.M.data, 0.01, rerun::Color(120, 255, 210), "M_propulsive");

        dynamics::EulerAngles eul_t;
        eul_t.set(context.data_context.Xt.q);
        stream_vector(rec, "state/p", context.data_context.Xt.p.data, xyz_labels);
        stream_vector(rec, "state/eul", eul_t.data, eul_labels);
        stream_vector(rec, "state/w", context.data_context.Xt.w.data, angular_rate_labels);
        stream_vector(rec, "state/v", context.data_context.Xt.v.data, velocity_labels);

        stream_vector(rec, "actuators/surface", actuators::unpack_surface_actuator_inputs(context.data_context.u_surface), surface_labels);
        stream_vector(rec, "actuators/propulsor", actuators::unpack_propulsor_actuator_inputs(context.data_context.u_propulsor), propulsor_labels);

        stream_vector(rec, "forces/net", context.data_context.WB_net.F.data, xyz_labels);
        stream_vector(rec, "moments/net", context.data_context.WB_net.M.data, xyz_labels);

        stream_vector(rec, "forces/aero", context.data_context.WB_aero.F.data, xyz_labels);
        stream_vector(rec, "moments/aero", context.data_context.WB_aero.M.data, xyz_labels);

        stream_vector(rec, "forces/propulsive", context.data_context.WB_propulsive.F.data, xyz_labels);
        stream_vector(rec, "moments/propulsive", context.data_context.WB_propulsive.M.data, xyz_labels);

        if (control_bool) {
            stream_vector(rec, "setpoint/eul", context.data_context.setpoint.eulIB.data, eul_labels);
            stream_vector(rec, "setpoint/w", context.data_context.setpoint.wB_BI.data, angular_rate_labels);
            stream_vector(rec, "setpoint/v", context.data_context.setpoint.vB_BI.data, velocity_labels);
        }

        if (avionics_bool) {
            dynamics::EulerAngles eul_meas_t;
            eul_meas_t.set(context.data_context.Yt.q);
            stream_vector(rec, "measured/p", context.data_context.Yt.p.data, xyz_labels);
            stream_vector(rec, "measured/eul", eul_meas_t.data, eul_labels);
            stream_vector(rec, "measured/w", context.data_context.Yt.w.data, angular_rate_labels);
            stream_vector(rec, "measured/v", context.data_context.Yt.v.data, velocity_labels);
        }

        if (estimation_bool) {
            dynamics::EulerAngles eul_est_t;
            eul_est_t.set(context.data_context.Zt.q);
            stream_vector(rec, "estimated/p", context.data_context.Zt.p.data, xyz_labels);
            stream_vector(rec, "estimated/eul", eul_est_t.data, eul_labels);
            stream_vector(rec, "estimated/w", context.data_context.Zt.w.data, angular_rate_labels);
            stream_vector(rec, "estimated/v", context.data_context.Zt.v.data, velocity_labels);
        }

        if (wind_bool) {
            stream_body_arrow(rec, "world/vehicle/vectors/wind", context.data_context.windB.data, 1.0, rerun::Color(180, 180, 180), "wind");
            stream_vector(rec, "wind/body", context.data_context.windB.data, xyz_labels);
        }

        if (!context.image.empty()) {
            try {
                stream_flightgear_image(rec, context.image);
            } catch (const std::exception& e) {
                spdlog::warn("RerunManager: failed to stream queued FlightGear image: {}", e.what());
            }
        }
    }

    void RerunManager::run_worker() {
        while (true) {
            RerunContext context;

            {
                std::unique_lock<std::mutex> lock(queue_mutex);
                queue_cv.wait(lock, [this] { return stop_worker || !context_queue.empty(); });

                if (stop_worker && context_queue.empty()) {
                    return;
                }

                context = std::move(context_queue.front());
                context_queue.pop_front();
            }

            stream_context(context);
        }
    }

    void RerunManager::run_camera_worker() {
        const std::chrono::duration<double> camera_period(1.0 / logging_hz);

        while (!stop_camera_worker) {
            try {
                cv::Mat image = get_flightgear_image(image_width, image_height);

                {
                    std::lock_guard<std::mutex> lock(camera_mutex);
                    latest_image = image;
                }

                std::this_thread::sleep_for(camera_period);
            } catch (const std::exception& e) {
                spdlog::warn("RerunManager: failed to capture FlightGear image: {}", e.what());
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
    }
}
