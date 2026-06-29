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
            stream_vehicle_model(rec);
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

    void RerunManager::step(int t, const DataContext& context) {
        if (!rerun_bool) {
            return;
        }

        cv::Mat camera_frame;
        {
            std::lock_guard<std::mutex> lock(camera_mutex);
            camera_frame = latest_camera_frame;
        }

        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            if (frame_queue.size() >= max_queue_size) {
                frame_queue.pop_front();
            }
            frame_queue.push_back(RerunFrame{.t=t, .context=context, .camera_frame=camera_frame});
        }

        queue_cv.notify_one();
    }

    void RerunManager::stream_frame(const RerunFrame& frame) {
        rec.set_time_sequence("step", frame.t);
        rec.set_time_duration_secs("sim_time", frame.t * constants::dt);

        stream_vehicle_transform(rec, frame.context.Xt);

        trajectory.emplace_back(
            static_cast<float>(frame.context.Xt.p.data.x()),
            static_cast<float>(frame.context.Xt.p.data.y()),
            static_cast<float>(frame.context.Xt.p.data.z())
        );
        
        if (trajectory.size() > max_traj_size) {
            trajectory.erase(trajectory.begin());
        }
        stream_vehicle_trajectory(rec, trajectory);

        stream_body_arrow(rec, "world/vehicle/vectors/velocity", frame.context.Xt.v.data, 1.0, rerun::Color(0, 180, 255), "velocity");

        stream_body_arrow(rec, "world/vehicle/vectors/force_net", frame.context.WB_net.F.data, 0.01, rerun::Color(255, 80, 80), "force_net");
        stream_body_arrow(rec, "world/vehicle/vectors/force_aero", frame.context.WB_aero.F.data, 0.01, rerun::Color(255, 160, 40), "force_aero");
        stream_body_arrow(rec, "world/vehicle/vectors/force_propulsive", frame.context.WB_propulsive.F.data, 0.001, rerun::Color(80, 220, 120), "force_prop");

        stream_body_arrow(rec, "world/vehicle/vectors/moment_net", frame.context.WB_net.M.data, 0.01, rerun::Color(210, 120, 255), "moment_net");
        stream_body_arrow(rec, "world/vehicle/vectors/moment_aero", frame.context.WB_aero.M.data, 0.01, rerun::Color(255, 220, 80), "moment_aero");
        stream_body_arrow(rec, "world/vehicle/vectors/moment_propulsive", frame.context.WB_propulsive.M.data, 0.01, rerun::Color(120, 255, 210), "moment_prop");

        dynamics::EulerAngles eul_t;
        eul_t.set(frame.context.Xt.q);
        stream_vector(rec, "state/p", frame.context.Xt.p.data, xyz_labels);
        stream_vector(rec, "state/eul", eul_t.data, eul_labels);
        stream_vector(rec, "state/w", frame.context.Xt.w.data, angular_rate_labels);
        stream_vector(rec, "state/v", frame.context.Xt.v.data, velocity_labels);

        stream_vector(rec, "actuators/surface", actuators::unpack_surface_actuator_inputs(frame.context.u_surface), surface_labels);
        stream_vector(rec, "actuators/propulsor", actuators::unpack_propulsor_actuator_inputs(frame.context.u_propulsor), propulsor_labels);

        stream_vector(rec, "forces/net", frame.context.WB_net.F.data, xyz_labels);
        stream_vector(rec, "moments/net", frame.context.WB_net.M.data, xyz_labels);

        stream_vector(rec, "forces/aero", frame.context.WB_aero.F.data, xyz_labels);
        stream_vector(rec, "moments/aero", frame.context.WB_aero.M.data, xyz_labels);

        stream_vector(rec, "forces/propulsive", frame.context.WB_propulsive.F.data, xyz_labels);
        stream_vector(rec, "moments/propulsive", frame.context.WB_propulsive.M.data, xyz_labels);

        if (control_bool) {
            stream_vector(rec, "setpoint/eul", frame.context.setpoint.eulIB.data, eul_labels);
            stream_vector(rec, "setpoint/w", frame.context.setpoint.wB_BI.data, angular_rate_labels);
            stream_vector(rec, "setpoint/v", frame.context.setpoint.vB_BI.data, velocity_labels);
        }

        if (avionics_bool) {
            dynamics::EulerAngles eul_meas_t;
            eul_meas_t.set(frame.context.Yt.q);
            stream_vector(rec, "measured/p", frame.context.Yt.p.data, xyz_labels);
            stream_vector(rec, "measured/eul", eul_meas_t.data, eul_labels);
            stream_vector(rec, "measured/w", frame.context.Yt.w.data, angular_rate_labels);
            stream_vector(rec, "measured/v", frame.context.Yt.v.data, velocity_labels);
        }

        if (estimation_bool) {
            dynamics::EulerAngles eul_est_t;
            eul_est_t.set(frame.context.Zt.q);
            stream_vector(rec, "estimated/p", frame.context.Zt.p.data, xyz_labels);
            stream_vector(rec, "estimated/eul", eul_est_t.data, eul_labels);
            stream_vector(rec, "estimated/w", frame.context.Zt.w.data, angular_rate_labels);
            stream_vector(rec, "estimated/v", frame.context.Zt.v.data, velocity_labels);
        }

        if (wind_bool) {
            stream_body_arrow(rec, "world/vehicle/vectors/wind", frame.context.windB.data, 1.0, rerun::Color(180, 180, 180), "wind");
            stream_vector(rec, "wind/body", frame.context.windB.data, xyz_labels);
        }

        if (!frame.camera_frame.empty()) {
            try {
                stream_flightgear_camera(rec, frame.camera_frame);
            } catch (const std::exception& e) {
                spdlog::warn("RerunManager: failed to stream queued FlightGear camera frame: {}", e.what());
            }
        }
    }

    void RerunManager::run_worker() {
        while (true) {
            RerunFrame frame;

            {
                std::unique_lock<std::mutex> lock(queue_mutex);
                queue_cv.wait(lock, [this] { return stop_worker || !frame_queue.empty(); });

                if (stop_worker && frame_queue.empty()) {
                    return;
                }

                frame = std::move(frame_queue.front());
                frame_queue.pop_front();
            }

            stream_frame(frame);
        }
    }

    void RerunManager::run_camera_worker() {
        const std::chrono::duration<double> camera_period(1.0 / logging_hz);

        while (!stop_camera_worker) {
            try {
                cv::Mat camera_frame = get_flightgear_camera(img_width, img_height);

                {
                    std::lock_guard<std::mutex> lock(camera_mutex);
                    latest_camera_frame = camera_frame;
                }

                std::this_thread::sleep_for(camera_period);
            } catch (const std::exception& e) {
                spdlog::warn("RerunManager: failed to capture FlightGear camera frame: {}", e.what());
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
    }
}
