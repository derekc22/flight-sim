#include <cstdlib>
#include <opencv2/opencv.hpp>
#include "core/io/camera/public.hpp"
#include "core/io/rerun/public.hpp"
#include "core/io/rerun/private.hpp"
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"

namespace io {

    RerunManager::RerunManager(bool rerun_bool, bool control_bool, bool sensor_bool, bool estimation_bool, bool wind_bool)
        : rerun_bool(rerun_bool), control_bool(control_bool), sensor_bool(sensor_bool), estimation_bool(estimation_bool), wind_bool(wind_bool), rec("flight_sim")
    {
        if (rerun_bool) {
            auto server_uri = rec.serve_grpc("0.0.0.0", 9876, "1GiB").value;
            std::system("rerun assets/default.rbl --connect rerun+http://127.0.0.1:9876/proxy &");

            rec.log_static("/", rerun::ViewCoordinates::FRD);
            stream_vehicle_model(rec);
        }
    }

    void RerunManager::step(int t, const DataContext& context) {
        if (!rerun_bool) {
            return;
        }

        rec.set_time_sequence("step", t);
        rec.set_time_duration_secs("sim_time", t * constants::dt);

        stream_vehicle_transform(rec, context.Xt);

        trajectory.emplace_back(
            static_cast<float>(context.Xt.p.data.x()),
            static_cast<float>(context.Xt.p.data.y()),
            static_cast<float>(context.Xt.p.data.z())
        );
        
        const std::size_t max_traj_points = static_cast<std::size_t>(60.0/constants::dt); // 60 seconds
        if (trajectory.size() > max_traj_points) {
            trajectory.erase(trajectory.begin());
        }
        stream_vehicle_trajectory(rec, trajectory);

        stream_body_arrow(rec, "world/vehicle/vectors/velocity", context.Xt.v.data, 1.0, rerun::Color(0, 180, 255), "velocity");

        stream_body_arrow(rec, "world/vehicle/vectors/force_net", context.WB_net.F.data, 0.01, rerun::Color(255, 80, 80), "force_net");
        stream_body_arrow(rec, "world/vehicle/vectors/force_aero", context.WB_aero.F.data, 0.01, rerun::Color(255, 160, 40), "force_aero");
        stream_body_arrow(rec, "world/vehicle/vectors/force_propulsive", context.WB_propulsive.F.data, 0.001, rerun::Color(80, 220, 120), "force_prop");

        stream_body_arrow(rec, "world/vehicle/vectors/moment_net", context.WB_net.M.data, 0.01, rerun::Color(210, 120, 255), "moment_net");
        stream_body_arrow(rec, "world/vehicle/vectors/moment_aero", context.WB_aero.M.data, 0.01, rerun::Color(255, 220, 80), "moment_aero");
        stream_body_arrow(rec, "world/vehicle/vectors/moment_propulsive", context.WB_propulsive.M.data, 0.01, rerun::Color(120, 255, 210), "moment_prop");

        dynamics::EulerAngles eul_t;
        eul_t.set(context.Xt.q);
        stream_vector(rec, "state/p", context.Xt.p.data, xyz_labels);
        stream_vector(rec, "state/eul", eul_t.data, eul_labels);
        stream_vector(rec, "state/w", context.Xt.w.data, angular_rate_labels);
        stream_vector(rec, "state/v", context.Xt.v.data, velocity_labels);

        stream_vector(rec, "actuators/surface", actuators::unpack_surface_actuator_inputs(context.u_surface), surface_labels);
        stream_vector(rec, "actuators/propulsor", actuators::unpack_propulsor_actuator_inputs(context.u_propulsor), propulsor_labels);

        stream_vector(rec, "forces/net", context.WB_net.F.data, xyz_labels);
        stream_vector(rec, "moments/net", context.WB_net.M.data, xyz_labels);

        stream_vector(rec, "forces/aero", context.WB_aero.F.data, xyz_labels);
        stream_vector(rec, "moments/aero", context.WB_aero.M.data, xyz_labels);

        stream_vector(rec, "forces/propulsive", context.WB_propulsive.F.data, xyz_labels);
        stream_vector(rec, "moments/propulsive", context.WB_propulsive.M.data, xyz_labels);

        if (control_bool) {
            stream_vector(rec, "setpoint/eul", context.setpoint.eulIB.data, eul_labels);
            stream_vector(rec, "setpoint/w", context.setpoint.wB_BI.data, angular_rate_labels);
            stream_vector(rec, "setpoint/v", context.setpoint.vB_BI.data, velocity_labels);
        }

        if (sensor_bool) {
            dynamics::EulerAngles eul_meas_t;
            eul_meas_t.set(context.Yt.q);
            stream_vector(rec, "measured/p", context.Yt.p.data, xyz_labels);
            stream_vector(rec, "measured/eul", eul_meas_t.data, eul_labels);
            stream_vector(rec, "measured/w", context.Yt.w.data, angular_rate_labels);
            stream_vector(rec, "measured/v", context.Yt.v.data, velocity_labels);
        }

        if (estimation_bool) {
            dynamics::EulerAngles eul_est_t;
            eul_est_t.set(context.Zt.q);
            stream_vector(rec, "estimated/p", context.Zt.p.data, xyz_labels);
            stream_vector(rec, "estimated/eul", eul_est_t.data, eul_labels);
            stream_vector(rec, "estimated/w", context.Zt.w.data, angular_rate_labels);
            stream_vector(rec, "estimated/v", context.Zt.v.data, velocity_labels);
        }

        if (wind_bool) {
            stream_body_arrow(rec, "world/vehicle/vectors/wind", context.windB.data, 1.0, rerun::Color(180, 180, 180), "wind");
            stream_vector(rec, "wind/body", context.windB.data, xyz_labels);
        }

        bool video_bool =  true;
        if (video_bool && t % 100 == 0) {
            cv::Mat frame = get_flightgear_camera();
            stream_flightgear_camera(rec, frame);
        }
    }
}
