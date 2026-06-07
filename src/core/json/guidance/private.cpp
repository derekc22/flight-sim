#include <stdexcept>
#include <string>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "core/json/guidance/private.hpp"
#include "core/json/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/control/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/guidance/public.hpp"
#include "simulation/util/validate/public.hpp"

namespace json {

    void validate_attitude_control_setpoint(const nlohmann::json& guidance_json, const control::ControllerType& controller_type) {
        if (controller_type == control::ControllerType::DamperPID) {
            if (guidance_json.contains("v")) { 
                throw std::runtime_error("json::validate_attitude_control_setpoint: DamperPID does not require v"); 
            }
            if (guidance_json.contains("w")) { 
                throw std::runtime_error("json::validate_attitude_control_setpoint: DamperPID does not require w"); 
            }
            if (guidance_json.contains("eul")) { 
                throw std::runtime_error("json::validate_attitude_control_setpoint: DamperPID does not require eul"); 
            }
        }

        if (controller_type == control::ControllerType::AxialPID) {
            if (guidance_json.contains("v")) { 
                throw std::runtime_error("json::validate_attitude_control_setpoint: AxialPID does not require v"); 
            }
            if (!guidance_json.contains("w")) { 
                throw std::runtime_error("json::validate_attitude_control_setpoint: AxialPID requires w"); 
            }
            if (!guidance_json.contains("eul")) { 
                throw std::runtime_error("json::validate_attitude_control_setpoint: AxialPID requires eul"); 
            }
        }
    }

    guidance::TrajectoryComponents& parse_attitude_control_setpoint(const nlohmann::json& guidance_json, const control::ControllerType& controller_type, guidance::TrajectoryComponents& traj_components) {
        validate_attitude_control_setpoint(guidance_json, controller_type);

        if (controller_type == control::ControllerType::DamperPID) {
            traj_components.n_rows = 1;
        }

        if (controller_type == control::ControllerType::AxialPID) {
            traj_components.w_traj = parse_MatrixXd(guidance_json.at("w"));
            traj_components.eul_traj = parse_MatrixXd(guidance_json.at("eul"));
            traj_components.n_rows = traj_components.w_traj.rows();
        }
        return traj_components;
    }

    void validate_velocity_control_setpoint(const nlohmann::json& guidance_json, const control::ControllerType& controller_type) {
        if (controller_type == control::ControllerType::VelocityPID) {
            if (!guidance_json.contains("v")) { 
                throw std::runtime_error("json::validate_velocity_control_setpoint: VelocityPID requires v"); 
            }
            if (guidance_json.contains("w")) { 
                throw std::runtime_error("json::validate_velocity_control_setpoint: VelocityPID does not require w"); 
            }
            if (guidance_json.contains("eul")) { 
                throw std::runtime_error("json::validate_velocity_control_setpoint: VelocityPID does not require eul"); 
            }
        }
    }

    guidance::TrajectoryComponents& parse_velocity_control_setpoint(const nlohmann::json& guidance_json, const control::ControllerType& controller_type, guidance::TrajectoryComponents& traj_components) {
        validate_velocity_control_setpoint(guidance_json, controller_type);

        if (controller_type == control::ControllerType::VelocityPID) {
            traj_components.v_traj = parse_MatrixXd(guidance_json.at("v"));
            traj_components.n_rows = traj_components.v_traj.rows();
        }
        return traj_components;
    }

    void validate_linear_quadratic_control_setpoint(const nlohmann::json& guidance_json, const control::ControllerType& controller_type) {
        if (
            controller_type == control::ControllerType::LinearQuadraticRegulator ||
            controller_type == control::ControllerType::LinearQuadraticTracker ||
            controller_type == control::ControllerType::LinearQuadraticIntegrator
        ) {
            if (!guidance_json.contains("v")) { 
                throw std::runtime_error("json::validate_linear_quadratic_control_setpoint: linear_quadratic requires v"); 
            }
            if (!guidance_json.contains("w")) { 
                throw std::runtime_error("json::validate_linear_quadratic_control_setpoint: linear_quadratic requires w"); 
            }
            if (!guidance_json.contains("eul")) { 
                throw std::runtime_error("json::validate_linear_quadratic_control_setpoint: linear_quadratic requires eul"); 
            }
        }
    }

    guidance::TrajectoryComponents& parse_linear_quadratic_control_setpoint(const nlohmann::json& guidance_json, const control::ControllerType& controller_type, guidance::TrajectoryComponents& traj_components) {
        validate_linear_quadratic_control_setpoint(guidance_json, controller_type);

        if (
            controller_type == control::ControllerType::LinearQuadraticRegulator ||
            controller_type == control::ControllerType::LinearQuadraticTracker ||
            controller_type == control::ControllerType::LinearQuadraticIntegrator
        ) {
            traj_components.v_traj = parse_MatrixXd(guidance_json.at("v"));
            traj_components.w_traj = parse_MatrixXd(guidance_json.at("w"));
            traj_components.eul_traj = parse_MatrixXd(guidance_json.at("eul"));
            traj_components.n_rows = traj_components.v_traj.rows();
        }
        return traj_components;
    }

    void validate_nonlinear_control_setpoint(const nlohmann::json& guidance_json, const control::ControllerType& controller_type) {
        if (
            controller_type == control::ControllerType::FeedbackLinearization ||
            controller_type == control::ControllerType::NonlinearDynamicInversion ||
            controller_type == control::ControllerType::IncrementalNonlinearDynamicInversion
        ) {
            if (!guidance_json.contains("v")) { 
                throw std::runtime_error("json::validate_nonlinear_control_setpoint: nonlinear requires v"); 
            }
            if (!guidance_json.contains("w")) { 
                throw std::runtime_error("json::validate_nonlinear_control_setpoint: nonlinear requires w"); 
            }
            if (!guidance_json.contains("eul")) { 
                throw std::runtime_error("json::validate_nonlinear_control_setpoint: nonlinear requires eul"); 
            }
        }
    }

    guidance::TrajectoryComponents& parse_nonlinear_control_setpoint(const nlohmann::json& guidance_json, const control::ControllerType& controller_type, guidance::TrajectoryComponents& traj_components) {
        validate_nonlinear_control_setpoint(guidance_json, controller_type);

        if (
            controller_type == control::ControllerType::FeedbackLinearization ||
            controller_type == control::ControllerType::NonlinearDynamicInversion ||
            controller_type == control::ControllerType::IncrementalNonlinearDynamicInversion
        ) {
            traj_components.v_traj = parse_MatrixXd(guidance_json.at("v"));
            traj_components.w_traj = parse_MatrixXd(guidance_json.at("w"));
            traj_components.eul_traj = parse_MatrixXd(guidance_json.at("eul"));
            traj_components.n_rows = traj_components.v_traj.rows();
        }
        return traj_components;
    }

    void validate_guidance(const nlohmann::json& guidance_json) {
        bool attitude_bool = guidance_json.contains("attitude");
        bool velocity_bool = guidance_json.contains("velocity");
        bool linear_quadratic_bool = guidance_json.contains("linear_quadratic");
        bool nonlinear_bool = guidance_json.contains("nonlinear");

        if (attitude_bool && linear_quadratic_bool) { 
            throw std::runtime_error("json::validate_guidance: attitude and linear_quadratic guidance cannot both be present"); 
        }
        if (attitude_bool && nonlinear_bool) { 
            throw std::runtime_error("json::validate_guidance: attitude and nonlinear guidance cannot both be present"); 
        }

        if (linear_quadratic_bool && nonlinear_bool) { 
            throw std::runtime_error("json::validate_guidance: linear_quadratic and nonlinear guidance cannot both be present"); 
        }

        if (velocity_bool && linear_quadratic_bool) { 
            throw std::runtime_error("json::validate_guidance: velocity and linear_quadratic guidance cannot both be present"); 
        }
        if (velocity_bool && nonlinear_bool) { 
            throw std::runtime_error("json::validate_guidance: velocity and nonlinear guidance cannot both be present"); 
        }
    }

    void validate_trajectory_type(const guidance::TrajectoryComponents& traj_components, guidance::TrajectoryType traj_type) {
        if (traj_type == guidance::TrajectoryType::Regulation && traj_components.n_rows > 1) { 
            throw std::runtime_error("json::validate_trajectory_type: regulation trajectory cannot have more than one row"); 
        }
        if (traj_type == guidance::TrajectoryType::Interpolated && traj_components.n_rows != 2) { 
            throw std::runtime_error("json::validate_trajectory_type: interpolated trajectory requires two rows"); 
        }
    }

    guidance::TrajectoryType map_trajectory_type(const std::string& trajectory_type_str) {
        if (trajectory_type_str == "regulation") { return guidance::TrajectoryType::Regulation; }
        if (trajectory_type_str == "tracking") { return guidance::TrajectoryType::Tracking; }
        if (trajectory_type_str == "interpolated") { return guidance::TrajectoryType::Interpolated; }
        throw std::runtime_error("json::map_trajectory_type unknown trajectory type: " + trajectory_type_str);
    }

    void fill_missing_trajectory_components(guidance::TrajectoryComponents& traj_components) {
        if (traj_components.n_rows <= 0) { 
            throw std::runtime_error(
                "json::fill_missing_trajectory_components: trajectory must have at least one row. "  
                "Possibly due to mismatched guidance and control configuration JSONs."
            ); 
        }
        if (traj_components.v_traj.size() == 0) { traj_components.v_traj = Eigen::MatrixXd::Zero(traj_components.n_rows, 3); }
        if (traj_components.w_traj.size() == 0) { traj_components.w_traj = Eigen::MatrixXd::Zero(traj_components.n_rows, 3); }
        if (traj_components.eul_traj.size() == 0) { traj_components.eul_traj = Eigen::MatrixXd::Zero(traj_components.n_rows, 3); }
    }

    void validate_trajectory_components(guidance::TrajectoryComponents& traj_components) {
        int n_cols = 3;  // v, w, and eul are 3-column vectors
        int n_rows = traj_components.n_rows;
        if (n_rows <= 0) { 
            throw std::runtime_error("json::validate_trajectory_components: trajectory must have at least one row"); 
        }

        std::string context = "guidance::validate_trajectory_components";
        util::validate_shape(traj_components.v_traj, n_rows, n_cols, context, "v_traj");
        util::validate_shape(traj_components.w_traj, n_rows, n_cols, context, "w_traj");
        util::validate_shape(traj_components.eul_traj, n_rows, n_cols, context, "eul_traj");

        traj_components.n_rows = n_rows;
    }

    guidance::Trajectory merge_trajectory_components(const guidance::TrajectoryComponents& traj_components) {
        guidance::Trajectory traj{ Eigen::MatrixXd::Zero(traj_components.n_rows, guidance::guidance_state_dim) };

        for (int i = 0; i < traj_components.n_rows; ++i) {
            dynamics::RigidBodyState X;
            X.v = dynamics::TranslationalVelocity{ traj_components.v_traj.row(i).transpose() };
            X.w = dynamics::AngularVelocity{ traj_components.w_traj.row(i).transpose() };

            dynamics::OrientationQuaternion qIB;
            qIB.set(dynamics::EulerAngles{ traj_components.eul_traj.row(i).transpose() });
            X.q = qIB;

            guidance::GuidanceSetpointVector traj_row_merged = guidance::unpack_guidance_setpoint(X);
            traj.data.row(i) = traj_row_merged.transpose();
        }
        return traj;
    }

    guidance::GuidanceProperties parse_guidance_properties(const nlohmann::json& config, const control::ControlProperties& control_properties) {
        validate_guidance(config);

        guidance::GuidanceProperties guidance_properties;
        guidance::TrajectoryComponents traj_components;

        if (config.contains("attitude")) {
            const auto& attitude_guidance_json = config.at("attitude");
            traj_components = parse_attitude_control_setpoint(attitude_guidance_json, control_properties.attitude_controller_type, traj_components);
        }

        if (config.contains("velocity")) {
            const auto& velocity_guidance_json = config.at("velocity");
            traj_components = parse_velocity_control_setpoint(velocity_guidance_json, control_properties.velocity_controller_type, traj_components);
        }

        if (config.contains("linear_quadratic")) {
            const auto& linear_quadratic_guidance_json = config.at("linear_quadratic");
            traj_components = parse_linear_quadratic_control_setpoint(linear_quadratic_guidance_json, control_properties.linear_quadratic_controller_type, traj_components);
        }

        if (config.contains("nonlinear")) {
            const auto& nonlinear_guidance_json = config.at("nonlinear");
            traj_components = parse_nonlinear_control_setpoint(nonlinear_guidance_json, control_properties.nonlinear_controller_type, traj_components);
        }

        fill_missing_trajectory_components(traj_components);
        validate_trajectory_components(traj_components);

        guidance::TrajectoryType traj_type = map_trajectory_type( config.at("trajectory_type").get<std::string>() );
        validate_trajectory_type(traj_components, traj_type);

        guidance_properties.trajectory_type = traj_type;
        guidance_properties.trajectory = merge_trajectory_components(traj_components);

        return guidance_properties;
    }

}
