#include <stdexcept>
#include <string>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "core/json/guidance/private/parsing.hpp"
#include "core/json/guidance/private/validation.hpp"
#include "core/json/public/data/helpers.hpp"
#include "simulation/control/public/manager.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/guidance/public/data/helpers.hpp"
#include "simulation/guidance/public/data/types.hpp"
#include "simulation/guidance/public/manager.hpp"

namespace json {

    guidance::TrajectoryComponents& parse_attitude_control_setpoint(const nlohmann::json& guidance_json, const control::ControllerType& controller_type, guidance::TrajectoryComponents& traj_components) {
        validate_attitude_control_setpoint(guidance_json, controller_type);

        if (controller_type == control::ControllerType::DamperPID) {
            traj_components.n_rows = 1;
        }

        if (controller_type == control::ControllerType::AttitudePID) {
            traj_components.w_traj = parse_MatrixXd(guidance_json.at("w"));
            traj_components.eul_traj = parse_MatrixXd(guidance_json.at("eul"));
            traj_components.n_rows = traj_components.w_traj.rows();
        }
        return traj_components;
    }

    guidance::TrajectoryComponents& parse_velocity_control_setpoint(const nlohmann::json& guidance_json, const control::ControllerType& controller_type, guidance::TrajectoryComponents& traj_components) {
        validate_velocity_control_setpoint(guidance_json, controller_type);

        if (controller_type == control::ControllerType::VelocityPID) {
            traj_components.v_traj = parse_MatrixXd(guidance_json.at("v"));
            traj_components.n_rows = traj_components.v_traj.rows();
        }
        return traj_components;
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

    guidance::GuidanceManager parse_guidance_manager(const nlohmann::json& config, const control::ControlManager& control_manager) {
        validate_guidance(config);

        guidance::GuidanceManager guidance_manager;
        guidance::TrajectoryComponents traj_components;

        if (config.contains("attitude")) {
            const auto& attitude_guidance_json = config.at("attitude");
            const control::ControllerType controller_type = control_manager.attitude_control.has_value() ? control_manager.attitude_control.value().controller_type : control::ControllerType::None;
            traj_components = parse_attitude_control_setpoint(attitude_guidance_json, controller_type, traj_components);
        }

        if (config.contains("velocity")) {
            const auto& velocity_guidance_json = config.at("velocity");
            const control::ControllerType controller_type = control_manager.velocity_control.has_value() ? control_manager.velocity_control.value().controller_type : control::ControllerType::None;
            traj_components = parse_velocity_control_setpoint(velocity_guidance_json, controller_type, traj_components);
        }

        if (config.contains("linear_quadratic")) {
            const auto& linear_quadratic_guidance_json = config.at("linear_quadratic");
            const control::ControllerType controller_type = control_manager.full_state_control.has_value() ? control_manager.full_state_control.value().controller_type : control::ControllerType::None;
            traj_components = parse_linear_quadratic_control_setpoint(linear_quadratic_guidance_json, controller_type, traj_components);
        }

        if (config.contains("nonlinear")) {
            const auto& nonlinear_guidance_json = config.at("nonlinear");
            const control::ControllerType controller_type = control_manager.full_state_control.has_value() ? control_manager.full_state_control.value().controller_type : control::ControllerType::None;
            traj_components = parse_nonlinear_control_setpoint(nonlinear_guidance_json, controller_type, traj_components);
        }

        fill_missing_trajectory_components(traj_components);
        validate_trajectory_components(traj_components);

        guidance::TrajectoryType traj_type = map_trajectory_type( config.at("trajectory_type").get<std::string>() );
        validate_trajectory_type(traj_components, traj_type);

        guidance::Trajectory trajectory = merge_trajectory_components(traj_components);

        switch (traj_type) {
            case guidance::TrajectoryType::Regulation:
                guidance_manager.regulation = guidance::RegulationGuidance{ .trajectory = trajectory };
                break;

            case guidance::TrajectoryType::Tracking:
                guidance_manager.tracking = guidance::TrackingGuidance{ .trajectory = trajectory };
                break;

            case guidance::TrajectoryType::Interpolated:
                guidance_manager.interpolated = guidance::InterpolatedGuidance{ .trajectory = trajectory };
                break;
        }

        return guidance_manager;
    }

}
