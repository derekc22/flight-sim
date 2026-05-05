#include <stdexcept>
#include <string>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "core/json/control.hpp"
#include "core/json/json.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/guidance/guidance.hpp"
#include "simulation/util/validate.hpp"

namespace json {

    void validate_axial_control_setpoint(const nlohmann::json& guidance_json, const control::ControlType& control_type) {
        if (control_type == control::ControlType::DamperPID) {
            if (guidance_json.contains("v")) { throw std::runtime_error("json::validate_axial_control_setpoint: DamperPID does not require v"); }
            if (guidance_json.contains("w")) { throw std::runtime_error("json::validate_axial_control_setpoint: DamperPID does not require w"); }
            if (guidance_json.contains("eul")) { throw std::runtime_error("json::validate_axial_control_setpoint: DamperPID does not require eul"); }
        }

        if (control_type == control::ControlType::AxialPID) {
            if (guidance_json.contains("v")) { throw std::runtime_error("json::validate_axial_control_setpoint: AxialPID does not require v"); }
            if (!guidance_json.contains("w")) { throw std::runtime_error("json::validate_axial_control_setpoint: AxialPID requires w"); }
            if (!guidance_json.contains("eul")) { throw std::runtime_error("json::validate_axial_control_setpoint: AxialPID requires eul"); }
        }
    }

    guidance::TrajectoryComponents& parse_axial_control_setpoint(
        const nlohmann::json& guidance_json,
        const control::ControlType& control_type,
        guidance::TrajectoryComponents& traj_components
    ) {
        validate_axial_control_setpoint(guidance_json, control_type);

        if (control_type == control::ControlType::DamperPID) {
            traj_components.n_rows = 1;
        }

        if (control_type == control::ControlType::AxialPID) {
            traj_components.w_mat = parse_MatrixXd(guidance_json.at("w"));
            traj_components.eul_mat = parse_MatrixXd(guidance_json.at("eul"));
            traj_components.n_rows = traj_components.w_mat.rows();
        }
        return traj_components;
    }

    void validate_velocity_control_setpoint(const nlohmann::json& guidance_json, const control::ControlType& control_type) {
        if (control_type == control::ControlType::VelocityPID) {
            if (!guidance_json.contains("v")) { throw std::runtime_error("json::validate_velocity_control_setpoint: VelocityPID requires v"); }
            if (guidance_json.contains("w")) { throw std::runtime_error("json::validate_velocity_control_setpoint: VelocityPID does not require w"); }
            if (guidance_json.contains("eul")) { throw std::runtime_error("json::validate_velocity_control_setpoint: VelocityPID does not require eul"); }
        }
    }

    guidance::TrajectoryComponents& parse_velocity_control_setpoint(
        const nlohmann::json& guidance_json,
        const control::ControlType& control_type,
        guidance::TrajectoryComponents& traj_components
    ) {
        validate_velocity_control_setpoint(guidance_json, control_type);

        if (control_type == control::ControlType::VelocityPID) {
            traj_components.v_mat = parse_MatrixXd(guidance_json.at("v"));
            traj_components.n_rows = traj_components.v_mat.rows();
        }
        return traj_components;
    }

    void validate_linear_full_state_feedback_control_setpoint(const nlohmann::json& guidance_json, const control::ControlType& control_type) {
        if (
            control_type == control::ControlType::LinearQuadraticRegulator ||
            control_type == control::ControlType::LinearQuadraticTracker ||
            control_type == control::ControlType::LinearQuadraticIntegrator
        ) {
            if (!guidance_json.contains("v")) { throw std::runtime_error("json::validate_linear_full_state_feedback_control_setpoint: linear_full_state_feedback requires v"); }
            if (!guidance_json.contains("w")) { throw std::runtime_error("json::validate_linear_full_state_feedback_control_setpoint: linear_full_state_feedback requires w"); }
            if (!guidance_json.contains("eul")) { throw std::runtime_error("json::validate_linear_full_state_feedback_control_setpoint: linear_full_state_feedback requires eul"); }
        }
    }

    guidance::TrajectoryComponents& parse_linear_full_state_feedback_control_setpoint(
        const nlohmann::json& guidance_json,
        const control::ControlType& control_type,
        guidance::TrajectoryComponents& traj_components
    ) {
        validate_linear_full_state_feedback_control_setpoint(guidance_json, control_type);

        if (
            control_type == control::ControlType::LinearQuadraticRegulator ||
            control_type == control::ControlType::LinearQuadraticTracker ||
            control_type == control::ControlType::LinearQuadraticIntegrator
        ) {
            traj_components.v_mat = parse_MatrixXd(guidance_json.at("v"));
            traj_components.w_mat = parse_MatrixXd(guidance_json.at("w"));
            traj_components.eul_mat = parse_MatrixXd(guidance_json.at("eul"));
            traj_components.n_rows = traj_components.v_mat.rows();
        }
        return traj_components;
    }

    void validate_nonlinear_control_setpoint(const nlohmann::json& guidance_json, const control::ControlType& control_type) {
        if (
            control_type == control::ControlType::FeedbackLinearization ||
            control_type == control::ControlType::NonlinearDynamicInversion ||
            control_type == control::ControlType::IncrementalNonlinearDynamicInversion
        ) {
            if (!guidance_json.contains("v")) { throw std::runtime_error("json::validate_nonlinear_control_setpoint: nonlinear requires v"); }
            if (!guidance_json.contains("w")) { throw std::runtime_error("json::validate_nonlinear_control_setpoint: nonlinear requires w"); }
            if (!guidance_json.contains("eul")) { throw std::runtime_error("json::validate_nonlinear_control_setpoint: nonlinear requires eul"); }
        }
    }

    guidance::TrajectoryComponents& parse_nonlinear_control_setpoint(
        const nlohmann::json& guidance_json,
        const control::ControlType& control_type,
        guidance::TrajectoryComponents& traj_components
    ) {
        validate_nonlinear_control_setpoint(guidance_json, control_type);

        if (
            control_type == control::ControlType::FeedbackLinearization ||
            control_type == control::ControlType::NonlinearDynamicInversion ||
            control_type == control::ControlType::IncrementalNonlinearDynamicInversion
        ) {
            traj_components.v_mat = parse_MatrixXd(guidance_json.at("v"));
            traj_components.w_mat = parse_MatrixXd(guidance_json.at("w"));
            traj_components.eul_mat = parse_MatrixXd(guidance_json.at("eul"));
            traj_components.n_rows = traj_components.v_mat.rows();
        }
        return traj_components;
    }

    void validate_guidance(const nlohmann::json& guidance_json){
        bool axial_bool = guidance_json.contains("axial");
        bool velocity_bool = guidance_json.contains("velocity");
        bool linear_full_state_feedback_bool = guidance_json.contains("linear_full_state_feedback");
        bool nonlinear_bool = guidance_json.contains("nonlinear");

        if (axial_bool && linear_full_state_feedback_bool) { throw std::runtime_error("json::validate_guidance: axial and linear_full_state_feedback guidance cannot both be present"); }
        if (axial_bool && nonlinear_bool) { throw std::runtime_error("json::validate_guidance: axial and nonlinear guidance cannot both be present"); }

        if (linear_full_state_feedback_bool && nonlinear_bool) { throw std::runtime_error("json::validate_guidance: linear_full_state_feedback and nonlinear guidance cannot both be present"); }

        if (velocity_bool && linear_full_state_feedback_bool) { throw std::runtime_error("json::validate_guidance: velocity and linear_full_state_feedback guidance cannot both be present"); }
        if (velocity_bool && nonlinear_bool) { throw std::runtime_error("json::validate_guidance: velocity and nonlinear guidance cannot both be present"); }
    }

    void validate_trajectory_type(const guidance::TrajectoryComponents& traj_components, guidance::TrajectoryType traj_type) {
        if (traj_type == guidance::TrajectoryType::Stationary && traj_components.n_rows > 1) { throw std::runtime_error("json::validate_trajectory_type: stationary trajectory cannot have more than one row"); }
        if (traj_type == guidance::TrajectoryType::Interpolated && traj_components.n_rows != 2) { throw std::runtime_error("json::validate_trajectory_type: interpolated trajectory requires two rows"); }
    }

    guidance::TrajectoryType map_trajectory_type(const std::string& trajectory_type_str) {
        if (trajectory_type_str == "stationary") { return guidance::TrajectoryType::Stationary; }
        if (trajectory_type_str == "prespecified") { return guidance::TrajectoryType::Prespecified; }
        if (trajectory_type_str == "interpolated") { return guidance::TrajectoryType::Interpolated; }
        throw std::runtime_error("json::map_trajectory_type unknown trajectory type: " + trajectory_type_str);
    }

    void fill_missing_trajectory_components(guidance::TrajectoryComponents& traj_components) {
        if (traj_components.n_rows <= 0) { throw std::runtime_error("json::fill_missing_trajectory_components: trajectory must have at least one row"); }
        if (traj_components.v_mat.size() == 0) { traj_components.v_mat = Eigen::MatrixXd::Zero(traj_components.n_rows, 3); }
        if (traj_components.w_mat.size() == 0) { traj_components.w_mat = Eigen::MatrixXd::Zero(traj_components.n_rows, 3); }
        if (traj_components.eul_mat.size() == 0) { traj_components.eul_mat = Eigen::MatrixXd::Zero(traj_components.n_rows, 3); }
    }

    void validate_trajectory_components(guidance::TrajectoryComponents& traj_components) {
        int n_cols = 3;  // v, w, and eul are 3-column vectors
        int n_rows = traj_components.n_rows;
        if (n_rows <= 0) { throw std::runtime_error("json::validate_trajectory_components: trajectory must have at least one row"); }

        std::string context = "guidance::validate_trajectory_components";
        util::validate_shape(traj_components.v_mat, n_rows, n_cols, context, "v_mat");
        util::validate_shape(traj_components.w_mat, n_rows, n_cols, context, "w_mat");
        util::validate_shape(traj_components.eul_mat, n_rows, n_cols, context, "eul_mat");

        traj_components.n_rows = n_rows;
    }

    guidance::Trajectory merge_trajectory_components(const guidance::TrajectoryComponents& traj_components) {
        guidance::Trajectory traj{ Eigen::MatrixXd::Zero(traj_components.n_rows, guidance::guidance_state_dim) };

        for (int i = 0; i < traj_components.n_rows; ++i) {
            dynamics::RigidBodyState rbs_temp;
            rbs_temp.v = dynamics::LinearVelocity{ traj_components.v_mat.row(i).transpose() };
            rbs_temp.w = dynamics::AngularVelocity{ traj_components.w_mat.row(i).transpose() };

            dynamics::OrientationQuaternion qIB;
            qIB.set(dynamics::EulerAngles{ traj_components.eul_mat.row(i).transpose() });
            rbs_temp.q = qIB;

            guidance::GuidanceStateVector traj_row_merged = guidance::unpack_rigid_body_state(rbs_temp);
            traj.data.row(i) = traj_row_merged.transpose();
        }
        return traj;
    }

    guidance::GuidanceProperties parse_guidance_properties(const nlohmann::json& config, const control::ControlProperties& control_properties) {
        validate_guidance(config);

        guidance::GuidanceProperties guidance_properties;
        guidance::TrajectoryComponents traj_components;

        if (config.contains("axial")) {
            const auto& axial_guidance_json = config.at("axial");
            traj_components = parse_axial_control_setpoint(
                axial_guidance_json,
                control_properties.axial_control_type,
                traj_components
            );
        }

        if (config.contains("velocity")) {
            const auto& velocity_guidance_json = config.at("velocity");
            traj_components = parse_velocity_control_setpoint(
                velocity_guidance_json,
                control_properties.velocity_control_type,
                traj_components
            );
        }

        if (config.contains("linear_full_state_feedback")) {
            const auto& linear_full_state_feedback_guidance_json = config.at("linear_full_state_feedback");
            traj_components = parse_linear_full_state_feedback_control_setpoint(
                linear_full_state_feedback_guidance_json,
                control_properties.linear_full_state_feedback_control_type,
                traj_components
            );
        }

        if (config.contains("nonlinear")) {
            const auto& nonlinear_guidance_json = config.at("nonlinear");
            traj_components = parse_nonlinear_control_setpoint(
                nonlinear_guidance_json,
                control_properties.nonlinear_control_type,
                traj_components
            );
        }

        fill_missing_trajectory_components(traj_components);
        validate_trajectory_components(traj_components);

        guidance::TrajectoryType traj_type = map_trajectory_type( config.at("trajectory_type").get<std::string>() );
        validate_trajectory_type(traj_components, traj_type);

        guidance_properties.trajectory_type = traj_type;
        guidance_properties.trajectory = merge_trajectory_components(traj_components);

        return guidance_properties;
    }


    guidance::GuidanceProperties parse_guidance_config(const control::ControlProperties& control_properties) {
        const auto config_path = resolve_run_config_entry_path("guidance_config");
        const auto config = read_json_file(config_path);
        return parse_guidance_properties(config, control_properties);
    }

}
