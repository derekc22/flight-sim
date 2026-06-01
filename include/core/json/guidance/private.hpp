#pragma once
#include <string>
#include <nlohmann/json.hpp>
#include "core/json/guidance/public.hpp"
#include "simulation/control/public.hpp"
#include "simulation/guidance/public.hpp"

namespace json {

    void validate_attitude_control_setpoint(const nlohmann::json& guidance_json, const control::ControllerType& controller_type);
    guidance::TrajectoryComponents& parse_attitude_control_setpoint(const nlohmann::json& guidance_json, const control::ControllerType& controller_type, guidance::TrajectoryComponents& traj_components);
    void validate_velocity_control_setpoint(const nlohmann::json& guidance_json, const control::ControllerType& controller_type);
    guidance::TrajectoryComponents& parse_velocity_control_setpoint(const nlohmann::json& guidance_json, const control::ControllerType& controller_type, guidance::TrajectoryComponents& traj_components);
    void validate_linear_quadratic_control_setpoint(const nlohmann::json& guidance_json, const control::ControllerType& controller_type);
    guidance::TrajectoryComponents& parse_linear_quadratic_control_setpoint(const nlohmann::json& guidance_json, const control::ControllerType& controller_type, guidance::TrajectoryComponents& traj_components);
    void validate_nonlinear_control_setpoint(const nlohmann::json& guidance_json, const control::ControllerType& controller_type);
    guidance::TrajectoryComponents& parse_nonlinear_control_setpoint(const nlohmann::json& guidance_json, const control::ControllerType& controller_type, guidance::TrajectoryComponents& traj_components);
    void validate_guidance(const nlohmann::json& guidance_json);
    void validate_trajectory_type(const guidance::TrajectoryComponents& traj_components, guidance::TrajectoryType traj_type);
    guidance::TrajectoryType map_trajectory_type(const std::string& trajectory_type_str);
    void fill_missing_trajectory_components(guidance::TrajectoryComponents& traj_components);
    void validate_trajectory_components(guidance::TrajectoryComponents& traj_components);
    guidance::Trajectory merge_trajectory_components(const guidance::TrajectoryComponents& traj_components);
    guidance::GuidanceProperties parse_guidance_properties(const nlohmann::json& config, const control::ControlProperties& control_properties);
}
