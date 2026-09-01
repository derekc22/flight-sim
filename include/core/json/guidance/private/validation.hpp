#pragma once
#include <nlohmann/json.hpp>
#include "simulation/control/public/data/types.hpp"
#include "simulation/guidance/public/data/types.hpp"

namespace json {

    void validate_attitude_control_setpoint(const nlohmann::json& guidance_json, const control::ControllerType& controller_type);
    void validate_velocity_control_setpoint(const nlohmann::json& guidance_json, const control::ControllerType& controller_type);
    void validate_linear_quadratic_control_setpoint(const nlohmann::json& guidance_json, const control::ControllerType& controller_type);
    void validate_nonlinear_control_setpoint(const nlohmann::json& guidance_json, const control::ControllerType& controller_type);
    void validate_guidance(const nlohmann::json& guidance_json);
    void validate_trajectory_type(const guidance::TrajectoryComponents& traj_components, guidance::TrajectoryType traj_type);
    void validate_trajectory_components(guidance::TrajectoryComponents& traj_components);

}
