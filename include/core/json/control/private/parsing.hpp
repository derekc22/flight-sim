#pragma once
#include <optional>
#include <string>
#include <nlohmann/json.hpp>
#include "simulation/control/public/manager.hpp"

namespace json {

    control::DamperPIDParameters parse_damper_pid_parameters(const nlohmann::json& controller_json);
    control::AttitudePIDParameters parse_attitude_pid_parameters(const nlohmann::json& controller_json);
    control::VelocityPIDParameters parse_velocity_pid_parameters(const nlohmann::json& controller_json);
    control::LinearQuadraticRegulatorParameters parse_linear_quadratic_regulator_parameters(const nlohmann::json& controller_json);
    control::LinearQuadraticIntegratorParameters parse_linear_quadratic_integrator_parameters(const nlohmann::json& controller_json);

    control::AttitudeControl make_attitude_control(control::ControllerType controller_type, const nlohmann::json& controller_json);
    control::VelocityControl make_velocity_control(control::ControllerType controller_type, const nlohmann::json& controller_json);
    control::FullStateControl make_full_state_control(control::ControllerType controller_type, const nlohmann::json& controller_json);

    control::ControllerType map_controller_type(const std::string& controller_type_str);
    control::ControllerType fetch_controller_type(const nlohmann::json& controller_json);

    void parse_attitude_control(const nlohmann::json& controller_json, std::optional<control::AttitudeControl>& component);
    void parse_velocity_control(const nlohmann::json& controller_json, std::optional<control::VelocityControl>& component);
    void parse_full_state_control(const nlohmann::json& controller_json, std::optional<control::FullStateControl>& component);

    control::ControlManager parse_control_manager(const nlohmann::json& config, bool trim_flag);
}
