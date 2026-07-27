#pragma once
#include <string>
#include <nlohmann/json.hpp>
#include "simulation/control/public.hpp"
#include "simulation/control/linear_quadratic/controllers/lqi/public.hpp"
#include "simulation/control/linear_quadratic/controllers/lqr/public.hpp"
#include "simulation/control/linear_quadratic/controllers/lqt/public.hpp"
#include "simulation/control/pid/controllers/attitude/public.hpp"
#include "simulation/control/pid/controllers/damper/public.hpp"
#include "simulation/control/pid/controllers/velocity/public.hpp"

namespace json {

    template <typename Controller, typename ControllerClass, typename ControllerParameters, typename ControllerClassInput>
    ControllerClass make_stateful_controller(const ControllerParameters& params);

    control::AttitudePIDParameters parse_damper_pid_parameters(const nlohmann::json& controller_json);
    control::AttitudePIDParameters parse_attitude_pid_parameters(const nlohmann::json& controller_json);
    control::VelocityPIDParameters parse_velocity_pid_parameters(const nlohmann::json& controller_json);
    control::LinearQuadraticRegulatorParameters parse_linear_quadratic_regulator_parameters(const nlohmann::json& controller_json);
    control::LinearQuadraticIntegratorParameters parse_linear_quadratic_integrator_parameters(const nlohmann::json& controller_json);

    control::AttitudeController make_attitude_controller(control::ControllerType controller_type, const nlohmann::json& controller_json);
    control::VelocityController make_velocity_controller(control::ControllerType controller_type, const nlohmann::json& controller_json);
    control::LinearQuadraticController make_linear_quadratic_controller(control::ControllerType controller_type, const nlohmann::json& controller_json);
    control::NonlinearController make_nonlinear_controller(control::ControllerType controller_type, const nlohmann::json& controller_json);

    control::ControllerType map_controller_type(const std::string& controller_type_str);
    control::ControllerType fetch_controller_type(const nlohmann::json& controller_json);

    void parse_attitude_controller(const nlohmann::json& controller_json, control::AttitudeController& controller, control::ControllerType& controller_type);
    void parse_velocity_controller(const nlohmann::json& controller_json, control::VelocityController& controller, control::ControllerType& controller_type);
    void parse_linear_quadratic_controller(const nlohmann::json& controller_json, control::LinearQuadraticController& controller, control::ControllerType& controller_type);
    void parse_nonlinear_controller(const nlohmann::json& controller_json, control::NonlinearController& controller, control::ControllerType& controller_type);

    void validate_controllers(const nlohmann::json& controllers_json, bool trim_flag);

    control::ControlProperties parse_control_properties(const nlohmann::json& config, bool trim_flag);
}

#include "core/json/control/private.tpp"
