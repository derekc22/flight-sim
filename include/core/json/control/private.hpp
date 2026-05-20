#pragma once
#include <string>
#include <nlohmann/json.hpp>
#include "core/json/control/public.hpp"
#include "simulation/control/public.hpp"
#include "simulation/control/linear_quadratic/controllers/lqi/public.hpp"
#include "simulation/control/linear_quadratic/controllers/lqr/public.hpp"
#include "simulation/control/linear_quadratic/controllers/lqt/public.hpp"
#include "simulation/control/pid/controllers/axial/public.hpp"
#include "simulation/control/pid/controllers/damper/public.hpp"
#include "simulation/control/pid/controllers/velocity/public.hpp"

namespace json {

    template <typename ControllerType, typename ControllerClass, typename ControllerParametersType, typename ControllerInputType>
    ControllerClass make_stateful_controller(const ControllerParametersType& params);

    control::AxialPIDParameters parse_damper_pid_parameters(const nlohmann::json& controller_json);
    control::AxialPIDParameters parse_axial_pid_parameters(const nlohmann::json& controller_json);
    control::VelocityPIDParameters parse_velocity_pid_parameters(const nlohmann::json& controller_json);
    control::LinearQuadraticRegulatorParameters parse_linear_quadratic_regulator_parameters(const nlohmann::json& controller_json);
    control::LinearQuadraticIntegratorParameters parse_linear_quadratic_integrator_parameters(const nlohmann::json& controller_json);
    control::AxialController make_axial_controller(control::ControlType control_type, const nlohmann::json& controller_json);
    control::VelocityController make_velocity_controller(control::ControlType control_type, const nlohmann::json& controller_json);
    control::LinearFullStateFeedbackController make_linear_full_state_feedback_controller(control::ControlType control_type, const nlohmann::json& controller_json);
    control::NonlinearController make_nonlinear_controller(control::ControlType control_type, const nlohmann::json& controller_json);
    control::ControlType map_control_type(const std::string& control_type_str);
    control::ControlType fetch_control_type(const nlohmann::json& controller_json);
    void parse_axial_controller(const nlohmann::json& controller_json, control::AxialController& controller, control::ControlType& ctrl_type);
    void parse_velocity_controller(const nlohmann::json& controller_json, control::VelocityController& controller, control::ControlType& ctrl_type);
    void parse_linear_full_state_feedback_controller(const nlohmann::json& controller_json, control::LinearFullStateFeedbackController& controller, control::ControlType& ctrl_type);
    void parse_nonlinear_controller(const nlohmann::json& controller_json, control::NonlinearController& controller, control::ControlType& ctrl_type);
    void validate_controllers(const nlohmann::json& controllers_json);
    control::ControlProperties parse_control_properties(const nlohmann::json& config);
}

#include "core/json/control/private.tpp"
