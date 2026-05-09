#include <stdexcept>
#include <string>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "core/json/control.hpp"
#include "core/json/json.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/control/control.hpp"
#include "simulation/control/linear_quadratic/controllers/lqi.hpp"
#include "simulation/control/linear_quadratic/controllers/lqr.hpp"
#include "simulation/control/linear_quadratic/controllers/lqt.hpp"
#include "simulation/control/pid/controllers/axial.hpp"
#include "simulation/control/pid/controllers/damper.hpp"
#include "simulation/control/pid/controllers/velocity.hpp"

namespace json {

    template <typename ControllerType, typename ControllerClass, typename ControllerParametersType, typename ControllerInputType>
    ControllerClass make_stateful_controller(const ControllerParametersType& params) {
        return [controller = ControllerType{ params }](const ControllerInputType& input) mutable {
            return controller.step(input);
        };
    }

    control::AxialPIDParameters parse_damper_pid_parameters(const nlohmann::json& controller_json) {
        const auto& parameters_json = controller_json.at("parameters");
        if (!parameters_json.is_object()) { throw std::runtime_error("json::parse_damper_pid_parameters expected parameters object"); }
        if (!parameters_json.contains("Kp_roll") || !parameters_json.contains("Kp_pitch") || !parameters_json.contains("Kp_yaw")) {
            throw std::runtime_error("json::parse_damper_pid_parameters requires Kp_roll, Kp_pitch, Kp_yaw");
        }

        control::AxialPIDParameters params{};
        params.Kp_roll = parameters_json.at("Kp_roll").get<double>();
        params.Kp_pitch = parameters_json.at("Kp_pitch").get<double>();
        params.Kp_yaw = parameters_json.at("Kp_yaw").get<double>();
        return params;
    }

    control::AxialPIDParameters parse_axial_pid_parameters(const nlohmann::json& controller_json) {
        const auto& parameters_json = controller_json.at("parameters");
        if (!parameters_json.is_object()) { throw std::runtime_error("json::parse_axial_pid_parameters expected parameters object"); }
        if (
            !parameters_json.contains("Kp_roll") || !parameters_json.contains("Ki_roll") || !parameters_json.contains("Kd_roll") ||
            !parameters_json.contains("Kp_pitch") || !parameters_json.contains("Ki_pitch") || !parameters_json.contains("Kd_pitch") ||
            !parameters_json.contains("Kp_yaw") || !parameters_json.contains("Ki_yaw") || !parameters_json.contains("Kd_yaw")
        ) {
            throw std::runtime_error("json::parse_axial_pid_parameters requires Kp, Ki, and Kd for the lateral, longitudinal, and vertical axes");
        }
        if (!parameters_json.contains("tau")) {
            throw std::runtime_error("json::parse_axial_pid_parameters requires tau");
        }
        if (parameters_json.at("tau").get<double>() < 0.0) {
            throw std::runtime_error("json::parse_axial_pid_parameters requires non-negative tau");
        }

        control::AxialPIDParameters params{};
        params.Kp_roll = parameters_json.at("Kp_roll").get<double>();
        params.Ki_roll = parameters_json.at("Ki_roll").get<double>();
        params.Kd_roll = parameters_json.at("Kd_roll").get<double>();
        params.Kp_pitch = parameters_json.at("Kp_pitch").get<double>();
        params.Ki_pitch = parameters_json.at("Ki_pitch").get<double>();
        params.Kd_pitch = parameters_json.at("Kd_pitch").get<double>();
        params.Kp_yaw = parameters_json.at("Kp_yaw").get<double>();
        params.Ki_yaw = parameters_json.at("Ki_yaw").get<double>();
        params.Kd_yaw = parameters_json.at("Kd_yaw").get<double>();
        params.tau = parameters_json.at("tau").get<double>();
        return params;
    }

    control::VelocityPIDParameters parse_velocity_pid_parameters(const nlohmann::json& controller_json) {
        const auto& parameters_json = controller_json.at("parameters");
        if (!parameters_json.is_object()) { throw std::runtime_error("json::parse_velocity_pid_parameters expected parameters object"); }
        if (!parameters_json.contains("Kp") || !parameters_json.contains("Ki") || !parameters_json.contains("Kd")) {
            throw std::runtime_error("json::parse_velocity_pid_parameters requires Kp, Kd, and Ki");
        }
        if (!parameters_json.contains("tau")) { throw std::runtime_error("json::parse_velocity_pid_parameters requires tau"); }
        if (parameters_json.at("tau").get<double>() < 0.0) { throw std::runtime_error("json::parse_velocity_pid_parameters requires non-negative tau"); }

        control::VelocityPIDParameters params{};
        params.Kp = parameters_json.at("Kp").get<double>();
        params.Ki = parameters_json.at("Ki").get<double>();
        params.Kd = parameters_json.at("Kd").get<double>();
        params.tau = parameters_json.at("tau").get<double>();
        return params;
    }

    control::LinearQuadraticRegulatorParameters parse_linear_quadratic_regulator_parameters(const nlohmann::json& controller_json) {
        const auto& parameters_json = controller_json.at("parameters");
        if (!parameters_json.is_object()) { throw std::runtime_error("json::parse_linear_quadratic_regulator_parameters expected parameters object"); }
        if (!parameters_json.contains("Q") || !parameters_json.contains("R")) {
            throw std::runtime_error("json::parse_linear_quadratic_regulator_parameters requires Q and R");
        }

        control::LinearQuadraticRegulatorParameters params{};
        params.Q = parse_MatrixXd(parameters_json.at("Q"));
        params.R = parse_MatrixXd(parameters_json.at("R"));
        return params;
    }

    control::LinearQuadraticIntegratorParameters parse_linear_quadratic_integrator_parameters(const nlohmann::json& controller_json) {
        const auto& parameters_json = controller_json.at("parameters");
        if (!parameters_json.is_object()) { throw std::runtime_error("json::parse_linear_quadratic_integrator_parameters expected parameters object"); }
        if (!parameters_json.contains("Q") || !parameters_json.contains("Qi") || !parameters_json.contains("R")) {
            throw std::runtime_error("json::parse_linear_quadratic_integrator_parameters requires Q, Qi, and R");
        }

        control::LinearQuadraticIntegratorParameters params{};
        params.Q = parse_MatrixXd(parameters_json.at("Q"));
        params.Qi = parse_MatrixXd(parameters_json.at("Qi"));
        params.R = parse_MatrixXd(parameters_json.at("R"));
        return params;
    }

    control::AxialController make_axial_controller(control::ControlType control_type, const nlohmann::json& controller_json) {
        switch (control_type) {
            case control::ControlType::AxialPID: {
                control::AxialPIDParameters params = parse_axial_pid_parameters(controller_json);
                return make_stateful_controller<struct control::AxialPID, control::AxialController, control::AxialPIDParameters, control::AxialControllerInput>(params);
            }

            case control::ControlType::DamperPID: {
                control::AxialPIDParameters params = parse_damper_pid_parameters(controller_json);
                return make_stateful_controller<struct control::DamperPID, control::AxialController, control::AxialPIDParameters, control::AxialControllerInput>(params);
            }

            default:
                throw std::runtime_error("json::make_axial_controller unknown control type");
        }
    }

    control::VelocityController make_velocity_controller(control::ControlType control_type, const nlohmann::json& controller_json) {
        switch (control_type) {
            case control::ControlType::VelocityPID: {
                control::VelocityPIDParameters params = parse_velocity_pid_parameters(controller_json);
                return make_stateful_controller<struct control::VelocityPID, control::VelocityController, control::VelocityPIDParameters, control::VelocityControllerInput>(params);
            }

            default:
                throw std::runtime_error("json::make_velocity_controller unknown control type");
        }
    }

    control::LinearFullStateFeedbackController make_linear_full_state_feedback_controller(control::ControlType control_type, const nlohmann::json& controller_json) {
        switch (control_type) {
            case control::ControlType::LinearQuadraticRegulator: {
                control::LinearQuadraticRegulatorParameters params = parse_linear_quadratic_regulator_parameters(controller_json);
                return make_stateful_controller<struct control::LinearQuadraticRegulator, control::LinearFullStateFeedbackController, control::LinearQuadraticRegulatorParameters, control::LinearFullStateFeedbackControllerInput>(params);
            }

            case control::ControlType::LinearQuadraticIntegrator: {
                control::LinearQuadraticIntegratorParameters params = parse_linear_quadratic_integrator_parameters(controller_json);
                return make_stateful_controller<struct control::LinearQuadraticIntegrator, control::LinearFullStateFeedbackController, control::LinearQuadraticIntegratorParameters, control::LinearFullStateFeedbackControllerInput>(params);
            }

            case control::ControlType::LinearQuadraticTracker:
                throw std::runtime_error("TODO: NOT IMPLEMENTED");

            default:
                throw std::runtime_error("json::make_linear_full_state_feedback_controller unknown control type");
        }
    }

    control::NonlinearController make_nonlinear_controller(control::ControlType control_type, const nlohmann::json& controller_json) {
        switch (control_type) {
            case control::ControlType::FeedbackLinearization:
                throw std::runtime_error("TODO: NOT IMPLEMENTED");

            case control::ControlType::NonlinearDynamicInversion:
                throw std::runtime_error("TODO: NOT IMPLEMENTED");

            case control::ControlType::IncrementalNonlinearDynamicInversion:
                throw std::runtime_error("TODO: NOT IMPLEMENTED");

            default:
                throw std::runtime_error("json::make_nonlinear_controller unknown control type");
        }
    }

    control::ControlType map_control_type(const std::string& control_type_str) {
        if (control_type_str == "AxialPID") { return control::ControlType::AxialPID; }
        if (control_type_str == "DamperPID") { return control::ControlType::DamperPID; }
        if (control_type_str == "VelocityPID") { return control::ControlType::VelocityPID; }
        if (control_type_str == "LinearQuadraticRegulator") { return control::ControlType::LinearQuadraticRegulator; }
        if (control_type_str == "LinearQuadraticIntegrator") { return control::ControlType::LinearQuadraticIntegrator; }
        if (control_type_str == "LinearQuadraticTracker") { return control::ControlType::LinearQuadraticTracker; }
        if (control_type_str == "FeedbackLinearization") { return control::ControlType::FeedbackLinearization; }
        if (control_type_str == "NonlinearDynamicInversion") { return control::ControlType::NonlinearDynamicInversion; }
        if (control_type_str == "IncrementalNonlinearDynamicInversion") { return control::ControlType::IncrementalNonlinearDynamicInversion; }
        throw std::runtime_error("json::map_control_type unknown control type: " + control_type_str);
    }

    control::ControlType fetch_control_type(const nlohmann::json& controller_json){
        std::string control_type_str = controller_json.at("control_type").get<std::string>();
        return map_control_type(control_type_str);
    }

    void parse_axial_controller(const nlohmann::json& controller_json, control::AxialController& controller, control::ControlType& ctrl_type) {
        ctrl_type = fetch_control_type(controller_json);
        controller = make_axial_controller(
            ctrl_type, 
            controller_json
        );
    }

    void parse_velocity_controller(const nlohmann::json& controller_json, control::VelocityController& controller, control::ControlType& ctrl_type) {
        ctrl_type = fetch_control_type(controller_json);
        controller = make_velocity_controller(
            ctrl_type, 
            controller_json
        );    }

    void parse_linear_full_state_feedback_controller(const nlohmann::json& controller_json, control::LinearFullStateFeedbackController& controller, control::ControlType& ctrl_type) {
        ctrl_type = fetch_control_type(controller_json);
        controller = make_linear_full_state_feedback_controller(
            ctrl_type, 
            controller_json
        );
    }

    void parse_nonlinear_controller(const nlohmann::json& controller_json, control::NonlinearController& controller, control::ControlType& ctrl_type) {
        ctrl_type = fetch_control_type(controller_json);
        controller = make_nonlinear_controller(
            ctrl_type, 
            controller_json
        );
    }

    void validate_controllers(const nlohmann::json& controllers_json){
        bool axial_bool = controllers_json.contains("axial");
        bool velocity_bool = controllers_json.contains("velocity");
        bool linear_full_state_feedback_bool = controllers_json.contains("linear_full_state_feedback");
        bool nonlinear_bool = controllers_json.contains("nonlinear");

        if (axial_bool && linear_full_state_feedback_bool) { throw std::runtime_error("json::validate_controllers: axial and linear_full_state_feedback control laws cannot both be present"); }
        if (axial_bool && nonlinear_bool) { throw std::runtime_error("json::validate_controllers: axial and nonlinear control laws cannot both be present"); }

        if (linear_full_state_feedback_bool && nonlinear_bool) { throw std::runtime_error("json::validate_controllers: linear_full_state_feedback and nonlinear control laws cannot both be present"); }

        if (velocity_bool && linear_full_state_feedback_bool) { throw std::runtime_error("json::validate_controllers: velocity and linear_full_state_feedback control laws cannot both be present"); }
        if (velocity_bool && nonlinear_bool) { throw std::runtime_error("json::validate_controllers: velocity and nonlinear control laws cannot both be present"); }
    }

    control::ControlProperties parse_control_properties(const nlohmann::json& config) {
        validate_controllers(config);
        control::ControlProperties control_properties;

        if (config.contains("axial")) {
            const auto& axial_controller_json = config.at("axial");
            parse_axial_controller(
                axial_controller_json,
                control_properties.axial_controller,
                control_properties.axial_control_type
            );
        }

        if (config.contains("velocity")) {
            const auto& velocity_controller_json = config.at("velocity");
            parse_velocity_controller(
                velocity_controller_json,
                control_properties.velocity_controller,
                control_properties.velocity_control_type
            );
        }

        if (config.contains("linear_full_state_feedback")) {
            const auto& linear_full_state_feedback_controller_json = config.at("linear_full_state_feedback");
            parse_linear_full_state_feedback_controller(
                linear_full_state_feedback_controller_json,
                control_properties.linear_full_state_feedback_controller,
                control_properties.linear_full_state_feedback_control_type
            );
        }

        if (config.contains("nonlinear")) {
            const auto& nonlinear_controller_json = config.at("nonlinear");
            parse_nonlinear_controller(
                nonlinear_controller_json,
                control_properties.nonlinear_controller,
                control_properties.nonlinear_control_type
            );
        }

        return control_properties;
    }

    control::ControlProperties parse_control_config() {
        const auto config_path = resolve_run_config_entry_path("control_config");
        const auto config = read_json_file(config_path);
        return parse_control_properties(config);
    }

}
