#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>
#include "core/json/control/private/parsing.hpp"
#include "core/json/control/private/validation.hpp"
#include "core/json/public/data/helpers.hpp"
#include "simulation/control/public/manager.hpp"

namespace json {

    control::AttitudePIDParameters parse_damper_pid_parameters(const nlohmann::json& controller_json) {
        const auto& parameters_json = controller_json.at("parameters");
        if (!parameters_json.is_object()) { 
            throw std::runtime_error("json::parse_damper_pid_parameters expected parameters object"); 
        }
        if (
            !parameters_json.contains("Kp_roll") || !parameters_json.contains("Ki_roll") || !parameters_json.contains("Kd_roll") ||
            !parameters_json.contains("Kp_pitch") || !parameters_json.contains("Ki_pitch") || !parameters_json.contains("Kd_pitch") ||
            !parameters_json.contains("Kp_yaw") || !parameters_json.contains("Ki_yaw") || !parameters_json.contains("Kd_yaw")
        ) {
            throw std::runtime_error("json::parse_damper_pid_parameters requires Kp, Ki, and Kd for the lateral, longitudinal, and vertical axes");
        }
        if (!parameters_json.contains("tau")) {
            throw std::runtime_error("json::parse_damper_pid_parameters requires tau");
        }
        if (parameters_json.at("tau").get<double>() < 0.0) {
            throw std::runtime_error("json::parse_damper_pid_parameters requires non-negative tau");
        }

        control::AttitudePIDParameters params{};
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

    control::AttitudePIDParameters parse_attitude_pid_parameters(const nlohmann::json& controller_json) {
        const auto& parameters_json = controller_json.at("parameters");
        if (!parameters_json.is_object()) { 
            throw std::runtime_error("json::parse_attitude_pid_parameters expected parameters object"); 
        }
        if (
            !parameters_json.contains("Kp_roll") || !parameters_json.contains("Ki_roll") || !parameters_json.contains("Kd_roll") ||
            !parameters_json.contains("Kp_pitch") || !parameters_json.contains("Ki_pitch") || !parameters_json.contains("Kd_pitch") ||
            !parameters_json.contains("Kp_yaw") || !parameters_json.contains("Ki_yaw") || !parameters_json.contains("Kd_yaw")
        ) {
            throw std::runtime_error("json::parse_attitude_pid_parameters requires Kp, Ki, and Kd for the lateral, longitudinal, and vertical axes");
        }
        if (!parameters_json.contains("tau")) {
            throw std::runtime_error("json::parse_attitude_pid_parameters requires tau");
        }
        if (parameters_json.at("tau").get<double>() < 0.0) {
            throw std::runtime_error("json::parse_attitude_pid_parameters requires non-negative tau");
        }

        control::AttitudePIDParameters params{};
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
        if (!parameters_json.is_object()) { 
            throw std::runtime_error("json::parse_velocity_pid_parameters expected parameters object"); 
        }
        if (!parameters_json.contains("Kp") || !parameters_json.contains("Ki") || !parameters_json.contains("Kd")) {
            throw std::runtime_error("json::parse_velocity_pid_parameters requires Kp, Kd, and Ki");
        }
        if (!parameters_json.contains("tau")) { 
            throw std::runtime_error("json::parse_velocity_pid_parameters requires tau"); 
        }
        if (parameters_json.at("tau").get<double>() < 0.0) { 
            throw std::runtime_error("json::parse_velocity_pid_parameters requires non-negative tau"); 
        }

        control::VelocityPIDParameters params{};
        params.Kp = parameters_json.at("Kp").get<double>();
        params.Ki = parameters_json.at("Ki").get<double>();
        params.Kd = parameters_json.at("Kd").get<double>();
        params.tau = parameters_json.at("tau").get<double>();
        return params;
    }

    control::LinearQuadraticRegulatorParameters parse_linear_quadratic_regulator_parameters(const nlohmann::json& controller_json) {
        const auto& parameters_json = controller_json.at("parameters");
        if (!parameters_json.is_object()) { 
            throw std::runtime_error("json::parse_linear_quadratic_regulator_parameters expected parameters object"); 
        }
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
        if (!parameters_json.is_object()) { 
            throw std::runtime_error("json::parse_linear_quadratic_integrator_parameters expected parameters object"); 
        }
        if (!parameters_json.contains("Q") || !parameters_json.contains("Qi") || !parameters_json.contains("R")) {
            throw std::runtime_error("json::parse_linear_quadratic_integrator_parameters requires Q, Qi, and R");
        }

        control::LinearQuadraticIntegratorParameters params{};
        params.Q = parse_MatrixXd(parameters_json.at("Q"));
        params.Qi = parse_MatrixXd(parameters_json.at("Qi"));
        params.R = parse_MatrixXd(parameters_json.at("R"));
        return params;
    }

    control::AttitudeControl make_attitude_control(control::ControllerType controller_type, const nlohmann::json& controller_json) {
        switch (controller_type) {
            case control::ControllerType::AttitudePID: {
                control::AttitudePIDParameters params = parse_attitude_pid_parameters(controller_json);
                return control::AttitudeControl(controller_type, params);
            }

            case control::ControllerType::DamperPID: {
                control::AttitudePIDParameters params = parse_damper_pid_parameters(controller_json);
                return control::AttitudeControl(controller_type, params);
            }

            default:
                throw std::runtime_error("json::make_attitude_control unknown control type");
        }
    }

    control::VelocityControl make_velocity_control(control::ControllerType controller_type, const nlohmann::json& controller_json) {
        switch (controller_type) {
            case control::ControllerType::VelocityPID: {
                control::VelocityPIDParameters params = parse_velocity_pid_parameters(controller_json);
                return control::VelocityControl(controller_type, params);
            }

            default:
                throw std::runtime_error("json::make_velocity_control unknown control type");
        }
    }

    control::FullStateControl make_full_state_control(control::ControllerType controller_type, const nlohmann::json& controller_json) {
        switch (controller_type) {
            case control::ControllerType::LinearQuadraticRegulator: {
                control::LinearQuadraticRegulatorParameters params = parse_linear_quadratic_regulator_parameters(controller_json);
                return control::FullStateControl(controller_type, params);
            }

            case control::ControllerType::LinearQuadraticIntegrator: {
                control::LinearQuadraticIntegratorParameters params = parse_linear_quadratic_integrator_parameters(controller_json);
                return control::FullStateControl(controller_type, params);
            }

            case control::ControllerType::LinearQuadraticTracker:
                throw std::runtime_error("TODO: NOT IMPLEMENTED");

            case control::ControllerType::FeedbackLinearization:
                throw std::runtime_error("TODO: NOT IMPLEMENTED");

            case control::ControllerType::NonlinearDynamicInversion:
                throw std::runtime_error("TODO: NOT IMPLEMENTED");

            case control::ControllerType::IncrementalNonlinearDynamicInversion:
                throw std::runtime_error("TODO: NOT IMPLEMENTED");

            default:
                throw std::runtime_error("json::make_full_state_control unknown control type");
        }
    }

    control::ControllerType map_controller_type(const std::string& controller_type_str) {
        if (controller_type_str == "AttitudePID") { return control::ControllerType::AttitudePID; }
        if (controller_type_str == "DamperPID") { return control::ControllerType::DamperPID; }
        if (controller_type_str == "VelocityPID") { return control::ControllerType::VelocityPID; }
        if (controller_type_str == "LinearQuadraticRegulator") { return control::ControllerType::LinearQuadraticRegulator; }
        if (controller_type_str == "LinearQuadraticIntegrator") { return control::ControllerType::LinearQuadraticIntegrator; }
        if (controller_type_str == "LinearQuadraticTracker") { return control::ControllerType::LinearQuadraticTracker; }
        if (controller_type_str == "FeedbackLinearization") { return control::ControllerType::FeedbackLinearization; }
        if (controller_type_str == "NonlinearDynamicInversion") { return control::ControllerType::NonlinearDynamicInversion; }
        if (controller_type_str == "IncrementalNonlinearDynamicInversion") { return control::ControllerType::IncrementalNonlinearDynamicInversion; }
        throw std::runtime_error("json::map_controller_type unknown control type: " + controller_type_str);
    }

    control::ControllerType fetch_controller_type(const nlohmann::json& controller_json) {
        std::string controller_type_str = controller_json.at("controller_type").get<std::string>();
        return map_controller_type(controller_type_str);
    }

    void parse_attitude_control(const nlohmann::json& controller_json, std::optional<control::AttitudeControl>& component) {
        control::ControllerType controller_type = fetch_controller_type(controller_json);
        component = make_attitude_control(controller_type, controller_json);
    }

    void parse_velocity_control(const nlohmann::json& controller_json, std::optional<control::VelocityControl>& component) {
        control::ControllerType controller_type = fetch_controller_type(controller_json);
        component = make_velocity_control(controller_type, controller_json);
    }

    void parse_full_state_control(const nlohmann::json& controller_json, std::optional<control::FullStateControl>& component) {
        control::ControllerType controller_type = fetch_controller_type(controller_json);
        component = make_full_state_control(controller_type, controller_json);
    }

    control::ControlManager parse_control_manager(const nlohmann::json& config, bool trim_flag) {
        validate_controllers(config, trim_flag);
        control::ControlManager control_manager;

        if (config.contains("attitude")) {
            const auto& attitude_controller_json = config.at("attitude");
            parse_attitude_control(attitude_controller_json, control_manager.attitude_control);
        }

        if (config.contains("velocity")) {
            const auto& velocity_controller_json = config.at("velocity");
            parse_velocity_control(velocity_controller_json, control_manager.velocity_control);
        }

        if (config.contains("linear_quadratic")) {
            const auto& linear_quadratic_controller_json = config.at("linear_quadratic");
            parse_full_state_control(linear_quadratic_controller_json, control_manager.full_state_control);
        }

        if (config.contains("nonlinear")) {
            const auto& nonlinear_controller_json = config.at("nonlinear");
            parse_full_state_control(nonlinear_controller_json, control_manager.full_state_control);
        }

        return control_manager;
    }

}
