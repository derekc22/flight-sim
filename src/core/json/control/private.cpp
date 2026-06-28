#include <stdexcept>
#include <string>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "core/json/control/private.hpp"
#include "core/json/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/control/public.hpp"
#include "simulation/control/linear_quadratic/controllers/lqi/public.hpp"
#include "simulation/control/linear_quadratic/controllers/lqr/public.hpp"
#include "simulation/control/linear_quadratic/controllers/lqt/public.hpp"
#include "simulation/control/pid/controllers/attitude/public.hpp"
#include "simulation/control/pid/controllers/damper/public.hpp"
#include "simulation/control/pid/controllers/velocity/public.hpp"

namespace json {

    control::AttitudePIDParameters parse_damper_pid_parameters(const nlohmann::json& controller_json) {
        const auto& parameters_json = controller_json.at("parameters");
        if (!parameters_json.is_object()) { 
            throw std::runtime_error("json::parse_damper_pid_parameters expected parameters object"); 
        }
        if (!parameters_json.contains("Kp_roll") || !parameters_json.contains("Kp_pitch") || !parameters_json.contains("Kp_yaw")) {
            throw std::runtime_error("json::parse_damper_pid_parameters requires Kp_roll, Kp_pitch, Kp_yaw");
        }

        control::AttitudePIDParameters params{};
        params.Kp_roll = parameters_json.at("Kp_roll").get<double>();
        params.Kp_pitch = parameters_json.at("Kp_pitch").get<double>();
        params.Kp_yaw = parameters_json.at("Kp_yaw").get<double>();
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

    control::AttitudeController make_attitude_controller(control::ControllerType controller_type, const nlohmann::json& controller_json) {
        switch (controller_type) {
            case control::ControllerType::AttitudePID: {
                control::AttitudePIDParameters params = parse_attitude_pid_parameters(controller_json);
                return make_stateful_controller<struct control::AttitudePID, control::AttitudeController, control::AttitudePIDParameters, control::AttitudeControllerInput>(params);
            }

            case control::ControllerType::DamperPID: {
                control::AttitudePIDParameters params = parse_damper_pid_parameters(controller_json);
                return make_stateful_controller<struct control::DamperPID, control::AttitudeController, control::AttitudePIDParameters, control::AttitudeControllerInput>(params);
            }

            default:
                throw std::runtime_error("json::make_attitude_controller unknown control type");
        }
    }

    control::VelocityController make_velocity_controller(control::ControllerType controller_type, const nlohmann::json& controller_json) {
        switch (controller_type) {
            case control::ControllerType::VelocityPID: {
                control::VelocityPIDParameters params = parse_velocity_pid_parameters(controller_json);
                return make_stateful_controller<struct control::VelocityPID, control::VelocityController, control::VelocityPIDParameters, control::VelocityControllerInput>(params);
            }

            default:
                throw std::runtime_error("json::make_velocity_controller unknown control type");
        }
    }

    control::LinearQuadraticController make_linear_quadratic_controller(control::ControllerType controller_type, const nlohmann::json& controller_json) {
        switch (controller_type) {
            case control::ControllerType::LinearQuadraticRegulator: {
                control::LinearQuadraticRegulatorParameters params = parse_linear_quadratic_regulator_parameters(controller_json);
                return make_stateful_controller<struct control::LinearQuadraticRegulator, control::LinearQuadraticController, control::LinearQuadraticRegulatorParameters, control::LinearQuadraticControllerInput>(params);
            }

            case control::ControllerType::LinearQuadraticIntegrator: {
                control::LinearQuadraticIntegratorParameters params = parse_linear_quadratic_integrator_parameters(controller_json);
                return make_stateful_controller<struct control::LinearQuadraticIntegrator, control::LinearQuadraticController, control::LinearQuadraticIntegratorParameters, control::LinearQuadraticControllerInput>(params);
            }

            case control::ControllerType::LinearQuadraticTracker:
                throw std::runtime_error("TODO: NOT IMPLEMENTED");

            default:
                throw std::runtime_error("json::make_linear_quadratic_controller unknown control type");
        }
    }

    control::NonlinearController make_nonlinear_controller(control::ControllerType controller_type, const nlohmann::json& controller_json) {
        switch (controller_type) {
            case control::ControllerType::FeedbackLinearization:
                throw std::runtime_error("TODO: NOT IMPLEMENTED");

            case control::ControllerType::NonlinearDynamicInversion:
                throw std::runtime_error("TODO: NOT IMPLEMENTED");

            case control::ControllerType::IncrementalNonlinearDynamicInversion:
                throw std::runtime_error("TODO: NOT IMPLEMENTED");

            default:
                throw std::runtime_error("json::make_nonlinear_controller unknown control type");
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

    void parse_attitude_controller(const nlohmann::json& controller_json, control::AttitudeController& controller, control::ControllerType& controller_type) {
        controller_type = fetch_controller_type(controller_json);
        controller = make_attitude_controller(controller_type, controller_json);
    }

    void parse_velocity_controller(const nlohmann::json& controller_json, control::VelocityController& controller, control::ControllerType& controller_type) {
        controller_type = fetch_controller_type(controller_json);
        controller = make_velocity_controller(controller_type, controller_json);
    }

    void parse_linear_quadratic_controller(const nlohmann::json& controller_json, control::LinearQuadraticController& controller, control::ControllerType& controller_type) {
        controller_type = fetch_controller_type(controller_json);
        controller = make_linear_quadratic_controller(controller_type, controller_json);
    }

    void parse_nonlinear_controller(const nlohmann::json& controller_json, control::NonlinearController& controller, control::ControllerType& controller_type) {
        controller_type = fetch_controller_type(controller_json);
        controller = make_nonlinear_controller(controller_type, controller_json);
    }

    void validate_controllers(const nlohmann::json& controllers_json) {
        bool attitude_bool = controllers_json.contains("attitude");
        bool velocity_bool = controllers_json.contains("velocity");
        bool linear_quadratic_bool = controllers_json.contains("linear_quadratic");
        bool nonlinear_bool = controllers_json.contains("nonlinear");

        if (attitude_bool && linear_quadratic_bool) { 
            throw std::runtime_error("json::validate_controllers: attitude and linear_quadratic control laws cannot both be present"); 
        }
        if (attitude_bool && nonlinear_bool) { 
            throw std::runtime_error("json::validate_controllers: attitude and nonlinear control laws cannot both be present"); 
        }

        if (linear_quadratic_bool && nonlinear_bool) { 
            throw std::runtime_error("json::validate_controllers: linear_quadratic and nonlinear control laws cannot both be present"); 
        }

        if (velocity_bool && linear_quadratic_bool) { 
            throw std::runtime_error("json::validate_controllers: velocity and linear_quadratic control laws cannot both be present"); 
        }
        if (velocity_bool && nonlinear_bool) { 
            throw std::runtime_error("json::validate_controllers: velocity and nonlinear control laws cannot both be present"); 
        }
    }

    control::ControlProperties parse_control_properties(const nlohmann::json& config) {
        validate_controllers(config);
        control::ControlProperties control_properties;

        if (config.contains("attitude")) {
            const auto& attitude_controller_json = config.at("attitude");
            parse_attitude_controller(attitude_controller_json, control_properties.attitude_controller, control_properties.attitude_controller_type);
        }

        if (config.contains("velocity")) {
            const auto& velocity_controller_json = config.at("velocity");
            parse_velocity_controller(velocity_controller_json, control_properties.velocity_controller, control_properties.velocity_controller_type);
        }

        if (config.contains("linear_quadratic")) {
            const auto& linear_quadratic_controller_json = config.at("linear_quadratic");
            parse_linear_quadratic_controller(linear_quadratic_controller_json, control_properties.linear_quadratic_controller, control_properties.linear_quadratic_controller_type);
        }

        if (config.contains("nonlinear")) {
            const auto& nonlinear_controller_json = config.at("nonlinear");
            parse_nonlinear_controller(nonlinear_controller_json, control_properties.nonlinear_controller, control_properties.nonlinear_controller_type);
        }

        return control_properties;
    }

}
