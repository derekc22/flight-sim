#include <stdexcept>
#include <string>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "core/json/control.hpp"
#include "core/json/json.hpp"
#include "simulation/constants/constants.hpp"

namespace json {

    template <typename ControlLawType, typename ControlLawClass, typename ControlLawParametersType, typename BaseControlLawInputType, typename ControlLawInputType>
    ControlLawClass make_stateful_control_law(const ControlLawParametersType& params) {
        return [controller = ControlLawType{ params }](const BaseControlLawInputType& input) mutable {
            return controller.step(static_cast<const ControlLawInputType&>(input));
        };
    }

    control::AxialPIDParameters parse_damper_pid_parameters(const nlohmann::json& controller_json) {
        const auto& parameters_json = controller_json.at("parameters");
        if (!parameters_json.is_object()) { throw std::runtime_error("json::parse_damper_pid_parameters expected parameters object"); }
        if (!parameters_json.contains("Kp_lateral") || !parameters_json.contains("Kp_longitudinal") || !parameters_json.contains("Kp_vertical")) {
            throw std::runtime_error("json::parse_damper_pid_parameters requires Kp_lateral, Kp_longitudinal, Kp_vertical");
        }

        control::AxialPIDParameters params{};
        params.Kp_lateral = parameters_json.at("Kp_lateral").get<double>();
        params.Kp_longitudinal = parameters_json.at("Kp_longitudinal").get<double>();
        params.Kp_vertical = parameters_json.at("Kp_vertical").get<double>();
        return params;
    }

    control::AxialPIDParameters parse_axial_pid_parameters(const nlohmann::json& controller_json) {
        const auto& parameters_json = controller_json.at("parameters");
        if (!parameters_json.is_object()) { throw std::runtime_error("json::parse_axial_pid_parameters expected parameters object"); }
        if (
            !parameters_json.contains("Kp_lateral") || !parameters_json.contains("Ki_lateral") || !parameters_json.contains("Kd_lateral") ||
            !parameters_json.contains("Kp_longitudinal") || !parameters_json.contains("Ki_longitudinal") || !parameters_json.contains("Kd_longitudinal") ||
            !parameters_json.contains("Kp_vertical") || !parameters_json.contains("Ki_vertical") || !parameters_json.contains("Kd_vertical")
        ) {
            throw std::runtime_error("json::parse_axial_pid_parameters requires Kp, Ki, and Kd for the lateral, longitudinal, and vertical axes");
        }
        if (!parameters_json.contains("tau_lateral") || !parameters_json.contains("tau_longitudinal") || !parameters_json.contains("tau_vertical")) {
            throw std::runtime_error("json::parse_axial_pid_parameters requires tau for the lateral, longitudinal, and vertical axes");
        }
        if (parameters_json.at("tau_lateral").get<double>() < 0.0 || parameters_json.at("tau_longitudinal").get<double>() < 0.0 || parameters_json.at("tau_vertical").get<double>() < 0.0) {
            throw std::runtime_error("json::parse_axial_pid_parameters requires non-negative tau");
        }

        control::AxialPIDParameters params{};
        params.Kp_lateral = parameters_json.at("Kp_lateral").get<double>();
        params.Ki_lateral = parameters_json.at("Ki_lateral").get<double>();
        params.Kd_lateral = parameters_json.at("Kd_lateral").get<double>();
        params.tau_lateral = parameters_json.at("tau_lateral").get<double>();
        params.Kp_longitudinal = parameters_json.at("Kp_longitudinal").get<double>();
        params.Ki_longitudinal = parameters_json.at("Ki_longitudinal").get<double>();
        params.Kd_longitudinal = parameters_json.at("Kd_longitudinal").get<double>();
        params.tau_longitudinal = parameters_json.at("tau_longitudinal").get<double>();
        params.Kp_vertical = parameters_json.at("Kp_vertical").get<double>();
        params.Ki_vertical = parameters_json.at("Ki_vertical").get<double>();
        params.Kd_vertical = parameters_json.at("Kd_vertical").get<double>();
        params.tau_vertical = parameters_json.at("tau_vertical").get<double>();
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

    control::AxialControlLaw make_axial_control_law(control::ControlType control_type, const nlohmann::json& controller_json) {
        switch (control_type) {
            case control::ControlType::AxialPID: {
                control::AxialPIDParameters params = parse_axial_pid_parameters(controller_json);
                return make_stateful_control_law<struct control::AxialPID, control::AxialControlLaw, control::AxialPIDParameters, control::AxialControlLawInput, control::AxialPIDInput>(params);
            }

            case control::ControlType::DamperPID: {
                control::AxialPIDParameters params = parse_damper_pid_parameters(controller_json);
                return make_stateful_control_law<struct control::DamperPID, control::AxialControlLaw, control::AxialPIDParameters, control::AxialControlLawInput, control::AxialPIDInput>(params);
            }

            default:
                throw std::runtime_error("json::make_axial_control_law unknown control type");
        }
    }

    control::VelocityControlLaw make_velocity_control_law(control::ControlType control_type, const nlohmann::json& controller_json) {
        switch (control_type) {
            case control::ControlType::VelocityPID: {
                control::VelocityPIDParameters params = parse_velocity_pid_parameters(controller_json);
                return make_stateful_control_law<struct control::VelocityPID, control::VelocityControlLaw, control::VelocityPIDParameters, control::VelocityControlLawInput, control::VelocityPIDInput>(params);
            }

            default:
                throw std::runtime_error("json::make_velocity_control_law unknown control type");
        }
    }

    control::LinearFullStateFeedbackControlLaw make_linear_full_state_feedback_control_law(control::ControlType control_type, const nlohmann::json& controller_json) {
        switch (control_type) {
            case control::ControlType::LinearQuadraticRegulator: {
                control::LinearQuadraticRegulatorParameters params = parse_linear_quadratic_regulator_parameters(controller_json);
                return make_stateful_control_law<struct control::LinearQuadraticRegulator, control::LinearFullStateFeedbackControlLaw, control::LinearQuadraticRegulatorParameters, control::LinearFullStateFeedbackControlLawInput, control::LinearQuadraticRegulatorInput>(params);
            }

            case control::ControlType::LinearQuadraticIntegrator: {
                control::LinearQuadraticIntegratorParameters params = parse_linear_quadratic_integrator_parameters(controller_json);
                return make_stateful_control_law<struct control::LinearQuadraticIntegrator, control::LinearFullStateFeedbackControlLaw, control::LinearQuadraticIntegratorParameters, control::LinearFullStateFeedbackControlLawInput, control::LinearQuadraticRegulatorInput>(params);
            }

            case control::ControlType::LinearQuadraticTracker:
                throw std::runtime_error("json::make_linear_full_state_feedback_control_law LinearQuadraticTracker is not implemented");

            default:
                throw std::runtime_error("json::make_linear_full_state_feedback_control_law unknown control type");
        }
    }

    control::NonlinearControlLaw make_nonlinear_control_law(control::ControlType control_type, const nlohmann::json& controller_json) {
        switch (control_type) {
            case control::ControlType::FeedbackLinearization:
                throw std::runtime_error("json::make_nonlinear_control_law FeedbackLinearization is not implemented");

            case control::ControlType::NonlinearDynamicInversion:
                throw std::runtime_error("json::make_nonlinear_control_law NonlinearDynamicInversion is not implemented");

            case control::ControlType::IncrementalNonlinearDynamicInversion:
                throw std::runtime_error("json::make_nonlinear_control_law IncrementalNonlinearDynamicInversion is not implemented");

            default:
                throw std::runtime_error("json::make_nonlinear_control_law unknown control type");
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

    void parse_axial_control_law(const nlohmann::json& controller_json, control::AxialControlLaw& control_law) {
        std::string control_type_str = controller_json.at("control_type").get<std::string>();

        control::ControlType control_type = map_control_type(control_type_str);
        control_law = make_axial_control_law(control_type, controller_json);
    }

    void parse_velocity_control_law(const nlohmann::json& controller_json, control::VelocityControlLaw& control_law) {
        std::string control_type_str = controller_json.at("control_type").get<std::string>();

        control::ControlType control_type = map_control_type(control_type_str);
        control_law = make_velocity_control_law(control_type, controller_json);
    }

    void parse_linear_full_state_feedback_control_law(const nlohmann::json& controller_json, control::LinearFullStateFeedbackControlLaw& control_law) {
        std::string control_type_str = controller_json.at("control_type").get<std::string>();

        control::ControlType control_type = map_control_type(control_type_str);
        control_law = make_linear_full_state_feedback_control_law(control_type, controller_json);
    }

    void parse_nonlinear_control_law(const nlohmann::json& controller_json, control::NonlinearControlLaw& control_law) {
        std::string control_type_str = controller_json.at("control_type").get<std::string>();

        control::ControlType control_type = map_control_type(control_type_str);
        control_law = make_nonlinear_control_law(control_type, controller_json);
    }

    void validate_control_laws(const nlohmann::json& controllers_json){
        bool axial_bool = controllers_json.contains("axial");
        bool velocity_bool = controllers_json.contains("velocity");
        bool linear_full_state_feedback_bool = controllers_json.contains("linear_full_state_feedback");
        bool nonlinear_bool = controllers_json.contains("nonlinear");

        if (axial_bool && linear_full_state_feedback_bool) { throw std::runtime_error("json::parse_control_properties: axial and linear_full_state_feedback control laws cannot both be present"); }
        if (axial_bool && nonlinear_bool) { throw std::runtime_error("json::parse_control_properties: axial and nonlinear control laws cannot both be present"); }

        if (linear_full_state_feedback_bool && nonlinear_bool) { throw std::runtime_error("json::parse_control_properties: linear_full_state_feedback and nonlinear control laws cannot both be present"); }

        if (velocity_bool && linear_full_state_feedback_bool) { throw std::runtime_error("json::parse_control_properties: velocity and linear_full_state_feedback control laws cannot both be present"); }
        if (velocity_bool && nonlinear_bool) { throw std::runtime_error("json::parse_control_properties: velocity and nonlinear control laws cannot both be present"); }
    }

    control::ControlProperties parse_control_properties(const nlohmann::json& config) {
        const auto& controllers_json = config.at("controllers");
        validate_control_laws(controllers_json);

        const auto& setpoint_json = config.at("setpoint");
        control::ControlProperties control_properties;

        if (controllers_json.contains("axial")) {
            const auto& axial_controller_json = controllers_json.at("axial");
            parse_axial_control_law(
                axial_controller_json,
                control_properties.axial_control_law
            );
            // control_properties.axial_setpoint = parse_axial_control_setpoint(setpoint_json);
        }

        if (controllers_json.contains("velocity")) {
            const auto& velocity_controller_json = controllers_json.at("velocity");
            parse_velocity_control_law(
                velocity_controller_json,
                control_properties.velocity_control_law
            );
            // control_properties.velocity_setpoint = parse_velocity_control_setpoint(setpoint_json, control_properties.velocity_control_type);
        }

        if (controllers_json.contains("linear_full_state_feedback")) {
            const auto& linear_full_state_feedback_controller_json = controllers_json.at("linear_full_state_feedback");
            parse_linear_full_state_feedback_control_law(
                linear_full_state_feedback_controller_json,
                control_properties.linear_full_state_feedback_control_law
            );
            // control_properties.linear_full_state_feedback_setpoint = parse_linear_full_state_feedback_control_setpoint(setpoint_json, control_properties.linear_full_state_feedback_control_type);
        }

        if (controllers_json.contains("nonlinear")) {
            const auto& nonlinear_controller_json = controllers_json.at("nonlinear");
            parse_nonlinear_control_law(
                nonlinear_controller_json,
                control_properties.nonlinear_control_law
            );
            // control_properties.nonlinear_setpoint = parse_nonlinear_control_setpoint(setpoint_json, control_properties.linear_full_state_feedback_control_type);
        }

        return control_properties;
    }

    control::ControlProperties parse_control_config() {
        const auto config_path = resolve_run_config_entry_path("control_config");
        const auto config = read_json_file(config_path);
        return parse_control_properties(config);
    }

}
