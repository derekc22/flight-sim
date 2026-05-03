#include <stdexcept>
#include <string>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "core/json/control.hpp"
#include "core/json/json.hpp"
#include "simulation/constants/constants.hpp"

namespace json {

    template <typename ControlLawType, typename ControlLawClass, typename ControlLawParametersType, typename ControlLawInputType>
    ControlLawClass make_stateful_control_law(const ControlLawParametersType& params) {
        return [controller = ControlLawType(params)](const ControlLawInputType& input) mutable {
            return controller.step(input);
        };
    }


    control::AxialControlLaw make_axial_control_law(control::ControlType control_type, const control::ControlLawParameters& params) {
        switch (control_type) {
            case control::ControlType::AxialPID:
                return make_stateful_control_law
                    <
                        control::AxialPID, 
                        control::AxialControlLaw, 
                        control::AxialControlLawParameters, 
                        control::AxialControlLawInput
                    >
                    (params);
                break;

            case control::ControlType::DamperPID:
                return make_stateful_control_law
                    <
                        control::DamperPID, 
                        control::AxialControlLaw, 
                        control::AxialControlLawParameters, 
                        control::AxialControlLawInput
                    >
                    (params);
                break;

            default:
                throw std::runtime_error("json::make_axial_control_law unknown control type");
        }
    }

    control::VelocityControlLaw make_velocity_control_law(control::ControlType control_type, const control::ControlLawParameters& params) {
        switch (control_type) {
            case control::ControlType::VelocityPID:
                return make_stateful_control_law
                    <
                        control::VelocityPID, 
                        control::VelocityControlLaw, 
                        control::VelocityControlLawParameters, 
                        control::VelocityControlLawInput
                    >
                    (params);
                break;

            default:
                throw std::runtime_error("json::make_velocity_control_law unknown control type");
        }
    }

    control::LinearQuadraticControlLaw make_linear_quadratic_control_law(control::ControlType control_type, const control::ControlLawParameters& params) {
        switch (control_type) {
            case control::ControlType::LinearQuadraticRegulator:
                return make_stateful_control_law
                    <
                        control::LinearQuadraticRegulator, 
                        control::LinearQuadraticControlLaw, 
                        control::LinearQuadraticControlLawParameters, 
                        control::LinearQuadraticControlLawInput
                    >
                    (params);
                break;

            case control::ControlType::LinearQuadraticIntegral:
                return make_stateful_control_law
                    <
                        control::LinearQuadraticIntegrator, 
                        control::LinearQuadraticControlLaw, 
                        control::LinearQuadraticControlLawParameters, 
                        control::LinearQuadraticControlLawInput
                    >
                    (params);
                break;

            case control::ControlType::LinearQuadraticTracker:
                return make_stateful_control_law
                    <
                        control::LinearQuadraticTracker, 
                        control::LinearQuadraticControlLaw, 
                        control::LinearQuadraticControlLawParameters, 
                        control::LinearQuadraticControlLawInput
                    >
                    (params);
                break;
            
            default:
                throw std::runtime_error("json::make_linear_quadratic_control_law unknown control type");
        }
    }

    control::NonlinearControlLaw make_nonlinear_control_law(control::ControlType control_type, const control::ControlLawParameters& params) {
        switch (control_type) {
            case control::ControlType::FeedbackLinearization:
                return make_stateful_control_law
                    <
                        control::FeedbackLinearization, 
                        control::NonlinearControlLaw, 
                        control::NonlinearControlLawParameters, 
                        control::NonlinearControlLawInput
                    >
                    (params);
                break;

            case control::ControlType::NonlinearDynamicInversion:
                return make_stateful_control_law
                    <
                        control::NonlinearDynamicInversion, 
                        control::NonlinearControlLaw, 
                        control::NonlinearControlLawParameters, 
                        control::NonlinearControlLawInput
                    >
                    (params);
                break;

            case control::ControlType::IncrementalNonlinearDynamicInversion:
                return make_stateful_control_law
                    <
                        control::IncrementalNonlinearDynamicInversion, 
                        control::NonlinearControlLaw, 
                        control::NonlinearControlLawParameters, 
                        control::NonlinearControlLawInput
                    >
                    (params);
                break;
            
            default:
                throw std::runtime_error("json::make_nonlinear_control_law unknown control type");
        }
    }

    control::AxialControlLawParameters parse_axial_control_law_parameters(const nlohmann::json& controller_json, control::ControlType& control_type) {
        const auto& parameters_json = controller_json.at("parameters");
        if (!parameters.is_object()) { throw std::runtime_error("json::parse_axial_control_law_parameters expected parameters object"); }
        control::AxialControlLawParameters params;

        switch (control_type) {
            case control::ControlType::DamperPID:
                if (!parameters.contains("Kp_lateral") || !parameters.contains("Kp_longitudinal") || !parameters.contains("Kp_vertical")) {
                    throw std::runtime_error("json::parse_axial_control_law_parameters DamperPID requires Kp_lateral, Kp_longitudinal, Kp_vertical");
                }
                params = control::AxialPIDParamters{ 
                    .Kp_lateral = parameters_json.at("Kp_lateral").get<double>(),
                    .Kp_longitudinal = parameters_json.at("Kp_longitudinal").get<double>(),
                    .Kp_vertical = parameters_json.at("Kp_vertical").get<double>(),
                };
                break;

            case control::ControlType::AxialPID:
                if (
                    !parameters.contains("Kp_lateral") || !parameters.contains("Kp_longitudinal") || !parameters.contains("Kp_vertical") ||
                    !parameters.contains("Ki_lateral") || !parameters.contains("Ki_longitudinal") || !parameters.contains("Ki_vertical") ||
                    !parameters.contains("Kd_lateral") || !parameters.contains("Kd_longitudinal") || !parameters.contains("Kd_vertical")
                ) {
                    throw std::runtime_error("json::parse_axial_control_law_parameters AxialPID requires Kp, Kd, and Ki for the lateral, longitudinal, and vertical axes");
                }
                if (!parameters.contains("tau_lateral") || !parameters.contains("tau_longitudinal") || !parameters.contains("tau_vertical")) {
                    throw std::runtime_error("json::parse_axial_control_law_parameters AxialPID requires tau for the lateral, longitudinal, and vertical axes");
                }
                if (parameters_json.at("tau_lateral").get<double>() < 0.0 || parameters_json.at("tau_longitudinal").get<double>() < 0.0 || parameters_json.at("tau_vertical").get<double>() < 0.0) {
                    throw std::runtime_error("json::parse_axial_control_law_parameters AxialPID requires non-negative tau");
                }
                params = control::AxialPIDParamters{ 
                    .Kp_lateral = parameters_json.at("Kp_lateral").get<double>(),
                    .Ki_longitudinal = parameters_json.at("Ki_longitudinal").get<double>(),
                    .Kd_vertical = parameters_json.at("Kd_vertical").get<double>(),

                    .Kp_lateral = parameters_json.at("Kp_lateral").get<double>(),
                    .Ki_longitudinal = parameters_json.at("Ki_longitudinal").get<double>(),
                    .Kd_vertical = parameters_json.at("Kd_vertical").get<double>(),

                    .Kp_lateral = parameters_json.at("Kp_lateral").get<double>(),
                    .Ki_longitudinal = parameters_json.at("Ki_longitudinal").get<double>(),
                    .Kd_vertical = parameters_json.at("Kd_vertical").get<double>(),

                    .tau_lateral = parameters_json.at("tau_lateral").get<double>(),
                    .tau_longitudinal = parameters_json.at("tau_longitudinal").get<double>(),
                    .tau_vertical = parameters_json.at("tau_vertical").get<double>()
                };
                break;

            default:
                throw std::runtime_error("json::parse_axial_control_law_parameters unknown control type");
        }
        return params;
    }

    control::VelocityControlLawParameters parse_velocity_control_law_parameters(const nlohmann::json& controller_json, control::ControlType& control_type) {
        const auto& parameters_json = controller_json.at("parameters");
        if (!parameters.is_object()) { throw std::runtime_error("json::parse_velocity_control_law_parameters expected parameters object"); }
        control::VelocityControlLawParameters params;

        switch (control_type) {
            case control::ControlType::VelocityPID:
                if (!parameters.contains("Kp") || !parameters.contains("Ki") || !parameters.contains("Kd")) {
                    throw std::runtime_error("json::parse_velocity_control_law_parameters VelocityPID requires Kp, Kd, and Ki");
                }
                if (!parameters.contains("tau")) {
                    throw std::runtime_error("json::parse_velocity_control_law_parameters VelocityPID requires tau");
                }
                if (parameters_json.at("tau").get<double>() < 0.0) {
                    throw std::runtime_error("json::parse_velocity_control_law_parameters VelocityPID requires non-negative tau");
                }
                params = control::VelocityPIDParamters{ 
                    .Kp = parameters_json.at("Kp").get<double>(),
                    .Ki = parameters_json.at("Ki").get<double>(),
                    .Kd = parameters_json.at("Kd").get<double>(),

                    .tau = parameters_json.at("tau").get<double>()
                };
                break;

            default:
                throw std::runtime_error("json::parse_velocity_control_law_parameters unknown control type");
        }
        return params;
    }

    control::LinearQuadraticControlLawParameters parse_linear_quadratic_control_law_parameters(const nlohmann::json& controller_json, control::ControlType& control_type) {
        const auto& parameters_json = controller_json.at("parameters");
        if (!parameters.is_object()) { throw std::runtime_error("json::parse_linear_quadratic_control_law_parameters expected parameters object"); }
        control::LinearQuadraticControlLawParameters params;

        switch (control_type) {
            case control::ControlType::LinearQuadraticRegulator:
                if (!parameters.contains("Q") || !parameters.contains("R") {
                    throw std::runtime_error("json::parse_linear_quadratic_control_law_parameters LinearQuadraticRegulator requires Q and R");
                }
                params = control::LinearQuadraticControlLawParamters{ 
                    .Q = parse_MatrixXd(gains_json.at("Q")),
                    .R = parse_MatrixXd(gains_json.at("R")),
                };
                break;

            case control::ControlType::LinearQuadraticIntegrator:
                if (!parameters.contains("Q") || !parameters.contains("Qi") || !parameters.contains("R") {
                    throw std::runtime_error("json::parse_linear_quadratic_control_law_parameters LinearQuadraticIntegrator requires Q, Qi, and R");
                }
                params = control::LinearQuadraticControlLawParamters{ 
                    .Q = parse_MatrixXd(gains_json.at("Q")),
                    .Qi = parse_MatrixXd(gains_json.at("Qi")),
                    .R = parse_MatrixXd(gains_json.at("R")),
                };
                break;

            default:
                throw std::runtime_error("json::parse_linear_quadratic_control_law_parameters unknown control type");
        }
        return params;
    }

    control::ControlType map_control_type(const std::string& control_type_str) {
        if (control_type_str == "AxialPID") { return control::ControlType::AxialPID; }
        if (control_type_str == "DamperPID") { return control::ControlType::RollDamper; }
        if (control_type_str == "VelocityPID") { return control::ControlType::VelocityPID; }
        if (control_type_str == "LinearQuadraticRegulator") { return control::ControlType::LinearQuadraticRegulator; }
        if (control_type_str == "LinearQuadraticIntegrator") { return control::ControlType::LinearQuadraticIntegrator; }
        if (control_type_str == "LinearQuadraticTracker") { return control::ControlType::LinearQuadraticTracker; }
        if (control_type_str == "FeedbackLinearization") { return control::ControlType::FeedbackLinearization; }
        if (control_type_str == "NonlinearDynamicInversion") { return control::ControlType::NonlinearDynamicInversion; }
        if (control_type_str == "IncrementalNonlinearDynamicInversion") { return control::ControlType::IncrementalNonlinearDynamicInversion; }
        throw std::runtime_error("json::map_control_type unknown control type: " + control_type_str);
    }

    void parse_axial_controller(const nlohmann::json& controller_json, control::AxialControlLaw<double>& control_law) {
        std::string control_type_str = controller_json.at("control_type").get<std::string>();

        control_type = map_control_type(control_type_str);
        control::AxialControlLawParameters params = parse_axial_control_law_parameters(controller_json, control_type);
        control_law = make_axial_control_law<double>(control_type, params); 
    }

    void parse_velocity_controller(const nlohmann::json& controller_json, control::VelocityControlLaw<double>& control_law) {
        std::string control_type_str = controller_json.at("control_type").get<std::string>();

        control_type = map_control_type(control_type_str);
        control::ControlLawParameters params = parse_control_law_parameters(controller_json, control_type);
        control_law = make_velocity_control_law<double>(control_type, params); 
    }

    void parse_linear_quadratic_controller(const nlohmann::json& controller_json, control::LinearQuadraticControlLaw<Eigen::VectorXd>& control_law) {
        std::string control_type_str = controller_json.at("control_type").get<std::string>();

        control_type = map_control_type(control_type_str);
        control::ControlLawParameters params = parse_control_law_parameters(controller_json, control_type);
        control_law = make_linear_quadratic_control_law<Eigen::VectorXd>(control_type, params); 
    }

    void validate_controllers(const nlohmann::json& controllers_json){
        bool axial_bool = controllers_json.contains("axial");
        bool velocity_bool = controllers_json.contains("velocity");
        bool linear_quadratic_bool = controllers_json.contains("linear_quadratic");
        bool nonlinear_bool = controllers_json.contains("nonlinear_bool");

        if (axial_bool && linear_quadratic_bool) { throw std::runtime_error("json::parse_control_properties: axial and linear_quadratic control laws cannot both be present"); }
        if (axial_bool && nonlinear_bool) { throw std::runtime_error("json::parse_control_properties: axial and nonlinear control laws cannot both be present"); }

        if (linear_quadratic_bool && nonlinear_bool) { throw std::runtime_error("json::parse_control_properties: linear_quadratic and nonlinear control laws cannot both be present"); }

        if (velocity_bool && linear_quadratic_bool) { throw std::runtime_error("json::parse_control_properties: velocity and linear_quadratic control laws cannot both be present"); }
        if (velocity_bool && nonlinear_bool) { throw std::runtime_error("json::parse_control_properties: velocity and nonlinear control laws cannot both be present"); }
    }
 
    control::ControlProperties parse_control_properties(const nlohmann::json& config) {
        const auto& controllers_json = config.at("controllers");
        validate_controllers(controller_json);

        const auto& setpoint_json = config.at("setpoint");
        control::ControlProperties control_properties;

        if (axial_bool) {
            const auto& axial_controllers_json = controllers_json.at("axial");
            parse_axial_controller(
                axial_controllers_json,     
                control_properties.axial_control_law
            );
            // control_properties.axial_setpoint = parse_axial_control_setpoint(setpoint_json);
        }

        if (velocity_bool) {
            const auto& velocity_controller_json = controllers_json.at("velocity");
            parse_velocity_controller(
                velocity_controller_json, 
                control_properties.velocity_control_law
            );
            // control_properties.velocity_setpoint = parse_velocity_control_setpoint(setpoint_json, control_properties.velocity_control_type);
        }

        if (linear_quadratic_bool) {
            const auto& linear_quadratic_controller_json = controllers_json.at("linear_quadratic");
            parse_linear_quadratic_controller(
                linear_quadratic_controller_json, 
                control_properties.linear_quadratic_control_law, 
            );
            // control_properties.linear_quadratic_setpoint = parse_linear_quadratic_control_setpoint(setpoint_json, control_properties.linear_quadratic_control_type);
        }
        return control_properties;
    }

    control::ControlProperties parse_control_config() {
        const auto config_path = resolve_run_config_entry_path("control_config");
        const auto config = read_json_file(config_path);
        return parse_control_properties(config);
    }

}
