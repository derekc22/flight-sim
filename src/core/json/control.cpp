#include <stdexcept>
#include <string>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "core/json/control.hpp"
#include "core/json/json.hpp"
#include "simulation/constants/constants.hpp"

namespace json {

    control::AxisControlSetpoint parse_axis_control_setpoint(const nlohmann::json& setpoint_json, const control::ControlType& longitudinal_control_type, const control::ControlType& lateral_control_type, const control::ControlType& vertical_control_type) {
        control::AxisControlSetpoint axis_ctrl_setpoint;

        if (longitudinal_control_type == control::ControlType::PitchDamper || lateral_control_type == control::ControlType::RollDamper || vertical_control_type == control::ControlType::YawDamper) {
            if (setpoint_json.contains("w")) { axis_ctrl_setpoint.wB_BI = dynamics::AngularVelocity{ parse_Vector3d(setpoint_json.at("w")) }; }
            else { throw std::runtime_error("json::parse_axis_control_setpoint: Damper controller requires w"); }
        }

        if (longitudinal_control_type == control::ControlType::PitchPIDController || lateral_control_type == control::ControlType::RollPIDController || vertical_control_type == control::ControlType::YawPIDController) {
            if (setpoint_json.contains("eul")) { axis_ctrl_setpoint.eulIB = dynamics::EulerAngles{ parse_Vector3d(setpoint_json.at("eul")) }; }
            else { throw std::runtime_error("json::parse_axis_control_setpoint: PIDController requires eul"); }
        }
        return axis_ctrl_setpoint;
    }

    control::VelocityControlSetpoint parse_velocity_control_setpoint(const nlohmann::json& setpoint_json, const control::ControlType& veloctiy_control_type) {
        control::VelocityControlSetpoint velocity_ctrl_setpoint;

        if (veloctiy_control_type == control::ControlType::VelocityPIDController) {
            if (setpoint_json.contains("v")) { velocity_ctrl_setpoint.vB_BI = dynamics::LinearVelocity{ parse_Vector3d(setpoint_json.at("v")) }; }
            else { throw std::runtime_error("json::parse_velocity_control_setpoint: VelocityControl controller requires v"); }
        }
        return velocity_ctrl_setpoint;
    }

    control::FullStateControlSetpoint parse_full_state_control_setpoint(const nlohmann::json& setpoint_json, const control::ControlType& full_state_control_type) {
        control::FullStateControlSetpoint full_state_ctrl_setpoint;

        if (full_state_control_type == control::ControlType::LinearQuadraticRegulator || full_state_control_type == control::ControlType::LinearQuadraticTracker) {
            if (setpoint_json.contains("v")) { full_state_ctrl_setpoint.vB_BI = dynamics::LinearVelocity{ parse_Vector3d(setpoint_json.at("v")) }; }
            else { throw std::runtime_error("json::parse_full_state_control_setpoint: LinearQuadratic controller requires v"); }

            if (setpoint_json.contains("w")) { full_state_ctrl_setpoint.wB_BI = dynamics::AngularVelocity{ parse_Vector3d(setpoint_json.at("w")) }; }
            else { throw std::runtime_error("json::parse_full_state_control_setpoint: LinearQuadratic controller requires w"); }

            if (setpoint_json.contains("eul")) { full_state_ctrl_setpoint.eulIB = dynamics::EulerAngles{ parse_Vector3d(setpoint_json.at("eul")) }; }
            else { throw std::runtime_error("json::parse_full_state_control_setpoint: LinearQuadratic controller requires eul"); }
        }
        return full_state_ctrl_setpoint;
    }

    void validate_control_law_tau(const nlohmann::json& parameters_json, const control::ControlType& control_type) {

        switch (control_type) {
            case control::ControlType::PitchPIDController:
            case control::ControlType::RollPIDController:
            case control::ControlType::YawPIDController:
            case control::ControlType::VelocityPIDController:
                if (!parameters_json.contains("tau")) { throw std::runtime_error("json::validate_control_law_tau missing tau"); }
                if (parameters_json.at("tau").get<double>() < 0.0) { throw std::runtime_error("json::validate_control_law_tau tau must be non-negative"); }
                return;
            default:
                if (parameters_json.contains("tau")) { throw std::runtime_error("json::validate_control_law_tau controller does not require tau"); }
                return;
        }
    }

    void validate_control_law_gains(const nlohmann::json& gains_json, control::ControlType& control_type) {
        if (!gains_json.is_object()) { throw std::runtime_error("json::validate_control_law_gains expected gains object"); }

        switch (control_type) {
            case control::ControlType::PitchPIDController:
            case control::ControlType::RollPIDController:
            case control::ControlType::YawPIDController:
            case control::ControlType::VelocityPIDController:
                if (!gains_json.contains("Kp") || !gains_json.contains("Kd") || !gains_json.contains("Ki")) {
                    throw std::runtime_error("json::validate_control_law_gains PID control requires Kp, Kd, Ki");
                }
                break;

            case control::ControlType::PitchDamper:
            case control::ControlType::RollDamper:
            case control::ControlType::YawDamper:
                if (!gains_json.contains("Kp")) {
                    throw std::runtime_error("json::validate_control_law_gains damper control requires Kp");
                }
                break;

            case control::ControlType::LinearQuadraticRegulator:
            case control::ControlType::LinearQuadraticTracker:
                if (!gains_json.contains("Q") || !gains_json.contains("R")) {
                    throw std::runtime_error("json::validate_control_law_gains LQ control requires Q and R");
                }
                break;
            default:
                break;
        }
    }

    void validate_control_law_parameters(const nlohmann::json& parameters_json, const nlohmann::json& gains_json, control::ControlType& control_type) {
        validate_control_law_tau(parameters_json, control_type);
        validate_control_law_gains(gains_json, control_type);
    }

    control::ControlLawGains parse_control_gains(const nlohmann::json& gains_json, control::ControlType control_type) {
        switch (control_type) {
            case control::ControlType::PitchPIDController:
            case control::ControlType::RollPIDController:
            case control::ControlType::YawPIDController:
            case control::ControlType::VelocityPIDController:
                return {
                    .Kp = gains_json.at("Kp").get<double>(),
                    .Kd = gains_json.at("Kd").get<double>(),
                    .Ki = gains_json.at("Ki").get<double>(),
                };

            case control::ControlType::PitchDamper:
            case control::ControlType::RollDamper:
            case control::ControlType::YawDamper:
                return {
                    .Kp = gains_json.at("Kp").get<double>(),
                };

            case control::ControlType::LinearQuadraticRegulator:
            case control::ControlType::LinearQuadraticTracker:
                return {
                    .Q = parse_MatrixXd(gains_json.at("Q")),
                    .R = parse_MatrixXd(gains_json.at("R")),
                };

            default:
                throw std::runtime_error("json::parse_control_gains unknown control type");
        }
    }

    template <typename ControlType, typename ControlLawCommand>
    control::AxisControlLaw<ControlLawCommand> make_stateful_axis_control_law(const control::ControlLawParameters& params) {
        return [controller = ControlType(params)](const control::PIDControlLawInput& ctrl_law_input) mutable {
            return controller._step(ctrl_law_input);
        };
    }

    template <typename ControlType, typename ControlLawCommand>
    control::VelocityControlLaw<ControlLawCommand> make_stateful_velocity_control_law(const control::ControlLawParameters& params) {
        return [controller = ControlType(params)](const control::PIDControlLawInput& ctrl_law_input) mutable {
            return controller._step(ctrl_law_input);
        };
    }

    template <typename ControlType, typename ControlLawCommand>
    control::FullStateControlLaw<ControlLawCommand> make_stateful_full_state_control_law(const control::ControlLawParameters& params) {
        return [controller = ControlType(params)](const control::FullStateControlLawInput& ctrl_law_input) mutable {
            return controller._step(ctrl_law_input);
        };
    }

    template <typename ControlLawCommand>
    control::AxisControlLaw<ControlLawCommand> make_axis_control_law(control::ControlType control_type, const control::ControlLawParameters& params) {
        switch (control_type) {
            case control::ControlType::PitchPIDController:
                return make_stateful_axis_control_law<control::PitchPIDController, double>(params);

            case control::ControlType::PitchDamper:
                return make_stateful_axis_control_law<control::PitchDamper, double>(params);

            case control::ControlType::RollPIDController:
                return make_stateful_axis_control_law<control::RollPIDController, double>(params);

            case control::ControlType::RollDamper:
                return make_stateful_axis_control_law<control::RollDamper, double>(params);

            case control::ControlType::YawPIDController:
                return make_stateful_axis_control_law<control::YawPIDController, double>(params);

            case control::ControlType::YawDamper:
                return make_stateful_axis_control_law<control::YawDamper, double>(params);

            default:
                throw std::runtime_error("json::make_axis_control_law unknown control type");
        }
    }

    template <typename ControlLawCommand>
    control::VelocityControlLaw<ControlLawCommand> make_velocity_control_law(control::ControlType control_type, const control::ControlLawParameters& params) {
        switch (control_type) {
            case control::ControlType::VelocityPIDController:
                return make_stateful_velocity_control_law<control::VelocityPIDController, double>(params);
            default:
                throw std::runtime_error("json::make_velocity_control_law unknown control type");
        }
    }

    template <typename ControlLawCommand>
    control::FullStateControlLaw<ControlLawCommand> make_full_state_control_law(control::ControlType control_type, const control::ControlLawParameters& params) {
        switch (control_type) {
            case control::ControlType::LinearQuadraticRegulator:
                return make_stateful_full_state_control_law<control::LinearQuadraticRegulator, Eigen::VectorXd>(params);

            case control::ControlType::LinearQuadraticTracker:
                return make_stateful_full_state_control_law<control::LinearQuadraticTracker, Eigen::VectorXd>(params);
            
            default:
                throw std::runtime_error("json::make_full_state_control_law unknown control type");
        }
    }

    control::ControlLawParameters parse_control_law_parameters(const nlohmann::json& controller_json, control::ControlType& control_type) {
        const auto& parameters_json = controller_json.at("parameters");
        const auto& gains_json = parameters_json.at("gains");

        validate_control_law_parameters(parameters_json, gains_json, control_type);

        control::ControlLawGains gains = parse_control_gains(gains_json, control_type);
        double tau = parameters_json.contains("tau") ? parameters_json.at("tau").get<double>() : 0.0;

        return { .gains = gains, .tau = tau };
    }

    control::ControlType map_control_type(const std::string& control_type_str) {
        if (control_type_str == "PitchPIDController") { return control::ControlType::PitchPIDController; }
        if (control_type_str == "PitchDamper") { return control::ControlType::PitchDamper; }
        if (control_type_str == "RollPIDController") { return control::ControlType::RollPIDController; }
        if (control_type_str == "RollDamper") { return control::ControlType::RollDamper; }
        if (control_type_str == "YawPIDController") { return control::ControlType::YawPIDController; }
        if (control_type_str == "YawDamper") { return control::ControlType::YawDamper; }
        if (control_type_str == "LinearQuadraticRegulator") { return control::ControlType::LinearQuadraticRegulator; }
        if (control_type_str == "LinearQuadraticTracker") { return control::ControlType::LinearQuadraticTracker; }
        if (control_type_str == "VelocityPIDController") { return control::ControlType::VelocityPIDController; }
        throw std::runtime_error("json::map_control_type unknown control type: " + control_type_str);
    }

    void parse_axis_controller(const nlohmann::json& controllers_json, const std::string& key, control::ControlType& control_type, control::AxisControlLaw<double>& control_law) {
        if (!controllers_json.contains(key)) { return; }
        const auto& controller_json = controllers_json.at(key);
        std::string control_type_str = controller_json.at("control_type").get<std::string>();

        control_type = map_control_type(control_type_str);
        control::ControlLawParameters control_law_parameters = parse_control_law_parameters(controller_json, control_type);
        control_law = make_axis_control_law<double>(control_type, control_law_parameters); 
    }

    void parse_velocity_controller(const nlohmann::json& controller_json, control::ControlType& control_type, control::VelocityControlLaw<double>& control_law) {
        std::string control_type_str = controller_json.at("control_type").get<std::string>();

        control_type = map_control_type(control_type_str);
        control::ControlLawParameters control_law_parameters = parse_control_law_parameters(controller_json, control_type);
        control_law = make_velocity_control_law<double>(control_type, control_law_parameters); 
    }

    void parse_full_state_controller(const nlohmann::json& controller_json, control::ControlType& control_type, control::FullStateControlLaw<Eigen::VectorXd>& control_law) {
        std::string control_type_str = controller_json.at("control_type").get<std::string>();

        control_type = map_control_type(control_type_str);
        control::ControlLawParameters control_law_parameters = parse_control_law_parameters(controller_json, control_type);
        control_law = make_full_state_control_law<Eigen::VectorXd>(control_type, control_law_parameters); 
    }
 
    control::ControlProperties parse_control_properties(const nlohmann::json& config) {
        const auto& controllers_json = config.at("controllers");
        bool axial_bool = controllers_json.contains("axial");
        bool velocity_bool = controllers_json.contains("velocity");
        bool full_state_bool = controllers_json.contains("full_state");
        if (axial_bool && full_state_bool) { throw std::runtime_error("json::parse_control_properties: axial and full state control laws cannot both be present"); }
        if (velocity_bool && full_state_bool) { throw std::runtime_error("json::parse_control_properties: velocity and full state control laws cannot both be present"); }

        const auto& setpoint_json = config.at("setpoint");
        control::ControlProperties control_properties;

        if (axial_bool) {
            const auto& axial_controllers_json = controllers_json.at("axial");
            parse_axis_controller(
                axial_controllers_json, 
                "longitudinal", 
                control_properties.longitudinal_control_type, 
                control_properties.longitudinal_controller
            );
            parse_axis_controller(
                axial_controllers_json, 
                "lateral", 
                control_properties.lateral_control_type, 
                control_properties.lateral_controller
            );
            parse_axis_controller(
                axial_controllers_json, 
                "vertical", 
                control_properties.vertical_control_type, 
                control_properties.vertical_controller
            );

            control_properties.axis_setpoint = parse_axis_control_setpoint(
                setpoint_json, 
                control_properties.longitudinal_control_type, 
                control_properties.lateral_control_type, 
                control_properties.vertical_control_type
            );
        }

        if (velocity_bool) {
            const auto& velocity_controller_json = controllers_json.at("velocity");
            parse_velocity_controller(
                velocity_controller_json, 
                control_properties.velocity_control_type, 
                control_properties.velocity_controller
            );

            control_properties.velocity_setpoint = parse_velocity_control_setpoint(setpoint_json, control_properties.velocity_control_type);
        }

        if (full_state_bool) {
            const auto& full_state_controller_json = controllers_json.at("full_state");
            parse_full_state_controller(
                full_state_controller_json, 
                control_properties.full_state_control_type, 
                control_properties.full_state_controller
            );

            control_properties.full_state_setpoint = parse_full_state_control_setpoint(setpoint_json, control_properties.full_state_control_type);
        }

        return control_properties;
    }

    control::ControlProperties parse_control_config() {
        const auto config_path = resolve_run_config_entry_path("control_config");
        const auto config = read_json_file(config_path);
        return parse_control_properties(config);
    }

}
