#include <stdexcept>
#include <string>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "core/json/control.hpp"
#include "core/json/json.hpp"
#include "simulation/constants/constants.hpp"

namespace json {    

    control::AxialControlSetpoint parse_axial_control_setpoint(const nlohmann::json& setpoint_json, const control::ControlType& longitudinal_control_type, const control::ControlType& lateral_control_type, const control::ControlType& vertical_control_type) {
        control::AxialControlSetpoint axial_ctrl_setpoint;

        if (longitudinal_control_type == control::ControlType::PitchDamper || lateral_control_type == control::ControlType::RollDamper || vertical_control_type == control::ControlType::YawDamper) {
            if (setpoint_json.contains("w")) { axial_ctrl_setpoint.wB_BI = dynamics::AngularVelocity{ parse_Vector3d(setpoint_json.at("w")) }; }
            else { throw std::runtime_error("json::parse_axial_control_setpoint: Damper controller requires w"); }
        }

        if (longitudinal_control_type == control::ControlType::PitchPIDControlLaw || lateral_control_type == control::ControlType::RollPIDControlLaw || vertical_control_type == control::ControlType::YawPIDControlLaw) {
            if (setpoint_json.contains("eul")) { axial_ctrl_setpoint.eulIB = dynamics::EulerAngles{ parse_Vector3d(setpoint_json.at("eul")) }; }
            else { throw std::runtime_error("json::parse_axial_control_setpoint: PIDControlLaw requires eul"); }
        }
        return axial_ctrl_setpoint;
    }

    control::VelocityControlSetpoint parse_velocity_control_setpoint(const nlohmann::json& setpoint_json, const control::ControlType& veloctiy_control_type) {
        control::VelocityControlSetpoint velocity_ctrl_setpoint;

        if (veloctiy_control_type == control::ControlType::VelocityPID) {
            if (setpoint_json.contains("v")) { velocity_ctrl_setpoint.vB_BI = dynamics::LinearVelocity{ parse_Vector3d(setpoint_json.at("v")) }; }
            else { throw std::runtime_error("json::parse_velocity_control_setpoint: VelocityControl controller requires v"); }
        }
        return velocity_ctrl_setpoint;
    }

    control::FullStateControlSetpoint parse_linear_quadratic_control_setpoint(const nlohmann::json& setpoint_json, const control::ControlType& linear_quadratic_control_type) {
        control::FullStateControlSetpoint linear_quadratic_ctrl_setpoint;

        if (linear_quadratic_control_type == control::ControlType::LinearQuadraticRegulator || linear_quadratic_control_type == control::ControlType::LinearQuadraticTracker) {
            if (setpoint_json.contains("v")) { linear_quadratic_ctrl_setpoint.vB_BI = dynamics::LinearVelocity{ parse_Vector3d(setpoint_json.at("v")) }; }
            else { throw std::runtime_error("json::parse_linear_quadratic_control_setpoint: LinearQuadratic controller requires v"); }

            if (setpoint_json.contains("w")) { linear_quadratic_ctrl_setpoint.wB_BI = dynamics::AngularVelocity{ parse_Vector3d(setpoint_json.at("w")) }; }
            else { throw std::runtime_error("json::parse_linear_quadratic_control_setpoint: LinearQuadratic controller requires w"); }

            if (setpoint_json.contains("eul")) { linear_quadratic_ctrl_setpoint.eulIB = dynamics::EulerAngles{ parse_Vector3d(setpoint_json.at("eul")) }; }
            else { throw std::runtime_error("json::parse_linear_quadratic_control_setpoint: LinearQuadratic controller requires eul"); }
        }
        return linear_quadratic_ctrl_setpoint;
    }
}