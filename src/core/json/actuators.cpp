#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>
#include "core/json/actuators.hpp"
#include "core/json/json.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/util/util.hpp"

namespace json {

    void validate_actuator_json(const nlohmann::json& actuator_json) {
        if (!actuator_json.contains("limit_max")) { throw std::runtime_error("json::validate_actuator_json: actuator maximum limit not present"); }
        if (!actuator_json.contains("limit_min")) { throw std::runtime_error("json::validate_actuator_json: actuator minimum limit not present"); }
        if (!actuator_json.contains("tau")) { throw std::runtime_error("json::validate_actuator_json: actuator tau not present"); }

        const double limit_max = actuator_json.at("limit_max").get<double>();
        const double limit_min = actuator_json.at("limit_min").get<double>();
        const double tau = actuator_json.at("tau").get<double>();

        if (limit_max < limit_min) { throw std::runtime_error("json::validate_actuator_json: actuator maximum limit must be greater than or equal to minimum limit"); }
        if (tau < 0.0) { throw std::runtime_error("json::validate_actuator_json: actuator tau must be non-negative"); }
    }

    void validate_propulsor_actuator_json(const nlohmann::json& propulsor_actuator_json, const std::string& key) {
        validate_actuator_json(propulsor_actuator_json);
        if (!propulsor_actuator_json.contains("inclination_angle")) { throw std::runtime_error("json::validate_propulsor_actuator_json: propulsor inclination_angle not present"); }
        if (!propulsor_actuator_json.contains("toe_angle")) { throw std::runtime_error("json::validate_propulsor_actuator_json: propulsor toe_angle not present"); }
        if (!propulsor_actuator_json.contains("pB_prop_cg")) { throw std::runtime_error("json::validate_propulsor_actuator_json: propulsor pB_prop_cg not present"); }

        Eigen::Vector3d prop_pos = parse_Vector3d(propulsor_actuator_json.at("pB_prop_cg"));
        if (key == "front_propulsor"){
            if (prop_pos(1) != 0.0) { throw std::runtime_error("json::validate_propulsor_actuator_json: front propulsor must have pB_prop_cg[1] = 0"); }
        }
        if (key == "left_propulsor"){
            if (prop_pos(1) > 0.0) { throw std::runtime_error("json::validate_propulsor_actuator_json: left propulsor must have pB_prop_cg[1] <= 0"); }
        }
        if (key == "right_propulsor"){
            if (prop_pos(1) < 0.0) { throw std::runtime_error("json::validate_propulsor_actuator_json: right propulsor must have pB_prop_cg[1] >= 0"); }
        }
    }

    template <typename SurfaceActuatorType>
    SurfaceActuatorType parse_surface_actuator(const nlohmann::json& config, const std::string& key) {
        const auto& actuator_json = config.at(key);
        validate_actuator_json(actuator_json);

        return SurfaceActuatorType(
            util::deg_to_rad(actuator_json.at("limit_max").get<double>()),
            util::deg_to_rad(actuator_json.at("limit_min").get<double>()),
            actuator_json.at("tau").get<double>()
        );
    }

    template <typename PropulsorActuatorType>
    PropulsorActuatorType parse_propulsor_actuator(const nlohmann::json& config, const std::string& key) {
        const auto& actuator_json = config.at(key);
        validate_propulsor_actuator_json(actuator_json, key);

        return PropulsorActuatorType(
            actuator_json.at("limit_max").get<double>(),
            actuator_json.at("limit_min").get<double>(),
            actuator_json.at("tau").get<double>(),
            util::deg_to_rad(actuator_json.at("inclination_angle").get<double>()),
            util::deg_to_rad(actuator_json.at("toe_angle").get<double>()),
            parse_Vector3d(actuator_json.at("pB_prop_cg"))
        );
    }


    actuators::ActuatorProperties parse_actuator_properties(const nlohmann::json& config) {
        const auto& surface_actuator_json = config.at("surfaces");
        const auto& propulsor_actuator_json = config.at("propulsors");

        actuators::SurfaceActuators surface_actuators = {
            .elevator = parse_surface_actuator<actuators::Elevator>(surface_actuator_json, "elevator"),
            .aileron = parse_surface_actuator<actuators::Aileron>(surface_actuator_json, "aileron"),
            .rudder = parse_surface_actuator<actuators::Rudder>(surface_actuator_json, "rudder"),
            .flap = parse_surface_actuator<actuators::Flap>(surface_actuator_json, "flap"),
            .spoiler = parse_surface_actuator<actuators::Spoiler>(surface_actuator_json, "spoiler")
        };

        actuators::PropulsorActuators propulsor_actuators = {
            .front_propulsor = parse_propulsor_actuator<actuators::FrontPropulsor>(propulsor_actuator_json, "front_propulsor"),
            .left_propulsor = parse_propulsor_actuator<actuators::LeftPropulsor>(propulsor_actuator_json, "left_propulsor"),
            .right_propulsor = parse_propulsor_actuator<actuators::RightPropulsor>(propulsor_actuator_json, "right_propulsor"),
        };

        return { .surface_actuators = surface_actuators, .propulsor_actuators = propulsor_actuators };
    }

    actuators::ActuatorProperties parse_actuator_config() {
        const auto config_path = resolve_run_config_entry_path("actuator_config");
        const auto config = read_json_file(config_path);
        return parse_actuator_properties(config);
    }

}
