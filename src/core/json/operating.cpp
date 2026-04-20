#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>
#include "core/json/operating.hpp"
#include "core/json/json.hpp"
#include "simulation/util/util.hpp"

namespace json {

    static void validate_fixed_controls_json(const nlohmann::json& fixed_controls_json) {
        if (!fixed_controls_json.is_object()) { throw std::runtime_error("json::validate_fixed_controls_json: fixed_controls must be an object"); }
    }

    static void validate_fixed_control(double cmd, const actuators::Actuator& actuator, const std::string& key) {
        if (cmd < actuator.limit_min || cmd > actuator.limit_max) {
            throw std::runtime_error("json::validate_fixed_control: " + key + " command exceeds actuator limits");
        }
    }

    static void validate_operating_properties(const operating::OperatingProperties& operating_properties, const actuators::ActuatorProperties& actuator_properties) {
        validate_fixed_control(operating_properties.fixed_controls.flap, actuator_properties.surface_actuators.flap, "flap");
        validate_fixed_control(operating_properties.fixed_controls.spoiler, actuator_properties.surface_actuators.spoiler, "spoiler");
    }

    static operating::OperatingProperties parse_operating_properties(const nlohmann::json& config) {
        operating::OperatingProperties operating_properties;
        if (!config.contains("fixed_controls")) { return operating_properties; }

        const auto& fixed_controls_json = config.at("fixed_controls");
        validate_fixed_controls_json(fixed_controls_json);

        if (fixed_controls_json.contains("flap")) {
            operating_properties.fixed_controls.flap = util::deg_to_rad(fixed_controls_json.at("flap").get<double>());
        }

        if (fixed_controls_json.contains("spoiler")) {
            operating_properties.fixed_controls.spoiler = util::deg_to_rad(fixed_controls_json.at("spoiler").get<double>());
        }

        return operating_properties;
    }

    operating::OperatingProperties parse_operating_config(const actuators::ActuatorProperties& actuator_properties) {
        const auto config_path = resolve_run_config_entry_path("operating_config");
        const auto config = read_json_file(config_path);
        const operating::OperatingProperties operating_properties = parse_operating_properties(config);
        validate_operating_properties(operating_properties, actuator_properties);
        return operating_properties;
    }

}
