#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>
#include "core/json/operating.hpp"
#include "core/json/json.hpp"
#include "simulation/util/util.hpp"

namespace json {

    void validate_fixed_surface_actuator_inputs_json(const nlohmann::json& fixed_surface_actuator_inputs_json) {
        if (!fixed_surface_actuator_inputs_json.is_object()) { throw std::runtime_error("json::validate_fixed_surface_actuator_inputs_json: fixed_surface_actuator_inputs must be an object"); }
    }

    void validate_fixed_control(double cmd, const actuators::Actuator& actuator, const std::string& key) {
        if (cmd < actuator.limit_min || cmd > actuator.limit_max) {
            throw std::runtime_error("json::validate_fixed_control: " + key + " command exceeds actuator limits");
        }
    }

    void validate_operating_properties(const operating::OperatingProperties& operating_properties, const actuators::ActuatorProperties& actuator_properties) {
        validate_fixed_control(operating_properties.fixed_surface_actuator_inputs.flap, actuator_properties.surface_actuators.flap, "flap");
        validate_fixed_control(operating_properties.fixed_surface_actuator_inputs.spoiler, actuator_properties.surface_actuators.spoiler, "spoiler");
    }

    operating::OperatingProperties parse_operating_properties(const nlohmann::json& config) {
        operating::OperatingProperties operating_properties;
        if (!config.contains("fixed_surface_actuator_inputs")) { return operating_properties; }

        const auto& fixed_surface_actuator_inputs_json = config.at("fixed_surface_actuator_inputs");
        validate_fixed_surface_actuator_inputs_json(fixed_surface_actuator_inputs_json);

        if (fixed_surface_actuator_inputs_json.contains("flap")) {
            operating_properties.fixed_surface_actuator_inputs.flap = util::deg_to_rad(fixed_surface_actuator_inputs_json.at("flap").get<double>());
        }

        if (fixed_surface_actuator_inputs_json.contains("spoiler")) {
            operating_properties.fixed_surface_actuator_inputs.spoiler = util::deg_to_rad(fixed_surface_actuator_inputs_json.at("spoiler").get<double>());
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
