#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>
#include "core/json/runtime/private.hpp"
#include "simulation/actuators/actuator/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/runtime/public.hpp"
#include "simulation/util/public.hpp"

namespace json {

    void validate_fixed_actuator_inputs_json(const nlohmann::json& fixed_actuator_inputs_json) {
        if (!fixed_actuator_inputs_json.is_object()) { 
            throw std::runtime_error("json::validate_fixed_actuator_inputs_json: fixed_actuator_inputs must be an object"); 
        }
    }

    void validate_fixed_control(double cmd, const actuators::Actuator& actuator, const std::string& key) {
        if (cmd < actuator.limit_min || cmd > actuator.limit_max) {
            throw std::runtime_error("json::validate_fixed_control: " + key + " command exceeds actuator limits");
        }
    }

    void validate_runtime_actuator_properties(const runtime::RuntimeActuatorProperties& runtime_actuator_properties, const actuators::ActuatorProperties& actuator_properties) {
        validate_fixed_control(
            runtime_actuator_properties.fixed_actuator_inputs.flap, 
            actuator_properties.surface_actuators.flap, 
            "flap"
        );
        validate_fixed_control(
            runtime_actuator_properties.fixed_actuator_inputs.spoiler, 
            actuator_properties.surface_actuators.spoiler, 
            "spoiler"
        );
    }

    void validate_runtime_actuators_json(const nlohmann::json& config) {
        if (config.contains("actuators")) {
            const auto& runtime_actuators_json = config.at("actuators");
            if (!runtime_actuators_json.is_object()) { 
                throw std::runtime_error("json::parse_runtime_properties actuators must be an object"); 
            }
            if (runtime_actuators_json.contains("fixed_actuator_inputs")) { 
                const auto& fixed_actuator_inputs_json = runtime_actuators_json.at("fixed_actuator_inputs");
                if (!fixed_actuator_inputs_json.is_object()) { 
                    throw std::runtime_error("json::validate_fixed_actuator_inputs_json: fixed_actuator_inputs must be an object"); 
                }
            }
            else { 
                throw std::runtime_error("json::parse_runtime_properties fixed_actuator_inputs configuration must be specified"); 
            }
        }
        else { 
            throw std::runtime_error("json::parse_runtime_properties actuators configuration must be specified"); 
        }
    }

    void validate_runtime_avionics_json(const nlohmann::json& config) {
        if (config.contains("avionics")) {
            const auto& runtime_avionics_json = config.at("avionics");
            if (!runtime_avionics_json.is_object()) { 
                throw std::runtime_error("json::parse_runtime_properties avionics must be an object"); 
            }
            if (!runtime_avionics_json.contains("use_gnss")) { 
                throw std::runtime_error("json::parse_runtime_properties use_gnss must be specified"); 
            }
            if (!runtime_avionics_json.at("use_gnss").is_boolean()) {
                throw std::runtime_error("json::parse_runtime_properties use_gnss must be a boolean");
            }
        }
        else { 
            throw std::runtime_error("json::parse_runtime_properties avionics configuration must be specified"); 
        }
    }

    runtime::RuntimeProperties parse_runtime_properties(const nlohmann::json& config, const actuators::ActuatorProperties& actuator_properties) {
        runtime::RuntimeProperties runtime_properties;

        validate_runtime_actuators_json(config);
        const auto& runtime_actuators_json = config.at("actuators");
        const auto& fixed_actuator_inputs_json = runtime_actuators_json.at("fixed_actuator_inputs");
        if (fixed_actuator_inputs_json.contains("flap")) {
            runtime_properties.runtime_actuator_properties.fixed_actuator_inputs.flap = util::deg_to_rad(
                fixed_actuator_inputs_json.at("flap").get<double>()
            );
        }
        if (fixed_actuator_inputs_json.contains("spoiler")) {
            runtime_properties.runtime_actuator_properties.fixed_actuator_inputs.spoiler = util::deg_to_rad(
                fixed_actuator_inputs_json.at("spoiler").get<double>()
            );
        }
        validate_runtime_actuator_properties(runtime_properties.runtime_actuator_properties, actuator_properties);


        validate_runtime_avionics_json(config);
        const auto& runtime_avionics_json = config.at("avionics");
        runtime_properties.runtime_avionics_properties.use_gnss = runtime_avionics_json.at("use_gnss").get<bool>();

        return runtime_properties;
    }

}
