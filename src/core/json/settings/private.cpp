#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>
#include "core/json/settings/private.hpp"
#include "simulation/actuators/actuator/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/settings/public.hpp"

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

    void validate_actuator_settings(const actuators::Settings& actuator_settings, const actuators::ActuatorProperties& actuator_properties) {
        validate_fixed_control(
            actuator_settings.fixed_actuator_inputs.flap, 
            actuator_properties.surface_actuators.flap, 
            "flap"
        );
        validate_fixed_control(
            actuator_settings.fixed_actuator_inputs.spoiler, 
            actuator_properties.surface_actuators.spoiler, 
            "spoiler"
        );
    }

    void validate_actuator_settings_json(const nlohmann::json& config) {
        if (config.contains("actuators")) {
            const auto& actuator_settings_json = config.at("actuators");
            if (!actuator_settings_json.is_object()) { 
                throw std::runtime_error("json::parse_settings_manager actuators must be an object"); 
            }
            if (actuator_settings_json.contains("fixed_actuator_inputs")) { 
                const auto& fixed_actuator_inputs_json = actuator_settings_json.at("fixed_actuator_inputs");
                if (!fixed_actuator_inputs_json.is_object()) { 
                    throw std::runtime_error("json::validate_fixed_actuator_inputs_json: fixed_actuator_inputs must be an object"); 
                }
            }
            else { 
                throw std::runtime_error("json::parse_settings_manager fixed_actuator_inputs configuration must be specified"); 
            }
        }
        else { 
            throw std::runtime_error("json::parse_settings_manager actuators configuration must be specified"); 
        }
    }

    void validate_avionics_settings_json(const nlohmann::json& config) {
        if (config.contains("avionics")) {
            const auto& avionics_settings_json = config.at("avionics");
            if (!avionics_settings_json.is_object()) { 
                throw std::runtime_error("json::parse_settings_manager avionics must be an object"); 
            }
            if (!avionics_settings_json.contains("use_gnss")) { 
                throw std::runtime_error("json::parse_settings_manager use_gnss must be specified"); 
            }
            if (!avionics_settings_json.at("use_gnss").is_boolean()) {
                throw std::runtime_error("json::parse_settings_manager use_gnss must be a boolean");
            }
        }
        else { 
            throw std::runtime_error("json::parse_settings_manager avionics configuration must be specified"); 
        }
    }

    settings::SettingsManager parse_settings(const nlohmann::json& config, const actuators::ActuatorProperties& actuator_properties) {
        settings::SettingsManager settings_manager;

        validate_actuator_settings_json(config);
        const auto& actuator_settings_json = config.at("actuators");
        const auto& fixed_actuator_inputs_json = actuator_settings_json.at("fixed_actuator_inputs");
        if (fixed_actuator_inputs_json.contains("flap")) {
            settings_manager.actuator_settings.fixed_actuator_inputs.flap =
                fixed_actuator_inputs_json.at("flap").get<double>();
        }
        if (fixed_actuator_inputs_json.contains("spoiler")) {
            settings_manager.actuator_settings.fixed_actuator_inputs.spoiler =
                fixed_actuator_inputs_json.at("spoiler").get<double>();
        }
        validate_actuator_settings(settings_manager.actuator_settings, actuator_properties);


        validate_avionics_settings_json(config);
        const auto& avionics_settings_json = config.at("avionics");
        settings_manager.avionics_settings.use_gnss = avionics_settings_json.at("use_gnss").get<bool>();

        return settings_manager;
    }

}
