#pragma once
#include <string>
#include <nlohmann/json.hpp>
#include "core/json/settings/public.hpp"
#include "simulation/actuators/actuator/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/settings/public.hpp"

namespace json {

    void validate_fixed_actuator_inputs_json(const nlohmann::json& fixed_actuator_inputs_json);
    void validate_fixed_control(double cmd, const actuators::Actuator& actuator, const std::string& key);
    void validate_actuator_settings(const actuators::Settings& actuator_settings, const actuators::ActuatorProperties& actuator_properties);
    void validate_actuator_settings_json(const nlohmann::json& config);
    void validate_avionics_settings_json(const nlohmann::json& config);
    settings::SettingsManager parse_settings(const nlohmann::json& config, const actuators::ActuatorProperties& actuator_properties);
}
