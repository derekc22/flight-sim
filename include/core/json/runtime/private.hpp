#pragma once
#include <string>
#include <nlohmann/json.hpp>
#include "core/json/runtime/public.hpp"
#include "simulation/actuators/actuator/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/runtime/public.hpp"

namespace json {

    void validate_fixed_actuator_inputs_json(const nlohmann::json& fixed_actuator_inputs_json);
    void validate_fixed_control(double cmd, const actuators::Actuator& actuator, const std::string& key);
    void validate_runtime_actuator_properties(const runtime::RuntimeActuatorProperties& runtime_actuator_properties, const actuators::ActuatorProperties& actuator_properties);
    void validate_runtime_actuators_json(const nlohmann::json& config);
    void validate_runtime_avionics_json(const nlohmann::json& config);
    runtime::RuntimeProperties parse_runtime_properties(const nlohmann::json& config, const actuators::ActuatorProperties& actuator_properties);
}
