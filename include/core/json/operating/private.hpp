#pragma once
#include <string>
#include <nlohmann/json.hpp>
#include "core/json/operating/public.hpp"
#include "simulation/actuators/actuator/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/operating/public.hpp"

namespace json {

    void validate_fixed_actuator_inputs_json(const nlohmann::json& fixed_actuator_inputs_json);
    void validate_fixed_control(double cmd, const actuators::Actuator& actuator, const std::string& key);
    void validate_operating_properties(const operating::OperatingProperties& operating_properties, const actuators::ActuatorProperties& actuator_properties);
    operating::OperatingProperties parse_operating_properties(const nlohmann::json& config);
}
