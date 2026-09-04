#pragma once
#include "simulation/actuators/public/components/base.hpp"
#include "simulation/actuators/public/manager.hpp"

#include <nlohmann/json.hpp>
#include <string>

namespace json
{

	void validate_fixed_actuator_inputs_json(const nlohmann::json& fixed_actuator_inputs_json);
	void validate_fixed_control(double cmd, const actuators::Actuator& actuator, const std::string& key);
	void validate_actuator_settings(const actuators::Settings& actuator_settings,
		const actuators::ActuatorManager& actuator_manager);
	void validate_actuator_settings_json(const nlohmann::json& config);
	void validate_avionics_settings_json(const nlohmann::json& config);

} // namespace json
