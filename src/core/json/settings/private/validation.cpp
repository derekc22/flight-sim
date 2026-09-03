#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>
#include "core/json/settings/private/validation.hpp"

namespace json {

	void validate_fixed_actuator_inputs_json(
	    const nlohmann::json& fixed_actuator_inputs_json)
	{
		if (!fixed_actuator_inputs_json.is_object()) {
			throw std::runtime_error(
			    "json::validate_fixed_actuator_inputs_json: fixed_actuator_inputs must be an object");
		}
	}

	void validate_fixed_control(
	    double cmd,
	    const actuators::Actuator& actuator,
	    const std::string& key)
	{
		if (cmd < actuator.limit_min || cmd > actuator.limit_max) {
			throw std::runtime_error("json::validate_fixed_control: " + key + " command exceeds actuator limits");
		}
	}

	void validate_actuator_settings(
	    const actuators::Settings& actuator_settings,
	    const actuators::ActuatorManager& actuator_manager)
	{
		validate_fixed_control(
		    actuator_settings.fixed_actuator_inputs.flap, actuator_manager.surface_actuators.flap, "flap");
		validate_fixed_control(
		    actuator_settings.fixed_actuator_inputs.spoiler, actuator_manager.surface_actuators.spoiler, "spoiler");
	}

	void validate_actuator_settings_json(
	    const nlohmann::json& config)
	{
		if (config.contains("actuators")) {
			const auto& actuator_settings_json = config.at("actuators");
			if (!actuator_settings_json.is_object()) {
				throw std::runtime_error("json::validate_actuator_settings_json actuators must be an object");
			}
			if (actuator_settings_json.contains("fixed_actuator_inputs")) {
				const auto& fixed_actuator_inputs_json = actuator_settings_json.at("fixed_actuator_inputs");
				if (!fixed_actuator_inputs_json.is_object()) {
					throw std::runtime_error(
					    "json::validate_fixed_actuator_inputs_json: fixed_actuator_inputs must be an object");
				}
			} else {
				throw std::runtime_error(
				    "json::validate_actuator_settings_json fixed_actuator_inputs configuration must be specified");
			}
		} else {
			throw std::runtime_error("json::validate_actuator_settings_json actuators configuration must be specified");
		}
	}

	void validate_avionics_settings_json(
	    const nlohmann::json& config)
	{
		if (config.contains("avionics")) {
			const auto& avionics_settings_json = config.at("avionics");
			if (!avionics_settings_json.is_object()) {
				throw std::runtime_error("json::validate_avionics_settings_json avionics must be an object");
			}
			if (!avionics_settings_json.contains("use_ins")) {
				throw std::runtime_error("json::validate_avionics_settings_json use_ins must be specified");
			}
			if (!avionics_settings_json.at("use_ins").is_boolean()) {
				throw std::runtime_error("json::validate_avionics_settings_json use_ins must be a boolean");
			}
		} else {
			throw std::runtime_error("json::validate_avionics_settings_json avionics configuration must be specified");
		}
	}

} // namespace json
