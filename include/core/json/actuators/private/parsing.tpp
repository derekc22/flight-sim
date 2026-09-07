#pragma once
#include "core/json/actuators/private/validation.hpp"
#include "simulation/actuators/public/data/helpers.hpp"
#include "simulation/actuators/public/manager.hpp"

#include <nlohmann/json.hpp>
#include <string>

namespace json
{

	template <typename SurfaceActuatorType>
	SurfaceActuatorType parse_surface_actuator(
		const nlohmann::json& config,
		const std::string& key)
	{
		if (config.at(key).is_null()) {
			return SurfaceActuatorType{};
		}

		const auto& surface_actuator_json = config.at(key);
		validate_actuator_json(surface_actuator_json);

		return SurfaceActuatorType(surface_actuator_json.at("limit_max").get<double>(),
			surface_actuator_json.at("limit_min").get<double>(),
			surface_actuator_json.at("tau").get<double>());
	}

	template <typename PropulsorActuatorType>
	PropulsorActuatorType parse_propulsor_actuator(
		const nlohmann::json& config,
		const std::string& key)
	{
		if (config.at(key).is_null()) {
			return PropulsorActuatorType{};
		}

		const auto& propulsor_actuator_json = config.at(key);
		validate_propulsor_actuator_json(propulsor_actuator_json);

		return PropulsorActuatorType(propulsor_actuator_json.at("limit_max").get<double>(),
			propulsor_actuator_json.at("limit_min").get<double>(),
			propulsor_actuator_json.at("tau").get<double>());
	}
} // namespace json
