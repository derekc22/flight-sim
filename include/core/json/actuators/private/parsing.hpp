#pragma once
#include "simulation/actuators/public/data/helpers.hpp"
#include "simulation/actuators/public/manager.hpp"

#include <nlohmann/json.hpp>
#include <string>

namespace json
{

	template <typename SurfaceActuatorType>
	SurfaceActuatorType parse_surface_actuator(const nlohmann::json& config, const std::string& key);

	template <typename PropulsorActuatorType>
	PropulsorActuatorType parse_propulsor_actuator(const nlohmann::json& config, const std::string& key);

	actuators::ActuatorManager parse_actuator_manager(const nlohmann::json& config);
} // namespace json

#include "core/json/actuators/private/parsing.tpp"
