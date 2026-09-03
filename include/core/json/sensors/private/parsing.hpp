#pragma once
#include <string>
#include <nlohmann/json.hpp>
#include "simulation/sensors/public/manager.hpp"

namespace json {

	template <typename SensorType> SensorType parse_sensor(const nlohmann::json& config, const std::string& key);

	sensors::SensorManager parse_sensor_manager(const nlohmann::json& config);

} // namespace json

#include "core/json/sensors/private/parsing.tpp"
