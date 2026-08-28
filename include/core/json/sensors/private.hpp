#pragma once
#include <string>
#include <nlohmann/json.hpp>
#include "simulation/sensors/public.hpp"

namespace json {

    void validate_sensor_json(const nlohmann::json& sensor_json);

    template <typename SensorType>
    SensorType parse_sensor(const nlohmann::json& config, const std::string& key);

    sensors::SensorManager parse_sensor_manager(const nlohmann::json& config);
}

#include "core/json/sensors/private.tpp"
