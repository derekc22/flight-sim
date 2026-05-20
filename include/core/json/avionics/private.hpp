#pragma once
#include <string>
#include <nlohmann/json.hpp>
#include "core/json/avionics/public.hpp"
#include "simulation/avionics/public.hpp"

namespace json {

    void validate_sensor_json(const nlohmann::json& sensor_json);

    template <typename SensorType>
    SensorType parse_sensor(const nlohmann::json& config, const std::string& key);

    avionics::AvionicsProperties parse_avionics_properties(const nlohmann::json& config);
}

#include "core/json/avionics/private.tpp"
