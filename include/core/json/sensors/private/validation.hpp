#pragma once
#include <nlohmann/json.hpp>

namespace json {

    void validate_sensor_json(const nlohmann::json& sensor_json);

}
