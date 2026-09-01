#pragma once
#include <nlohmann/json.hpp>

namespace json {

    void validate_attitude_heading_reference_system_json(const nlohmann::json& ahrs_json);

}
