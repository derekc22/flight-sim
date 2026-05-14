#pragma once
#include <nlohmann/json.hpp>
#include "simulation/control/public.hpp"

namespace json {

    control::ControlProperties parse_control_config();

    control::ControlType fetch_control_type(const nlohmann::json& control_config);
}
