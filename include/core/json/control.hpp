#pragma once
#include <iostream>
#include "simulation/control/control.hpp"

namespace json {

    control::ControlProperties parse_control_config();

    control::ControlType fetch_control_type(const nlohmann::json& control_config);
}
