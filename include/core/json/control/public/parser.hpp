#pragma once
#include "simulation/control/public/manager.hpp"

namespace json
{

	control::ControlManager parse_control_config(bool trim_flag);
}
