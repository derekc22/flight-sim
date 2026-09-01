#pragma once
#include "simulation/control/public/manager.hpp"
#include "simulation/guidance/public/manager.hpp"

namespace json {

    guidance::GuidanceManager parse_guidance_config(const control::ControlManager& control_manager);

}
