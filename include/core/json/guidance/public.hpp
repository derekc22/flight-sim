#pragma once
#include "simulation/control/public.hpp"
#include "simulation/guidance/public.hpp"

namespace json {

    guidance::GuidanceManager parse_guidance_config(const control::ControlManager& control_manager);

}
