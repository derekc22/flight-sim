#pragma once
#include "simulation/control/control.hpp"
#include "simulation/guidance/guidance.hpp"

namespace json {

    guidance::GuidanceProperties parse_guidance_config(const control::ControlProperties& control_properties);

}
