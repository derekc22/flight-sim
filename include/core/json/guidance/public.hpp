#pragma once
#include "simulation/control/public.hpp"
#include "simulation/guidance/public.hpp"

namespace json {

    guidance::GuidanceProperties parse_guidance_config(const control::ControlProperties& control_properties);

}
