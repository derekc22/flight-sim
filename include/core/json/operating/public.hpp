#pragma once
#include "simulation/actuators/public.hpp"
#include "simulation/operating/public.hpp"

namespace json {

    operating::OperatingProperties parse_operating_config(const actuators::ActuatorProperties& actuator_properties);

}
