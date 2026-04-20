#pragma once
#include "simulation/actuators/actuators.hpp"
#include "simulation/operating/operating.hpp"

namespace json {

    operating::OperatingProperties parse_operating_config(const actuators::ActuatorProperties& actuator_properties);

}
