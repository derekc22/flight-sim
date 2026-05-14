#pragma once
#include "simulation/actuators/public.hpp"
#include "simulation/structural/public.hpp"

namespace json {

    actuators::ActuatorProperties parse_actuator_config(structural::StructuralProperties& structural_properties);

}
