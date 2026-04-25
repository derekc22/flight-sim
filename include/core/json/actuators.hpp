#pragma once
#include "simulation/actuators/actuators.hpp"
#include "simulation/structural/structural.hpp"

namespace json {

    actuators::ActuatorProperties parse_actuator_config(structural::StructuralProperties& structural_properties);

}
