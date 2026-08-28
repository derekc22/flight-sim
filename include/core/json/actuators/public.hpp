#pragma once
#include "simulation/actuators/public.hpp"
#include "simulation/structural/public.hpp"

namespace json {

    actuators::ActuatorManager parse_actuator_config(structural::StructuralManager& structural_manager);

}
