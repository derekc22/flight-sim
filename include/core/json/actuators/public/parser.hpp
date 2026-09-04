#pragma once
#include "simulation/actuators/public/data/helpers.hpp"
#include "simulation/actuators/public/manager.hpp"
#include "simulation/structural/public/manager.hpp"

namespace json
{

	actuators::ActuatorManager parse_actuator_config(structural::StructuralManager& structural_manager);

}
