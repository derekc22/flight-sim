#pragma once
#include "simulation/propulsion/public/manager.hpp"
#include "simulation/structural/public/manager.hpp"

namespace json
{

	propulsion::PropulsionManager parse_propulsion_config(structural::StructuralManager& structural_manager);

}
