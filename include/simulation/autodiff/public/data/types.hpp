#pragma once
#include "simulation/actuators/public/components/collection.hpp"
#include "simulation/actuators/public/data/types.hpp"
#include "simulation/aerodynamics/public/manager.hpp"
#include "simulation/propulsion/public/manager.hpp"
#include "simulation/structural/public/data/types.hpp"

namespace autodiff
{

	struct AutoDiffModel {
		structural::StructuralState struc_t;
		aerodynamics::AerodynamicsManager& aerodynamic;
		actuators::PropulsorActuators& propulsor_actuators;
		propulsion::PropulsionManager& propulsion;
		actuators::ActuatorLimits actuator_limits;
		actuators::FixedActuatorInputs fixed_actuator_inputs;
	};

} // namespace autodiff
