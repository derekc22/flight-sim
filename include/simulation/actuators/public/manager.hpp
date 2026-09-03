#pragma once
#include "simulation/actuators/public/components/collection.hpp"
#include "simulation/actuators/public/data/types.hpp"

namespace actuators {

	struct ActuatorManager {
		SurfaceActuators surface_actuators;
		PropulsorActuators propulsor_actuators;
		Settings settings;

		ActuatorManagerOutput step(const ActuatorManagerInput& input);
		SurfaceActuatorInputs_T<double> step_surfaces(const SurfaceActuatorInputs_T<double>& u_cmd, double dt);
		PropulsorActuatorInputs_T<double> step_propulsors(const PropulsorActuatorInputs_T<double>& u_cmd, double dt);
	};

} // namespace actuators
