#pragma once
#include "simulation/actuators/public/components/collection.hpp"
#include "simulation/actuators/public/data/types.hpp"

namespace actuators
{

	struct ActuatorManager {
		SurfaceActuators surface_actuators;
		PropulsorActuators propulsor_actuators;
		Settings settings;

		/**
		 * @brief Advances all actuators for one simulation step.
		 *
		 * Replaces the requested flap and spoiler commands with the configured fixed inputs before stepping the surface
		 * and propulsor actuators.
		 *
		 * @param[in] input Requested actuator commands and time step.
		 * @return Applied commands and lagged actuator outputs.
		 */
		ActuatorManagerOutput step(const ActuatorManagerInput& input);

		/**
		 * @brief Advances all surface actuators for one time step.
		 *
		 * @param[in] u_cmd Requested surface-actuator commands in actuator command units.
		 * @param[in] dt Time step [s].
		 * @return Lagged surface-actuator commands in actuator command units.
		 */
		SurfaceActuatorInputs_T<double> step_surfaces(const SurfaceActuatorInputs_T<double>& u_cmd, double dt);

		/**
		 * @brief Advances all propulsor actuators for one time step.
		 *
		 * @param[in] u_cmd Requested propulsor-actuator commands in actuator command units.
		 * @param[in] dt Time step [s].
		 * @return Lagged propulsor-actuator commands in actuator command units.
		 */
		PropulsorActuatorInputs_T<double> step_propulsors(const PropulsorActuatorInputs_T<double>& u_cmd, double dt);
	};

} // namespace actuators
