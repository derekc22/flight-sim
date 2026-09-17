#pragma once
#include "simulation/control/public/data/types.hpp"

namespace control
{

	struct AttitudeControl {
		AttitudeControlImplementation implementation;

		AttitudeControl(const AttitudePIDParameters& params);

		AttitudeControl(const DamperPIDParameters& params);

		/**
		 * @brief Advances the attitude controller and identifies its controlled channels.
		 *
		 * @param[in] input Current state, attitude setpoint, and previous allocation residual.
		 * @param[in] dt Controller step [s].
		 * @return Moment command and masks for the active virtual-control and actuator channels.
		 */
		ControlComponentOutput step(const AttitudeControlInput& input, double dt);
	};

} // namespace control
