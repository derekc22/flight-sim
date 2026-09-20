#pragma once
#include "simulation/control/public/data/types.hpp"

namespace control
{

	struct VelocityControl {
		VelocityControlImplementation implementation;

		VelocityControl(const VelocityPIDParameters& params);

		/**
		 * @brief Advances the velocity controller and identifies its controlled channels.
		 *
		 * @param[in] input Current state, velocity setpoint, and previous allocation residual.
		 * @param[in] dt Controller step [s].
		 * @return Body x-force command and masks for the active virtual-control and actuator channels.
		 */
		ControlComponentOutput step(const VelocityControlInput& input, double dt);
	};

} // namespace control
