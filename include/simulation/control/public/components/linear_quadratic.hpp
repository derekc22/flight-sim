#pragma once
#include "simulation/control/public/data/types.hpp"

namespace control
{

	struct LinearQuadraticControl {
		LinearQuadraticControlImplementation implementation;

		LinearQuadraticControl(const LinearQuadraticRegulatorParameters& params);

		LinearQuadraticControl(const LinearQuadraticIntegratorParameters& params);

		/**
		 * @brief Advances the linear-quadratic controller and marks all control channels active.
		 *
		 * @param[in] input Current state, setpoint, trim state, virtual linearization, and allocation residual.
		 * @param[in] dt Controller step [s].
		 * @return Virtual wrench command with all virtual-control and actuator channels active.
		 */
		ControlComponentOutput step(const LinearQuadraticControlInput& input, double dt);
	};

} // namespace control
