#pragma once
#include "simulation/control/private/data/types.hpp"

namespace control
{

	struct PIDPolicy {
		PIDPolicyParameters params;
		double integral = 0.0;
		double d_filtered = 0.0;
		double prev_err = 0.0;

		PIDPolicy(const PIDPolicyParameters& params);

		/**
		 * @brief Advances the PID policy and returns its control command.
		 *
		 * @param[in] input Current value, desired value, optional derivative, and allocation residual.
		 * @param[in] dt Integration and derivative step [s].
		 * @return Scalar control command in the units implied by the configured gains.
		 */
		double step(const PIDPolicyInput& input, double dt);
	};
} // namespace control
