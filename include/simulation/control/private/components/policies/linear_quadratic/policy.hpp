#pragma once
#include "simulation/control/private/data/types.hpp"
#include "simulation/control/public/data/types.hpp"

namespace control
{

	struct LinearQuadraticPolicy {
		LinearQuadraticPolicyParameters params;

		LinearQuadraticPolicy(const LinearQuadraticPolicyParameters& params);

		/**
		 * @brief Computes the virtual-control deviation for a state deviation.
		 *
		 * If no gain matrix was provided, the policy solves the continuous-time
		 * algebraic Riccati equation and caches the resulting gain before applying
		 * the feedback law.
		 *
		 * @param[in] input State deviation and virtual-system matrices.
		 * @return Virtual-control deviation from the negative state-feedback law.
		 */
		VirtualControlOutputVector_T<double> step(const LinearQuadraticPolicyInput& input);
	};
} // namespace control
