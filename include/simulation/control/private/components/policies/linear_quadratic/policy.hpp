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
		 * @param[in] input State deviation and virtual-system matrices.
		 * @return Virtual-control deviation from the negative state-feedback law.
		 */
		VirtualControlOutputVector_T<double> step(const LinearQuadraticPolicyInput& input);
	};
} // namespace control
