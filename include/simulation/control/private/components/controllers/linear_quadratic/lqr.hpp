#pragma once
#include "simulation/control/private/components/policies/linear_quadratic/policy.hpp"
#include "simulation/control/public/data/types.hpp"

#include <Eigen/Dense>

namespace control
{

	struct LinearQuadraticRegulator {
		LinearQuadraticPolicy policy;

		LinearQuadraticRegulator(const LinearQuadraticRegulatorParameters& params);

		/**
		 * @brief Computes the virtual-control deviation from the current state error.
		 *
		 * @param[in] input Current state, setpoint, trim state, and virtual linearization.
		 * @param[in] dt Controller step duration, which is unused by the regulator [s].
		 * @return Virtual-control deviation with force components [N] and moment components [N m].
		 */
		VirtualControlOutput_T<double> step(const LinearQuadraticControlInput& input, double);

		/**
		 * @brief Builds the regulator policy input from the state deviation and virtual linearization.
		 *
		 * @param[in] input Current state, setpoint, and virtual linearization.
		 * @return Linear-quadratic policy input for the regulator.
		 */
		LinearQuadraticPolicyInput make_linear_quadratic_policy_input(const LinearQuadraticControlInput& input);
	};
} // namespace control
