#pragma once
#include "simulation/control/private/components/policies/linear_quadratic/policy.hpp"
#include "simulation/control/public/data/types.hpp"

#include <Eigen/Dense>

namespace control
{

	struct LinearQuadraticRegulator {
		LinearQuadraticPolicy policy;

		LinearQuadraticRegulator(const LinearQuadraticRegulatorParameters& params);
		VirtualControlOutput_T<double> step(const LinearQuadraticControlInput& input, double);

		LinearQuadraticPolicyInput make_linear_quadratic_policy_input(const LinearQuadraticControlInput& input);
	};
} // namespace control
