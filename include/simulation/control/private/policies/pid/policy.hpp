#pragma once
#include "simulation/control/private/data/types.hpp"

namespace control {

	struct PIDPolicy {
		PIDPolicyParameters params;
		double integral = 0.0;
		double d_filtered = 0.0;
		double prev_err = 0.0;

		PIDPolicy(const PIDPolicyParameters& params);
		double step(const PIDPolicyInput& input, double dt);
	};
} // namespace control
