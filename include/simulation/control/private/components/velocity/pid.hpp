#pragma once
#include "simulation/control/private/policies/pid/policy.hpp"
#include "simulation/control/public/data/types.hpp"

namespace control {

	struct VelocityPID {
		PIDPolicy policy;

		VelocityPID(const VelocityPIDParameters& params);
		VirtualControlOutput_T<double> step(const VelocityControlInput& input, double dt);

		PIDPolicyInput make_pid_policy_input(const VelocityControlInput& input);
	};
} // namespace control
