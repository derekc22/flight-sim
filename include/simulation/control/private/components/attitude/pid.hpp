#pragma once
#include "simulation/control/private/policies/pid/policy.hpp"
#include "simulation/control/public/data/types.hpp"

namespace control
{

	struct AttitudePID {
		enum AttitudeAxis {
			Longitudinal,
			Lateral,
			Vertical
		};

		PIDPolicy lateral_policy;
		PIDPolicy longitudinal_policy;
		PIDPolicy vertical_policy;

		AttitudePID(const AttitudePIDParameters& params);
		VirtualControlOutput_T<double> step(const AttitudeControlInput& input, double dt);

		virtual PIDPolicyInput make_pid_policy_input(const AttitudeControlInput& input, AttitudeAxis axis);
	};

} // namespace control
