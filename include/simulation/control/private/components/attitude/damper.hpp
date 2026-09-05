#pragma once

#include "simulation/control/private/components/attitude/pid.hpp"

namespace control
{

	struct DamperPID : AttitudePID {
		using AttitudePID::AttitudePID;

		PIDPolicyInput make_pid_policy_input(const AttitudeControlInput& input, AttitudeAxis axis) override;
	};
} // namespace control
