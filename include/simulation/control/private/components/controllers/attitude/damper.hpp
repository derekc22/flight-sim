#pragma once

#include "simulation/control/private/components/controllers/attitude/pid.hpp"

namespace control
{

	struct DamperPID : AttitudePID {
		using AttitudePID::AttitudePID;

		/**
		 * @brief Builds the PID-policy input for a body-axis angular-rate damper.
		 *
		 * @param[in] input Current state, angular-rate setpoint, and allocation residual.
		 * @param[in] axis Attitude axis to map into a scalar policy input.
		 * @return PID-policy input for the selected body axis.
		 */
		PIDPolicyInput make_pid_policy_input(const AttitudeControlInput& input, AttitudeAxis axis) override;
	};
} // namespace control
