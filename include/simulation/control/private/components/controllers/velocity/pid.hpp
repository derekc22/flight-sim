#pragma once
#include "simulation/control/private/components/policies/pid/policy.hpp"
#include "simulation/control/public/data/types.hpp"

namespace control
{

	struct VelocityPID {
		PIDPolicy policy;

		VelocityPID(const VelocityPIDParameters& params);

		/**
		 * @brief Computes the virtual force command for forward-velocity control.
		 *
		 * @param[in] input Current state, velocity setpoint, and previous allocation residual.
		 * @param[in] dt Integration step [s].
		 * @return Virtual control with commanded body x-force [N] and zero remaining components.
		 */
		VirtualControlOutput_T<double> step(const VelocityControlInput& input, double dt);

		/**
		 * @brief Builds the PID-policy input for forward body velocity.
		 *
		 * @param[in] input Current state, velocity setpoint, and allocation residual.
		 * @return PID-policy input for the body x-velocity channel.
		 */
		PIDPolicyInput make_pid_policy_input(const VelocityControlInput& input);
	};
} // namespace control
