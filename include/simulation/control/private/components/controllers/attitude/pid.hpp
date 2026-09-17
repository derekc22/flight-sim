#pragma once
#include "simulation/control/private/components/policies/pid/policy.hpp"
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

		/**
		 * @brief Computes the virtual moment command for attitude control.
		 *
		 * @param[in] input Current state, attitude setpoint, and previous allocation residual.
		 * @param[in] dt Integration step [s].
		 * @return Virtual control with zero force and commanded body moments [N m].
		 */
		VirtualControlOutput_T<double> step(const AttitudeControlInput& input, double dt);

		/**
		 * @brief Builds the PI-D policy input for one attitude axis.
		 *
		 * The selected axis maps to roll, pitch, or yaw angle, its measured body
		 * angular rate, and its corresponding previous-step moment-allocation residual.
		 *
		 * @param[in] input Current state, attitude setpoint, and allocation residual.
		 * @param[in] axis Attitude axis to map into a scalar policy input.
		 * @return PI-D policy input for the selected attitude axis.
		 */
		virtual PIDPolicyInput make_pid_policy_input(const AttitudeControlInput& input, AttitudeAxis axis);
	};

} // namespace control
