
#include "simulation/control/private/components/controllers/attitude/damper.hpp"

#include "simulation/dynamics/public/data/types.hpp"

#include <stdexcept>

namespace control
{

	PIDPolicyInput DamperPID::make_pid_policy_input(
		const AttitudeControlInput& input,
		AttitudeAxis axis)
	{
		dynamics::RigidBodyState Zt = input.Zt;

		// All 3 attitude axes omit x_dot. Thus, PID policy runs as PID (as opposed to PI-D)
		switch (axis) {
			case AttitudeAxis::Lateral:
				return {.x = Zt.w.p(), .x_des = input.setpoint.wB_BI.p(), .delta_mu = input.delta_mu_vec_t_1[3]};

			case AttitudeAxis::Longitudinal:
				return {.x = Zt.w.q(), .x_des = input.setpoint.wB_BI.q(), .delta_mu = input.delta_mu_vec_t_1[4]};

			case AttitudeAxis::Vertical:
				return {.x = Zt.w.r(), .x_des = input.setpoint.wB_BI.r(), .delta_mu = input.delta_mu_vec_t_1[5]};

			default:
				throw std::runtime_error("control::make_pid_policy_input invalid control axis");
		}
	}

} // namespace control
