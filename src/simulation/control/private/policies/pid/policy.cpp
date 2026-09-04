#include "simulation/control/private/policies/pid/policy.hpp"

#include "simulation/constants/public/scalars.hpp"
#include "simulation/util/public/filters.hpp"

namespace control
{

	PIDPolicy::PIDPolicy(
		const PIDPolicyParameters& params)
		: params(params)
	{
	}

	double PIDPolicy::step(
		const PIDPolicyInput& input,
		double dt)
	{
		double err = input.x_des - input.x;

		double d_term = input.x_dot.has_value() ? input.x_dot.value()	 // PI-D
												: (prev_err - err) / dt; // PID

		// filtered derivative
		d_filtered = util::first_order_lag(d_term, d_filtered, params.tau, dt);

		// integral candidate
		double integral_new = integral + err * dt;

		// unsaturated control
		double u = params.Kp * err - params.Kd * d_filtered + params.Ki * integral_new;

		// anti-windup
		if (!((input.delta_mu < -constants::eps && err > 0.0) || (input.delta_mu > constants::eps && err < 0.0))) {
			integral = integral_new;
		}

		prev_err = err;

		return u;
	}

} // namespace control
