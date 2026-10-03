#include "simulation/actuators/public/components/base.hpp"

#include "simulation/constants/public/scalars.hpp"
#include "simulation/util/public/math.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <unsupported/Eigen/MatrixFunctions>

namespace actuators
{

	Actuator::Actuator(
		double limit_max,
		double limit_min,
		double wn,
		double zeta)
		: limit_max(limit_max), limit_min(limit_min), wn(wn), zeta(zeta)
	{
	}

	double Actuator::step(
		double cmd,
		double dt)
	{
		double cmd_clamped = util::clamp(cmd, limit_min, limit_max);
		double init_val = util::clamp(0.0, limit_min, limit_max);
		double val = val_state ? val_state.value() : init_val;
		double rate = val_state ? rate_state : 0.0;

		double err = val - cmd_clamped;

		// assume cmd is constant during the timestep, otherwise err_rate != rate
		Eigen::Vector2d err_state(err, rate);

		Eigen::Matrix2d A;
		A << 0.0, 1.0, -wn * wn, -2.0 * zeta * wn;

		Eigen::Matrix2d Phi = (A * dt).exp();

		err_state = Phi * err_state;
		val = cmd_clamped + err_state(0);
		rate = err_state(1);

		if (val <= limit_min) {
			val = limit_min;
			if (rate < 0.0) {
				rate = 0.0;
			}
		}
		if (val >= limit_max) {
			val = limit_max;
			if (rate > 0.0) {
				rate = 0.0;
			}
		}

		val_state = val;
		rate_state = rate;
		return val;
	}

} // namespace actuators
