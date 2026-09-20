#pragma once
#include "simulation/sensors/public/components/base.hpp"
#include "simulation/sensors/public/data/types.hpp"

namespace sensors
{

	struct AngleOfAttackVane : Sensor {
		std::optional<double> prev_alpha_lag;

		/**
		 * @brief Measures angle of attack through the configured sensor model.
		 *
		 * @param[in] alpha Ground-truth angle of attack [rad].
		 * @param[in] dt Measurement interval [s].
		 * @return Lagged, biased, and noised angle-of-attack measurement [rad].
		 */
		AngleOfAttackMeasurement measure(const aerodynamics::AngleOfAttack& alpha, double dt);
	};

} // namespace sensors
