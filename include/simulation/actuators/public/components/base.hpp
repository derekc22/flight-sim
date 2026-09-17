#pragma once
#include "simulation/actuators/public/data/types.hpp"

#include <optional>

namespace actuators
{

	struct Actuator {
		double limit_max;
		double limit_min;
		double tau;
		std::optional<double> lag_state;

		/**
		 * @brief Advances the actuator response for one time step.
		 *
		 * Clamps @p cmd to the actuator limits, applies the configured first-order lag, stores the resulting lag state,
		 * and returns it. An uninitialized lag state starts from zero clamped to the actuator limits.
		 *
		 * @param[in] cmd Requested actuator command in actuator command units.
		 * @param[in] dt Time step [s].
		 * @return Lagged actuator command in actuator command units.
		 */
		double step(double cmd, double dt);

		Actuator(double limit_max, double limit_min, double tau);

		Actuator();
	};

	struct SurfaceActuator : Actuator {
		using Actuator::Actuator;
	};

	struct PropulsorActuator : Actuator {
		using Actuator::Actuator;
	};

} // namespace actuators
