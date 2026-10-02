#pragma once
#include "simulation/actuators/public/data/types.hpp"

#include <optional>

namespace actuators
{

	struct Actuator {
		double limit_max;
		double limit_min;
		double wn;
		double zeta;
		std::optional<double> val_state;
		double rate_state = 0.0;

		/**
		 * @brief Advances the actuator response for one time step.
		 *
		 * @param[in] cmd Requested actuator command in actuator command units.
		 * @param[in] dt Time step [s].
		 * @return Lagged actuator command in actuator command units.
		 */
		double step(double cmd, double dt);

		Actuator(double limit_max, double limit_min, double wn, double zeta);
	};

	struct SurfaceActuator : Actuator {
		using Actuator::Actuator;
	};

	struct PropulsorActuator : Actuator {
		using Actuator::Actuator;
	};

} // namespace actuators
