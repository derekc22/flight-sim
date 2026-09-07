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
