#pragma once
#include "simulation/sensors/public/components/base.hpp"
#include "simulation/sensors/public/data/types.hpp"

namespace sensors
{

	struct AngleOfAttackVane : Sensor {
		std::optional<double> prev_alpha_lag;
		AngleOfAttackMeasurement measure(const aerodynamics::AngleOfAttack& alpha, double dt);
	};

} // namespace sensors
