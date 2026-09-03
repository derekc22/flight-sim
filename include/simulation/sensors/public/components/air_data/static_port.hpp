#pragma once
#include "simulation/sensors/public/components/base.hpp"
#include "simulation/sensors/public/data/types.hpp"

namespace sensors {

	struct StaticPort : Sensor {
		std::optional<double> prev_P_lag;
		StaticAirPressureMeasurement measure(const atmospheric::StaticAirPressure& P, double dt);
	};

} // namespace sensors
