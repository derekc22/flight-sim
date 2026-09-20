#pragma once
#include "simulation/sensors/public/components/base.hpp"
#include "simulation/sensors/public/data/types.hpp"

namespace sensors
{

	struct StaticPort : Sensor {
		std::optional<double> prev_P_lag;

		/**
		 * @brief Measures static air pressure through the configured sensor model.
		 *
		 * @param[in] P Ground-truth static air pressure [Pa].
		 * @param[in] dt Measurement interval [s].
		 * @return Lagged, biased, and noised static air pressure [Pa].
		 */
		StaticAirPressureMeasurement measure(const atmospheric::StaticAirPressure& P, double dt);
	};

} // namespace sensors
