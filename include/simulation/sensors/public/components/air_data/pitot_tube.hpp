#pragma once
#include "simulation/sensors/public/components/base.hpp"
#include "simulation/sensors/public/data/types.hpp"

namespace sensors
{

	struct PitotTube : Sensor {
		std::optional<double> prev_P0_lag;

		/**
		 * @brief Measures stagnation air pressure through the configured sensor model.
		 *
		 * @param[in] P0 Ground-truth stagnation air pressure [Pa].
		 * @param[in] dt Measurement interval [s].
		 * @return Lagged, biased, and noised stagnation air pressure [Pa].
		 */
		StagnationAirPressureMeasurement measure(const atmospheric::StagnationAirPressure& P0, double dt);
	};

} // namespace sensors
