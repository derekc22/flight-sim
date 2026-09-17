#pragma once
#include "simulation/sensors/public/components/base.hpp"
#include "simulation/sensors/public/data/types.hpp"

namespace sensors
{

	struct TotalAirTemperatureProbe : Sensor {
		std::optional<double> prev_T0_lag;

		/**
		 * @brief Measures stagnation air temperature through the configured sensor model.
		 *
		 * @param[in] T0 Ground-truth stagnation air temperature [K].
		 * @param[in] dt Measurement interval [s].
		 * @return Lagged, biased, and noised stagnation air temperature [K].
		 */
		StagnationAirTemperatureMeasurement measure(const atmospheric::StagnationAirTemperature& T0, double dt);
	};

} // namespace sensors
