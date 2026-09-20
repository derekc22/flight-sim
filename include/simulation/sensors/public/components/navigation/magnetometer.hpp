#pragma once
#include "simulation/sensors/public/components/base.hpp"
#include "simulation/sensors/public/data/types.hpp"

namespace sensors
{

	struct Magnetometer : Sensor {
		std::optional<double> prev_heading_lag;

		/**
		 * @brief Measures heading through the configured sensor model.
		 *
		 * @param[in] heading Ground-truth heading [rad].
		 * @param[in] dt Measurement interval [s].
		 * @return Lagged, biased, and noised heading measurement [rad].
		 */
		HeadingMeasurement measure(const geography::Heading& heading, double dt);
	};

} // namespace sensors
