#pragma once
#include "simulation/sensors/public/components/collection.hpp"
#include "simulation/sensors/public/data/types.hpp"

#include <optional>

namespace sensors
{

	struct SensorManager {
		Sensors sensors;
		std::optional<SensorMeasurements> hist;
		std::optional<SensorMeasurements> prev_hist;
		double sensor_dt = 0.0;

		/**
		 * @brief Samples every sensor and updates the current and previous measurement sets.
		 *
		 * @param[in] input Ground-truth sensor quantities and measurement interval.
		 * @return Current modeled sensor measurements.
		 */
		SensorManagerOutput step(const SensorManagerInput& input);
	};

} // namespace sensors
