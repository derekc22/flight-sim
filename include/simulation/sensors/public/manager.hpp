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
		 * Moves the prior @c hist value into @c prev_hist, stores the newly produced
		 * measurements in @c hist, and records the measurement interval in @c sensor_dt.
		 *
		 * @param[in] input Ground-truth sensor quantities and measurement interval.
		 * @return Current modeled sensor measurements.
		 */
		SensorManagerOutput step(const SensorManagerInput& input);
	};

} // namespace sensors
