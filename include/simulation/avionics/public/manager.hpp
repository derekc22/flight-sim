#pragma once
#include "simulation/avionics/public/components/collection.hpp"
#include "simulation/avionics/public/data/types.hpp"

#include <optional>

namespace avionics
{

	struct AvionicsManager {
		Avionics avionics;
		std::optional<AvionicsMeasurements> hist;

		/**
		 * @brief Computes the current avionics measurements and updates their history.
		 *
		 * Derives Mach number and static temperature from air-data sensors, advances
		 * INS and AHRS estimates from the previous avionics measurements, and computes
		 * the remaining air-data quantities. On the first call, INS position and
		 * velocity and AHRS orientation are initialized from ground truth. When no
		 * prior sensor measurements are available, vertical speed is also initialized
		 * from ground truth.
		 *
		 * The avionics interval advances the INS and AHRS, while the sensor interval
		 * is used to differentiate consecutive pressure samples.
		 *
		 * @param[in] input Current and previous sensor data, ground truth, and update intervals.
		 * @return Current derived avionics measurements.
		 */
		AvionicsManagerOutput step(const AvionicsManagerInput& input);

		Settings settings;
	};

} // namespace avionics
