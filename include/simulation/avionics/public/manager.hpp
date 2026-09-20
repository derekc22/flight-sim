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
		 * @param[in] input Current and previous sensor data, ground truth, and update intervals.
		 * @return Current derived avionics measurements.
		 */
		AvionicsManagerOutput step(const AvionicsManagerInput& input);

		Settings settings;
	};

} // namespace avionics
