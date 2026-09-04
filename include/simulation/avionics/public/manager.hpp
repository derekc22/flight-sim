#pragma once
#include "simulation/avionics/public/components/collection.hpp"
#include "simulation/avionics/public/data/types.hpp"

#include <optional>

namespace avionics
{

	struct AvionicsManager {
		Avionics avionics;
		std::optional<AvionicsMeasurements> hist;

		AvionicsManagerOutput step(const AvionicsManagerInput& input);

		Settings settings;
	};

} // namespace avionics
