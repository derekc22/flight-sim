#pragma once
#include <optional>
#include "simulation/aerodynamics/public/data/types.hpp"
#include "simulation/frames/public/data/types.hpp"
#include "simulation/geography/public/data/types.hpp"

namespace json {

	struct ParsedStepOptions : frames::StandardFrameFieldsOptional {
		std::optional<geography::Latitude> lat;
		std::optional<geography::Longitude> lon;
		std::optional<geography::GeometricAltitude> alt;
		std::optional<aerodynamics::AngleOfAttack> alpha;
		std::optional<aerodynamics::SideslipAngle> beta;
	};

} // namespace json
