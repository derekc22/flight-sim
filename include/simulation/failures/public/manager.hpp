#pragma once
#include "simulation/failures/public/data/types.hpp"

namespace failures
{

	struct FailureManager {

		FailureManagerOutput step(const FailureManagerInput& input);
	};

} // namespace failures
