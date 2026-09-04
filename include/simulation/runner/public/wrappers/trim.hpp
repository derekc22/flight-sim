#pragma once
#include "simulation/runner/public/data/types.hpp"

namespace runner
{

	struct TrimWrapper {
		trim::TrimSolution trim_sol;

		TrimWrapperOutput step(const TrimWrapperInput& input);
	};

} // namespace runner
