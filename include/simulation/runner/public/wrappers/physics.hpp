#pragma once
#include "simulation/constants/public/linalg.hpp"
#include "simulation/runner/public/data/types.hpp"

namespace runner
{

	struct PhysicsWrapper {
		// initialize prior-step net wrench
		dynamics::Wrench WB_net_t_1{.F = dynamics::Force{constants::Zero3}, .M = dynamics::Moment{constants::Zero3}};

		PhysicsWrapperOutput step(const PhysicsWrapperInput& input);
	};

} // namespace runner
