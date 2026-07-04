#include <spdlog/spdlog.h>
#include "simulation/failures/public.hpp"
#include "simulation/constants/public.hpp"

namespace failures {

	void FailureManager::check_runtime_failures(const FailureInputs& input) {
		// check collision
		if (input.height_agl.data <= constants::collision_threshold) {
			spdlog::warn("FailureManager: Aircraft has collided with the terrain");
		}
	}

}

