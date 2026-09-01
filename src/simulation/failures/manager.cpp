#include <spdlog/spdlog.h>
#include "simulation/constants/public.hpp"
#include "simulation/failures/public/manager.hpp"

namespace failures {

	void FailureManager::check_runtime_failures(const FailureInputs& input) {
		// check collision
		if (input.height_agl.data <= constants::collision_threshold) {
			spdlog::warn("FailureManager: Terrain collision detected");
		}
	}

}
