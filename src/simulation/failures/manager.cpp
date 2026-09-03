#include <spdlog/spdlog.h>
#include "simulation/constants/public/scalars.hpp"
#include "simulation/failures/public/manager.hpp"

namespace failures {

	FailureManagerOutput FailureManager::step(const FailureManagerInput& input) {
		// check collision
		bool terrain_collision = input.height_agl.data <= constants::collision_threshold;
		if (terrain_collision) {
			spdlog::warn("FailureManager: Terrain collision detected");
		}

		return { .terrain_collision = terrain_collision };
	}

}
