#include <stdexcept>
#include "simulation/failures/public.hpp"
#include "simulation/constants/public.hpp"

namespace failures {

	void FailureManager::check_runtime_failures(const FailureInputs& input) {
		// check collision
		if (input.height_agl.data <= constants::collision_threshold) {
			throw std::runtime_error("failures::FailureManager::check_runtime_failures: aircraft has collided with the terrain");
		}
	}

}

