#include <stdexcept>
#include "simulation/runtime/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/util/public.hpp"

namespace runtime {

	void RuntimeProperties::check_runtime_failures(const RuntimeFailureInputs& input) {
		// check collision
		if (input.height_agl.data <= constants::collision_threshold) {
			throw std::runtime_error("runtime::RuntimeProperties::check_runtime_failures: aircraft has collided with the terrain");
		}
	}

}

