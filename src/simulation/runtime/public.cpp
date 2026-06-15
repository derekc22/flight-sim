#include <stdexcept>
#include "simulation/runtime/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/util/public.hpp"

namespace runtime {

	void RuntimeProperties::check_runtime_failures(const RuntimeFailureInputs& failure_inputs) {
		// check collision
		double altitude_agl = util::abs(failure_inputs.altitude) - failure_inputs.ground_elev;
		if (altitude_agl <= constants::collision_threshold) {
			throw std::runtime_error("runtime::RuntimeProperties::check_runtime_failures: aircraft has collided with the terrain");
		}
	}

}

