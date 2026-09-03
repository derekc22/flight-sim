#pragma once
#include "simulation/control/public/data/types.hpp"

namespace control {

	struct AttitudeControl {
		AttitudeControlImplementation implementation;

		AttitudeControl(const AttitudePIDParameters& params);
		AttitudeControl(const DamperPIDParameters& params);
		ControlComponentOutput step(const AttitudeControlInput& input, double dt);
	};

} // namespace control
