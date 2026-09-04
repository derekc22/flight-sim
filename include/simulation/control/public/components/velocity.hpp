#pragma once
#include "simulation/control/public/data/types.hpp"

namespace control
{

	struct VelocityControl {
		VelocityControlImplementation implementation;

		VelocityControl(const VelocityPIDParameters& params);
		ControlComponentOutput step(const VelocityControlInput& input, double dt);
	};

} // namespace control
