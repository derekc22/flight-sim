#pragma once
#include "simulation/actuators/public/data/types.hpp"
#include "simulation/integrators/public/data/types.hpp"
#include "simulation/operating/public/data/types.hpp"

namespace integrators
{

	RK4Output step_rigid_body_rk4(const dynamics::RigidBodyState& Xt,
		RK4Model& model,
		const operating::OperatingConditions& conditions,
		const actuators::ActuatorInputs_T<double>& u,
		double dt);

}
