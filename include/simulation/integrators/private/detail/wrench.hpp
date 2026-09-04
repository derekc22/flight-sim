#pragma once
#include "simulation/actuators/public/data/helpers.hpp"
#include "simulation/integrators/public/data/types.hpp"
#include "simulation/operating/public/data/types.hpp"

namespace integrators
{

	WrenchEvaluation compute_net_wrench(const dynamics::RigidBodyState& Xt,
		RK4Model& model,
		const operating::OperatingConditions& conditions,
		const actuators::ActuatorInputs_T<double>& u,
		double dt);

}
