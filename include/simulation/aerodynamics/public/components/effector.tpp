#pragma once
#include "simulation/actuators/public/data/helpers.hpp"
#include "simulation/aerodynamics/public/components/effector.hpp"
#include "simulation/util/public/cppad.hpp"
#include "simulation/util/public/math.hpp"

namespace aerodynamics
{

	template <typename T>
	SurfaceCoefficients_T<T> SurfaceEffector::step(
		const actuators::SurfaceActuatorInputs_T<T>& u)
	{
		const T actuator_input = actuators::get_surface_actuator_input_T<T>(actuator_id, u);
		return {
			.CL = T(dCL) * actuator_input,
			.CD = T(dCD) * util::smooth_abs(actuator_input),
			.CM = T(dCM) * actuator_input
		};
	}

} // namespace aerodynamics
