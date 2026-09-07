#pragma once
#include "simulation/aerodynamics/public/data/types.hpp"

namespace aerodynamics
{

	struct SurfaceEffector {
		actuators::SurfaceActuatorID actuator_id;
		double dCL;
		double dCD;
		double dCM;

		template <typename T> SurfaceCoefficients_T<T> step(const actuators::SurfaceActuatorInputs_T<T>& u);
	};

} // namespace aerodynamics

#include "simulation/aerodynamics/public/components/effector.tpp"
