#pragma once
#include "simulation/aerodynamics/public/data/types.hpp"

namespace aerodynamics
{

	struct SurfaceEffector {
		actuators::SurfaceActuatorID actuator_id;
		// Control derivatives
		double dCL;
		double dCD;
		double dCM;

		/**
		 * @brief Computes the aerodynamic-coefficient contribution from this control effector.
		 *
		 * @tparam T Scalar type used for the computation.
		 * @param[in] u Surface-actuator inputs.
		 * @return Lift, drag, and pitching-moment coefficient contributions [-].
		 */
		template <typename T> SurfaceCoefficients_T<T> step(const actuators::SurfaceActuatorInputs_T<T>& u);
	};

} // namespace aerodynamics

#include "simulation/aerodynamics/public/components/effector.tpp"
