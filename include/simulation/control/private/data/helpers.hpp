#pragma once
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/guidance/public/data/types.hpp"

namespace control
{

	/**
	 * @brief Packs a linear-quadratic setpoint into the reduced state vector.
	 *
	 * The returned vector is ordered as body translational velocity, body angular
	 * velocity, roll, and pitch. Yaw is excluded from the modeled state.
	 *
	 * @param[in] setpoint Guidance setpoint to pack.
	 * @return Reduced desired-state vector.
	 */
	dynamics::StateVector_T<double> unpack_state(const guidance::LinearQuadraticSetpoint& setpoint);

} // namespace control
