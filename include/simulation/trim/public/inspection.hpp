#pragma once
#include "simulation/atmospheric/public/data/types.hpp"
#include "simulation/autodiff/public/data/types.hpp"
#include "simulation/trim/public/data/types.hpp"

namespace vehicles
{
	struct Aircraft;
} // namespace vehicles

namespace trim
{

	/**
	 * @brief Builds and solves a trim problem from the aircraft's current condition.
	 *
	 * Uses the current sideslip, roll, pitch, longitudinal and vertical velocity, and yaw rate as targets. Surface
	 * controls start at zero and propulsor inputs start at the midpoint of their limits.
	 *
	 * @param[in] aircraft Aircraft providing the current frames and state.
	 * @param[in] model Automatic-differentiation model and actuator limits used by the solver.
	 * @param[in] wind Body-expressed wind velocity [m/s].
	 * @return Converged trim solution.
	 */
	TrimSolution
	inspect_trim(vehicles::Aircraft& aircraft, const autodiff::AutoDiffModel& model, const atmospheric::Wind& wind);

} // namespace trim
