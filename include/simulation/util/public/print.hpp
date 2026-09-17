#pragma once
#include "simulation/aerodynamics/public/data/types.hpp"
#include "simulation/atmospheric/public/data/types.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/geography/public/data/types.hpp"

#include <Eigen/Dense>
#include <string>

namespace util
{

	/**
	 * @brief Formats a named three-dimensional vector for diagnostic output.
	 *
	 * Formats each component with fixed-point notation and three digits after the
	 * decimal point, appends @p unit, and terminates the text with a newline.
	 *
	 * @param[in] name Label printed before the vector.
	 * @param[in] x Vector to format.
	 * @param[in] unit Unit label printed after the vector.
	 * @return Formatted single-line vector description.
	 */
	std::string print_vec(const char* name, const Eigen::Vector3d& x, const char* unit);

	/**
	 * @brief Logs a formatted snapshot of the current simulation state.
	 *
	 * Reports time, position, intrinsic ZYX Euler angles, translational and angular
	 * velocity, body gravity, inertial wind, geographic coordinates, angle of
	 * attack, and sideslip. Angular quantities are converted to degrees for output.
	 *
	 * @param[in] t Simulation-step index.
	 * @param[in] Xt Current rigid-body state.
	 * @param[in] geo Current geographic state.
	 * @param[in] aero Current aerodynamic state.
	 * @param[in] windI Current inertial-expressed wind velocity [m/s].
	 */
	void print_state(int t,
		const dynamics::RigidBodyState& Xt,
		const geography::GeographicState& geo,
		const aerodynamics::AerodynamicState& aero,
		const atmospheric::Wind& windI);

} // namespace util
