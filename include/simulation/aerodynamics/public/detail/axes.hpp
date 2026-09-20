#pragma once
#include "simulation/aerodynamics/public/data/types.hpp"

namespace aerodynamics
{

	/**
	 * @brief Constructs the body-to-stability orientation matrix from angle of attack.
	 *
	 * @param[in] alpha Angle of attack [rad].
	 * @return Body-to-stability orientation matrix CBS [-].
	 */
	dynamics::OrientationMatrix CBS(const AngleOfAttack& alpha);

	/**
	 * @brief Constructs the stability-to-wind orientation matrix from sideslip angle.
	 *
	 * @param[in] beta Sideslip angle [rad].
	 * @return Stability-to-wind orientation matrix CSW [-].
	 */
	dynamics::OrientationMatrix CSW(const SideslipAngle& beta);

} // namespace aerodynamics
