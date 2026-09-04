#pragma once
#include "simulation/aerodynamics/public/data/types.hpp"

namespace aerodynamics
{

	dynamics::OrientationMatrix CBS(const AngleOfAttack& alpha);
	dynamics::OrientationMatrix CSW(const SideslipAngle& beta);

} // namespace aerodynamics
