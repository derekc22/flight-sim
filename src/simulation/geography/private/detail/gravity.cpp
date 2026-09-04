#include "simulation/geography/private/detail/gravity.hpp"

#include "simulation/constants/public/scalars.hpp"

#include <Eigen/Dense>

namespace geography
{

	dynamics::Gravity gE(
		const dynamics::Position& pE)
	{
		return {-constants::g_earth * pE.data.normalized()};
	}

} // namespace geography
