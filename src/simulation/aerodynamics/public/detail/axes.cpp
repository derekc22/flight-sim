#include "simulation/aerodynamics/public/detail/axes.hpp"

#include "simulation/util/public/trig.hpp"

#include <Eigen/Core>

namespace aerodynamics
{

	dynamics::OrientationMatrix CBS(
		const AngleOfAttack& alpha)
	{
		Eigen::Matrix3d CBS;
		double a = alpha.data;
		// clang-format off
		CBS <<    util::cos(a),  0,  util::sin(a),
		                     0,  1,             0,
		         -util::sin(a),  0,  util::cos(a);
		// clang-format on
		return {CBS};
	};

	dynamics::OrientationMatrix CSW(
		const SideslipAngle& beta)
	{
		Eigen::Matrix3d CSW;
		double b = beta.data;
		// clang-format off
		CSW <<    util::cos(b),  util::sin(b),  0,
		         -util::sin(b),  util::cos(b),  0,
		                     0,             0,  1;
		// clang-format on
		return {CSW};
	};

} // namespace aerodynamics
