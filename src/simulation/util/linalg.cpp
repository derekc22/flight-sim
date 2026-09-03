#include <cmath>
#include "simulation/constants/public/scalars.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/util/public/linalg.hpp"
#include "simulation/util/public/math.hpp"

namespace util {

	Eigen::Matrix3d hat(
	    const Eigen::Vector3d& v)
	{
		Eigen::Matrix3d S;
		S << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
		return S;
	}

	Eigen::Vector3d vee(
	    const Eigen::Matrix3d& S)
	{
		return Eigen::Vector3d(S(2, 1), S(0, 2), S(1, 0));
	}

	Eigen::Vector3d norm(
	    const Eigen::Vector3d& v)
	{
		double n = v.norm();
		if (n < constants::eps)
			return constants::Zero3;
		return v / n;
	}

} // namespace util
