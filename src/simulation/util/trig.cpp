#include <cmath>
#include "simulation/constants/public/scalars.hpp"
#include "simulation/util/public/trig.hpp"

namespace util {

	double deg_to_rad(
	    double deg)
	{
		return deg * (constants::pi / 180.0);
	}

	double rad_to_deg(
	    double rad)
	{
		return rad * (180.0 / constants::pi);
	}

	double csc(
	    double x)
	{
		return 1.0 / std::sin(x);
	}

	double sec(
	    double x)
	{
		return 1.0 / std::cos(x);
	}

	double cot(
	    double x)
	{
		return std::cos(x) / std::sin(x);
	}

	double sin(
	    double x)
	{
		return std::sin(x);
	}

	double cos(
	    double x)
	{
		return std::cos(x);
	}

	double tan(
	    double x)
	{
		return std::tan(x);
	}

	double asin(
	    double x)
	{
		return std::asin(x);
	}

	double acos(
	    double x)
	{
		return std::acos(x);
	}

	double atan2(
	    double y,
	    double x)
	{
		return std::atan2(y, x);
	}

} // namespace util
