#pragma once

namespace util
{

	/**
	 * @brief Converts an angle from degrees to radians.
	 *
	 * @param[in] deg Angle [deg].
	 * @return Angle [rad].
	 */
	double deg_to_rad(double deg);

	/**
	 * @brief Converts an angle from radians to degrees.
	 *
	 * @param[in] rad Angle [rad].
	 * @return Angle [deg].
	 */
	double rad_to_deg(double rad);

	double csc(double x);

	double sec(double x);

	double cot(double x);

	double sin(double x);

	double cos(double x);

	double tan(double x);

	double asin(double x);

	double acos(double x);

	double atan2(double y, double x);

} // namespace util
