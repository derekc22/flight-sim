#pragma once

namespace util
{

	/**
	 * @brief Converts speed from knots to metres per second.
	 *
	 * @param[in] kts Speed [kn].
	 * @return Speed [m/s].
	 */
	double kts_to_mps(double kts);

	/**
	 * @brief Converts speed from metres per second to knots.
	 *
	 * @param[in] ms Speed [m/s].
	 * @return Speed [kn].
	 */
	double ms_to_kts(double ms);

	/**
	 * @brief Converts length from feet to metres.
	 *
	 * @param[in] ft Length [ft].
	 * @return Length [m].
	 */
	double ft_to_m(double ft);

	/**
	 * @brief Converts length from metres to feet.
	 *
	 * @param[in] m Length [m].
	 * @return Length [ft].
	 */
	double m_to_ft(double m);

} // namespace util
