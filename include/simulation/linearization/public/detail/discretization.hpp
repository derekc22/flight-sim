#pragma once
#include "simulation/linearization/public/data/types.hpp"

namespace linearization
{

	/**
	 * @brief Discretizes a continuous local linearization using exact zero-order hold.
	 *
	 * @param[in] lin_sol Continuous-time local linearization.
	 * @param[in] dt Discretization interval [s].
	 * @return Zero-order-hold discrete local linearization.
	 */
	DiscretizedLocalLinearization discretize(const LocalLinearization& lin_sol, double dt);

	/**
	 * @brief Discretizes a continuous local linearization using forward Euler.
	 *
	 * @param[in] lin_sol Continuous-time local linearization.
	 * @param[in] dt Discretization interval [s].
	 * @return Forward-Euler discrete local linearization.
	 */
	DiscretizedLocalLinearization discretize_euler(const LocalLinearization& lin_sol, double dt);

} // namespace linearization
