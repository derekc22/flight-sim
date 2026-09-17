#pragma once
#include "simulation/linearization/public/data/types.hpp"

namespace linearization
{

	/**
	 * @brief Discretizes a continuous local linearization using exact zero-order hold.
	 *
	 * Computes the discrete A and B matrices from the exponential of the augmented
	 * continuous system matrix. The C and D matrices pass through unchanged.
	 *
	 * @param[in] lin_sol Continuous-time local linearization.
	 * @param[in] dt Discretization interval [s].
	 * @return Zero-order-hold discrete local linearization.
	 */
	DiscretizedLocalLinearization discretize(const LocalLinearization& lin_sol, double dt);

	/**
	 * @brief Discretizes a continuous local linearization using forward Euler.
	 *
	 * Computes A_d = I + dt A and B_d = dt B. The C and D matrices pass through
	 * unchanged.
	 *
	 * @param[in] lin_sol Continuous-time local linearization.
	 * @param[in] dt Discretization interval [s].
	 * @return Forward-Euler discrete local linearization.
	 */
	DiscretizedLocalLinearization discretize_euler(const LocalLinearization& lin_sol, double dt);

} // namespace linearization
