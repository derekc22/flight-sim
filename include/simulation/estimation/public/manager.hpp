#pragma once
#include "simulation/control/public/data/types.hpp"
#include "simulation/estimation/public/components/ekf.hpp"
#include "simulation/estimation/public/components/lkf.hpp"
#include "simulation/estimation/public/data/types.hpp"

#include <optional>

namespace estimation
{

	struct EstimationManager {
		std::optional<LinearKalmanFilter> linear_kalman_filter;
		std::optional<ExtendedKalmanFilter> extended_kalman_filter;

		/**
		 * @brief Advances the configured estimator and returns its state estimate.
		 *
		 * With no configured filter, the measurement is returned unchanged. If both
		 * filters are present, the extended Kalman filter result takes precedence.
		 *
		 * @param[in] input Measurement, trim, linearization, model, actuator, condition, and timing data.
		 * @return Estimated rigid-body state.
		 */
		EstimationManagerOutput step(const EstimationManagerInput& input);
	};

} // namespace estimation
