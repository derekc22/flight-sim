#pragma once
#include "simulation/control/public/data/types.hpp"
#include "simulation/estimation/public/components/extended_kalman.hpp"
#include "simulation/estimation/public/components/linear_kalman.hpp"
#include "simulation/estimation/public/data/types.hpp"

#include <optional>

namespace estimation
{

	struct EstimationManager {
		std::optional<LinearKalmanEstimator> linear_kalman_estimator;
		std::optional<ExtendedKalmanEstimator> extended_kalman_estimator;

		EstimationManagerOutput step(const EstimationManagerInput& input);
	};

} // namespace estimation
