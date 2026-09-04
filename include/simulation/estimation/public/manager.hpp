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

		EstimationManagerOutput step(const EstimationManagerInput& input);
	};

} // namespace estimation
