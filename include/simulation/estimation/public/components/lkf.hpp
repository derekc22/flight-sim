#pragma once
#include "simulation/estimation/public/data/types.hpp"

#include <optional>

namespace estimation
{

	struct LinearKalmanFilter {
		LinearKalmanFilterParameters params;
		std::optional<KalmanState> state;

		LinearKalmanFilter(const LinearKalmanFilterParameters& params);
		dynamics::RigidBodyState step(const LinearKalmanFilterInput& input, double dt);

		KalmanState predict(const linearization::DiscretizedLocalLinearization& lin_sol_k,
			const actuators::ActuatorInputsVector_T<double>& ut_1);
		KalmanState correct(const linearization::OutputJacobian& C, const dynamics::StateVector_T<double>& yt);
	};

} // namespace estimation
