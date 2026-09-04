#pragma once
#include "simulation/estimation/public/data/types.hpp"

#include <optional>
#include <tuple>

namespace estimation
{

	struct ExtendedKalmanFilter {
		ExtendedKalmanFilterParameters params;
		std::optional<KalmanState> state;

		ExtendedKalmanFilter(const ExtendedKalmanFilterParameters& params);
		dynamics::RigidBodyState step(const ExtendedKalmanFilterInput& input, double dt);

		std::tuple<KalmanState, linearization::OutputJacobian> predict(const ExtendedKalmanFilterInput& input,
			const actuators::ActuatorInputsVector_T<double>& ut_1,
			double dt);
		KalmanState correct(const dynamics::StateVector_T<double>& yt, const linearization::OutputJacobian& C);
	};

} // namespace estimation
