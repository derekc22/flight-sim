#pragma once
#include "simulation/estimation/public/data/types.hpp"

#include <optional>
#include <tuple>

namespace estimation
{

	struct ExtendedKalmanEstimator {
		ExtendedKalmanFilterParameters params;
		std::optional<KalmanState> state;

		ExtendedKalmanEstimator(const ExtendedKalmanFilterParameters& params);
		dynamics::RigidBodyState step(const ExtendedKalmanEstimatorInput& input, double dt);

		std::tuple<KalmanState, linearization::OutputJacobian> predict(const ExtendedKalmanEstimatorInput& input,
			const actuators::ActuatorInputsVector_T<double>& ut_1,
			double dt);
		KalmanState correct(const dynamics::StateVector_T<double>& yt,
			const linearization::OutputJacobian& output_jacobian);
	};

} // namespace estimation
