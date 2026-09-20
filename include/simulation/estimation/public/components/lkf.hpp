#pragma once
#include "simulation/estimation/public/data/types.hpp"

#include <optional>

namespace estimation
{

	struct LinearKalmanFilter {
		LinearKalmanFilterParameters params;
		std::optional<KalmanState> state;

		LinearKalmanFilter(const LinearKalmanFilterParameters& params);

		/**
		 * @brief Advances the linear Kalman filter and returns a rigid-body state estimate.
		 *
		 * @param[in] input Measurement, operating point, local linearization, and previous actuator input.
		 * @param[in] dt Estimation interval [s].
		 * @return Rigid-body estimate with operating-point offsets restored and measured position and yaw preserved.
		 */
		dynamics::RigidBodyState step(const LinearKalmanFilterInput& input, double dt);

		/**
		 * @brief Predicts the deviation-state estimate and covariance.
		 *
		 * @param[in] lin_sol_k Discrete local state-space linearization.
		 * @param[in] ut_1 Previous actuator-input deviation from the operating point.
		 * @return Predicted deviation-state estimate and error covariance.
		 */
		KalmanState predict(const linearization::DiscretizedLocalLinearization& lin_sol_k,
			const actuators::ActuatorInputsVector_T<double>& ut_1);

		/**
		 * @brief Corrects the predicted deviation state with a measurement.
		 *
		 * @param[in] C Output Jacobian mapping the state estimate to the measurement.
		 * @param[in] yt Measured state deviation from the operating point.
		 * @return Corrected deviation-state estimate and error covariance.
		 */
		KalmanState correct(const linearization::OutputJacobian& C, const dynamics::StateVector_T<double>& yt);
	};

} // namespace estimation
