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

		/**
		 * @brief Advances the extended Kalman filter and returns a rigid-body state estimate.
		 *
		 * The first call initializes the state from the measurement and uses @c P0 as
		 * its covariance. Subsequent calls propagate the nonlinear model and perform
		 * a measurement correction.
		 *
		 * @param[in] input Measurement, previous actuator input, model, and operating conditions.
		 * @param[in] dt Estimation interval [s].
		 * @return Rigid-body state estimate with measured position and yaw preserved.
		 */
		dynamics::RigidBodyState step(const ExtendedKalmanFilterInput& input, double dt);

		/**
		 * @brief Predicts the absolute reduced state and covariance using the nonlinear dynamics.
		 *
		 * Propagates the state with forward Euler and propagates the covariance with
		 * the forward-Euler discretized state Jacobian.
		 *
		 * @param[in] input Aircraft model and operating-condition context.
		 * @param[in] ut_1 Previous actual actuator-input vector.
		 * @param[in] dt Estimation interval [s].
		 * @return Predicted absolute reduced-state estimate, covariance, and output Jacobian.
		 */
		std::tuple<KalmanState, linearization::OutputJacobian> predict(const ExtendedKalmanFilterInput& input,
			const actuators::ActuatorInputsVector_T<double>& ut_1,
			double dt);

		/**
		 * @brief Corrects the predicted absolute reduced state with a measurement.
		 *
		 * Uses the Joseph form to update the error covariance.
		 *
		 * @param[in] yt Measured reduced state vector.
		 * @param[in] C Output Jacobian mapping the state estimate to the measurement.
		 * @return Corrected absolute reduced-state estimate and error covariance.
		 */
		KalmanState correct(const dynamics::StateVector_T<double>& yt, const linearization::OutputJacobian& C);
	};

} // namespace estimation
