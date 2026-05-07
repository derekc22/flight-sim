#pragma once
#include "simulation/estimation/kalman/kalman.hpp"
#include "simulation/linearization/linearization.hpp"
#include "simulation/trim/trim.hpp"

namespace estimation {

    struct LinearKalmanFilterParameters : KalmanFilterParameters {};

    struct LinearKalmanFilter {
        LinearKalmanFilterParameters params;
        KalmanFilter policy;
        KalmanState state;
        bool initialized = false;

        LinearKalmanFilter(const LinearKalmanFilterParameters& params);
        EstimationOutput step(const KalmanFilterInput& estimator_input);

        Eigen::VectorXd make_measurement_deviation(const KalmanFilterInput& estimator_input);
        Eigen::VectorXd make_input_deviation(const KalmanFilterInput& estimator_input);
        dynamics::RigidBodyState pack_state_estimate(const KalmanFilterInput& estimator_input, const Eigen::VectorXd& x_est_deviation);
    };
}
