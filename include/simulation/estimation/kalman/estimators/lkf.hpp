#pragma once
#include "simulation/estimation/kalman/kalman.hpp"
#include "simulation/linearization/linearization.hpp"
#include "simulation/types/types.hpp"

namespace estimation {

    struct LinearKalmanFilterParameters : KalmanFilterParameters {};

    struct LinearKalmanFilter {
        LinearKalmanFilterParameters params;
        KalmanFilter policy;
        KalmanState state;
        bool initialized = false;

        LinearKalmanFilter(const LinearKalmanFilterParameters& params);
        EstimationOutput step(const KalmanFilterInput& estimator_input);

        types::StateVector_T<double> make_measurement_deviation(const KalmanFilterInput& estimator_input);
        types::ActuatorInputsVector_T<double> make_input_deviation(const KalmanFilterInput& estimator_input);
        dynamics::RigidBodyState pack_state_estimate(const KalmanFilterInput& estimator_input, const types::StateVector_T<double>& zN_t_deviation);
    };
}
