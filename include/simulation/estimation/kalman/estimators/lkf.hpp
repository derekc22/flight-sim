#pragma once
#include "simulation/actuators/shared.hpp"
#include "simulation/dynamics/shared.hpp"
#include "simulation/estimation/kalman/kalman.hpp"
#include "simulation/linearization/shared.hpp"

namespace estimation {

    struct LinearKalmanFilterParameters : KalmanFilterParameters {};

    struct LinearKalmanFilter {
        LinearKalmanFilterParameters params;
        KalmanFilter policy;
        KalmanState state;
        bool initialized = false;

        LinearKalmanFilter(const LinearKalmanFilterParameters& params);
        EstimationOutput step(const KalmanFilterInput& estimator_input);

        dynamics::StateVector_T<double> make_measurement_deviation(const KalmanFilterInput& estimator_input);
        actuators::ActuatorInputsVector_T<double> make_input_deviation(const KalmanFilterInput& estimator_input);
        dynamics::RigidBodyState pack_state_estimate(const KalmanFilterInput& estimator_input, const dynamics::StateVector_T<double>& zN_t_deviation);
    };
}
