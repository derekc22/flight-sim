#pragma once
#include "simulation/actuators/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/estimation/kalman/public.hpp"
#include "simulation/linearization/public.hpp"

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
